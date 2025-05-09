#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>
#include <cudaHeaderFiles.h>

#include <Serialization.hpp>

#include "main.cuh"

void Host_PointCloud::Initialize(unsigned int numberOfPoints)
{
    this->numberOfPoints = numberOfPoints;

    points = new Eigen::Vector3f[numberOfPoints];
    normals = new Eigen::Vector3f[numberOfPoints];
    colors = new Eigen::Vector3b[numberOfPoints];
}

void Host_PointCloud::Terminate()
{
    delete[] points;
    delete[] normals;
    delete[] colors;
}

void Device_PointCloud::Initialize(unsigned int numberOfPoints)
{
    this->numberOfPoints = numberOfPoints;

    cudaMalloc(&points, sizeof(Eigen::Vector3f) * numberOfPoints);
    cudaMalloc(&normals, sizeof(Eigen::Vector3f) * numberOfPoints);
    cudaMalloc(&colors, sizeof(Eigen::Vector3b) * numberOfPoints);
}

void Device_PointCloud::Terminate()
{
    cudaFree(points);
    cudaFree(normals);
    cudaFree(colors);
}

__device__ __host__ inline float hashToFloat(uint32_t seed)
{
    seed ^= seed >> 13;
    seed *= 0x5bd1e995;
    seed ^= seed >> 15;
    return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
};

__device__ __host__ inline size_t voxel_hash(int3 coord, size_t tableSize)
{
    return ((size_t)(coord.x * 73856093) ^ (coord.y * 19349663) ^ (coord.z * 83492791)) % tableSize;
}

void HashMap::Initialize()
{
    cudaMalloc(&info.d_hashTable, sizeof(HashMapVoxel) * info.capacity);
    cudaMemset(info.d_hashTable, 0, sizeof(HashMapVoxel) * info.capacity);

    cudaMalloc(&info.d_numberOfOccupiedVoxels, sizeof(unsigned int) * info.capacity);
    cudaMemset(info.d_numberOfOccupiedVoxels, 0, sizeof(unsigned int) * info.capacity);

    cudaMalloc(&info.d_occupiedVoxelIndices, sizeof(int3) * info.capacity);
    cudaMemset(info.d_occupiedVoxelIndices, 0, sizeof(int3) * info.capacity);
}

void HashMap::Terminate()
{
    cudaFree(info.d_hashTable);
    cudaFree(info.d_numberOfOccupiedVoxels);
    cudaFree(info.d_occupiedVoxelIndices);
}

__global__ void Kernel_InsertPoints(HashMapInfo info, Device_PointCloud pointCloud)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pointCloud.numberOfPoints) return;

    auto p = pointCloud.points[idx];
    auto n = pointCloud.normals[idx].normalized();
    auto c = pointCloud.colors[idx];

    int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));

    size_t h = voxel_hash(coord, info.capacity);
    for (int i = 0; i < info.maxProbe; ++i) {
        size_t slot = (h + i) % info.capacity;
        int prev = atomicCAS(&(info.d_hashTable[slot].label), 0, slot);

        if (prev == 0) {
            // 새로운 슬롯에 삽입
            info.d_hashTable[slot].coord = coord;
            info.d_hashTable[slot].position = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);
            info.d_hashTable[slot].normal = Eigen::Vector3f(n.x(), n.y(), n.z());
            info.d_hashTable[slot].color = Eigen::Vector3b(c.x(), c.y(), c.z());

            auto oldIndex = atomicAdd(info.d_numberOfOccupiedVoxels, 1);
            info.d_occupiedVoxelIndices[oldIndex] = coord;
            return;
        }
        else {
            int3 existing = info.d_hashTable[slot].coord;
            if (existing.x == coord.x && existing.y == coord.y && existing.z == coord.z) {
                info.d_hashTable[slot].normal = (info.d_hashTable[slot].normal + Eigen::Vector3f(n.x(), n.y(), n.z())).normalized();
                info.d_hashTable[slot].color = Eigen::Vector3b(c.x(), c.y(), c.z());
                return;
            }
        }
    }
}

void HashMap::InsertHPoints(Host_PointCloud pointCloud)
{
    Device_PointCloud d_pointCloud;
    d_pointCloud.Initialize(pointCloud.numberOfPoints);

    cudaMemcpy(d_pointCloud.points, pointCloud.points, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pointCloud.normals, pointCloud.normals, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pointCloud.colors, pointCloud.colors, sizeof(Eigen::Vector3b) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);

    InsertDPoints(d_pointCloud);

    d_pointCloud.Terminate();
}

void HashMap::InsertDPoints(Device_PointCloud pointCloud)
{
    unsigned int blockSize = 256;
    unsigned int gridOccupied = (pointCloud.numberOfPoints + blockSize - 1) / blockSize;

    Kernel_InsertPoints << <gridOccupied, blockSize >> > (info, pointCloud);

    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    alog("numberOfOccupiedVoxels : %d\n", numberOfOccupiedVoxels);

    cudaDeviceSynchronize();
}

__global__ void Kernel_ComputeNormalDivergence(HashMapInfo info)
{
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= *info.d_numberOfOccupiedVoxels) return;

    int3 coord = info.d_occupiedVoxelIndices[threadid];
    size_t h = voxel_hash(coord, info.capacity);

    HashMapVoxel* voxel = nullptr;

    // 현재 voxel 찾기
    for (int i = 0; i < info.maxProbe; ++i)
    {
        size_t slot = (h + i) % info.capacity;
        if (info.d_hashTable[slot].label == 0) break;

        if (info.d_hashTable[slot].coord.x == coord.x &&
            info.d_hashTable[slot].coord.y == coord.y &&
            info.d_hashTable[slot].coord.z == coord.z)
        {
            voxel = &info.d_hashTable[slot];
            break;
        }
    }

    if (!voxel) return;

    const Eigen::Vector3f n0 = voxel->normal.normalized();

    const int3 offsets[6] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };

    float divergenceSum = 0.0f;
    int validNeighbors = 0;

    for (int ni = 0; ni < 6; ++ni)
    {
        int3 ncoord = make_int3(coord.x + offsets[ni].x,
            coord.y + offsets[ni].y,
            coord.z + offsets[ni].z);

        size_t nh = voxel_hash(ncoord, info.capacity);

        for (int j = 0; j < info.maxProbe; ++j)
        {
            size_t slot = (nh + j) % info.capacity;
            auto& neighbor = info.d_hashTable[slot];

            if (neighbor.label == 0) break;

            if (neighbor.coord.x == ncoord.x &&
                neighbor.coord.y == ncoord.y &&
                neighbor.coord.z == ncoord.z)
            {
                Eigen::Vector3f n1 = neighbor.normal.normalized();
                float dot = fminf(fmaxf(n0.dot(n1), -1.0f), 1.0f);
                float angle = acosf(dot); // radian
                divergenceSum += angle;
                ++validNeighbors;
                break;
            }
        }
    }

    if (validNeighbors > 0)
        voxel->divergence = divergenceSum / validNeighbors;
    else
        voxel->divergence = 0.0f;
}

void HashMap::ComputeNormalDivergence()
{
    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

    Kernel_ComputeNormalDivergence << <gridOccupied, blockSize >> > (info);

    cudaDeviceSynchronize();
}

__global__ void Kernel_Serialize(HashMapInfo info, Device_PointCloud pointCloud)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pointCloud.numberOfPoints) return;

    int3 coord = info.d_occupiedVoxelIndices[idx];
    size_t h = voxel_hash(coord, info.capacity);

    for (unsigned int i = 0; i < info.maxProbe; ++i)
    {
        size_t slot = (h + i) % info.capacity;
        auto& voxel = info.d_hashTable[slot];

        if (0 == voxel.label) return;

        if (voxel.coord.x == coord.x &&
            voxel.coord.y == coord.y &&
            voxel.coord.z == coord.z)
        {
            pointCloud.points[idx] = voxel.position;
            pointCloud.normals[idx] = voxel.normal;
            pointCloud.colors[idx] = voxel.color;
            return;
        }
    }
}

void HashMap::SerializeToPLY(const std::string& filename)
{
    PLYFormat ply;

    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    Device_PointCloud pointCloud;
    pointCloud.Initialize(numberOfOccupiedVoxels);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

    Kernel_Serialize << <gridOccupied, blockSize >> > (info, pointCloud);

    cudaDeviceSynchronize();

    Eigen::Vector3f* h_points = new Eigen::Vector3f[numberOfOccupiedVoxels];
    Eigen::Vector3f* h_normals = new Eigen::Vector3f[numberOfOccupiedVoxels];
    Eigen::Vector3b* h_colors = new Eigen::Vector3b[numberOfOccupiedVoxels];

    cudaMemcpy(h_points, pointCloud.points, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_normals, pointCloud.normals, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_colors, pointCloud.colors, sizeof(Eigen::Vector3b) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < numberOfOccupiedVoxels; i++)
    {
        auto& p = h_points[i];
        auto& n = h_normals[i];
        auto& c = h_colors[i];

        ply.AddPoint(p.x(), p.y(), p.z());
        ply.AddNormal(n.x(), n.y(), n.z());
        ply.AddColor(c.x() / 255.0f, c.y() / 255.0f, c.z() / 255.0f);
    }

    ply.Serialize(filename);

    pointCloud.Terminate();

    delete[] h_points;
    delete[] h_normals;
    delete[] h_colors;
}

__global__ void Kernel_SerializeColoringByLabel(HashMapInfo info, Device_PointCloud pointCloud)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pointCloud.numberOfPoints) return;

    int3 coord = info.d_occupiedVoxelIndices[idx];
    size_t h = voxel_hash(coord, info.capacity);

    for (unsigned int i = 0; i < info.maxProbe; ++i)
    {
        size_t slot = (h + i) % info.capacity;
        auto& voxel = info.d_hashTable[slot];

        if (0 == voxel.label) return;

        if (voxel.coord.x == coord.x &&
            voxel.coord.y == coord.y &&
            voxel.coord.z == coord.z)
        {
            pointCloud.points[idx] = voxel.position;
            pointCloud.normals[idx] = voxel.normal;

            float r = hashToFloat(voxel.label * 3 + 0);
            float g = hashToFloat(voxel.label * 3 + 1);
            float b = hashToFloat(voxel.label * 3 + 2);

            pointCloud.colors[idx] = Eigen::Vector3b(r * 255.0f, g * 255.0f, b * 255.0f);

            return;
        }
    }
}

void HashMap::SerializeColoringByLabel(const std::string& filename)
{
    PLYFormat ply;

    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    alog("numberOfOccupiedVoxels : %d\n", numberOfOccupiedVoxels);

    Device_PointCloud pointCloud;
    pointCloud.Initialize(numberOfOccupiedVoxels);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

    Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (info, pointCloud);

    cudaDeviceSynchronize();

    Eigen::Vector3f* h_points = new Eigen::Vector3f[numberOfOccupiedVoxels];
    Eigen::Vector3f* h_normals = new Eigen::Vector3f[numberOfOccupiedVoxels];
    Eigen::Vector3b* h_colors = new Eigen::Vector3b[numberOfOccupiedVoxels];

    cudaMemcpy(h_points, pointCloud.points, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_normals, pointCloud.normals, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_colors, pointCloud.colors, sizeof(Eigen::Vector3b) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < numberOfOccupiedVoxels; i++)
    {
        auto& p = h_points[i];
        auto& n = h_normals[i];
        auto& c = h_colors[i];

        ply.AddPoint(p.x(), p.y(), p.z());
        ply.AddNormal(n.x(), n.y(), n.z());
        ply.AddColor(c.x() / 255.0f, c.y() / 255.0f, c.z() / 255.0f);
    }

    ply.Serialize(filename);

    pointCloud.Terminate();

    delete[] h_points;
    delete[] h_normals;
    delete[] h_colors;
}

__global__ void Kernel_SerializeColorByDivergence(HashMapInfo info, Device_PointCloud pointCloud, float maxDivergence)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= *info.d_numberOfOccupiedVoxels) return;

    int3 coord = info.d_occupiedVoxelIndices[idx];
    size_t h = voxel_hash(coord, info.capacity);

    for (int i = 0; i < info.maxProbe; ++i)
    {
        size_t slot = (h + i) % info.capacity;
        auto& voxel = info.d_hashTable[slot];

        if (voxel.label == 0) return;

        if (voxel.coord.x == coord.x &&
            voxel.coord.y == coord.y &&
            voxel.coord.z == coord.z)
        {
            pointCloud.points[idx] = voxel.position;
            pointCloud.normals[idx] = voxel.normal;

            float t = fminf(voxel.divergence / maxDivergence, 1.0f); // normalize
            // Heatmap color (Blue → Green → Red)
            float r = fminf(fmaxf(2.0f * t - 1.0f, 0.0f), 1.0f);
            float g = 1.0f - fabsf(2.0f * t - 1.0f);
            float b = fminf(fmaxf(1.0f - 2.0f * t, 0.0f), 1.0f);

            pointCloud.colors[idx] = Eigen::Vector3b(
                static_cast<unsigned char>(r * 255),
                static_cast<unsigned char>(g * 255),
                static_cast<unsigned char>(b * 255));
            return;
        }
    }
}

void HashMap::SerializeColoringByDivergence(const std::string& filename)
{
    PLYFormat ply;

    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    alog("numberOfOccupiedVoxels : %d\n", numberOfOccupiedVoxels);

    Device_PointCloud pointCloud;
    pointCloud.Initialize(numberOfOccupiedVoxels);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

    float maxDivergence = 0.5f;
    Kernel_SerializeColorByDivergence << <gridOccupied, blockSize >> > (info, pointCloud, maxDivergence);

    cudaDeviceSynchronize();

    Eigen::Vector3f* h_points = new Eigen::Vector3f[numberOfOccupiedVoxels];
    Eigen::Vector3f* h_normals = new Eigen::Vector3f[numberOfOccupiedVoxels];
    Eigen::Vector3b* h_colors = new Eigen::Vector3b[numberOfOccupiedVoxels];

    cudaMemcpy(h_points, pointCloud.points, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_normals, pointCloud.normals, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_colors, pointCloud.colors, sizeof(Eigen::Vector3b) * numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < numberOfOccupiedVoxels; i++)
    {
        auto& p = h_points[i];
        auto& n = h_normals[i];
        auto& c = h_colors[i];

        ply.AddPoint(p.x(), p.y(), p.z());
        ply.AddNormal(n.x(), n.y(), n.z());
        ply.AddColor(c.x() / 255.0f, c.y() / 255.0f, c.z() / 255.0f);
    }

    ply.Serialize(filename);

    pointCloud.Terminate();

    delete[] h_points;
    delete[] h_normals;
    delete[] h_colors;
}

//__device__ __forceinline__ unsigned int FindRootVoxel(HashMapInfo info, unsigned int idx)
//{
//    while (true)
//    {
//        unsigned int parent = info.d_hashTable[idx].label;
//        unsigned int grand = info.d_hashTable[parent].label;
//        if (parent == idx) break;
//        if (parent != grand) info.d_hashTable[idx].label = grand;
//        idx = parent;
//    }
//    return idx;
//}

__device__ __forceinline__ unsigned int FindRootVoxel(HashMapInfo info, unsigned int idx)
{
    while (info.d_hashTable[idx].label != idx)
    {
        unsigned int parent = info.d_hashTable[idx].label;
        unsigned int grandparent = info.d_hashTable[parent].label;
        if (parent != grandparent)
            info.d_hashTable[idx].label = grandparent;

        idx = info.d_hashTable[idx].label;
    }
    return idx;
}

//__device__ __forceinline__ void UnionVoxel(HashMapInfo info, unsigned int a, unsigned int b)
//{
//    unsigned int rootA = FindRootVoxel(info, a);
//    unsigned int rootB = FindRootVoxel(info, b);
//
//    auto nA = info.d_hashTable[rootA].normal;
//    auto nB = info.d_hashTable[rootB].normal;
//
//    if (0.5f > nA.dot(nB)) return;
//
//    if (rootA != rootB)
//    {
//        if (rootA < rootB)
//        {
//            atomicMin(&info.d_hashTable[rootB].label, rootA);
//        }
//        else
//        {
//            atomicMin(&info.d_hashTable[rootA].label, rootB);
//        }
//    }
//}

//__device__ __forceinline__ void UnionVoxel(HashMapInfo info, unsigned int a, unsigned int b)
//{
//    unsigned int rootA = FindRootVoxel(info, a);
//    unsigned int rootB = FindRootVoxel(info, b);
//
//    if (rootA == rootB) return;
//
//    auto nA = info.d_hashTable[rootA].normal.normalized();
//    auto nB = info.d_hashTable[rootB].normal.normalized();
//
//    if (nA.dot(nB) < 0.5f) return;
//
//    auto& voxelA = info.d_hashTable[rootA];
//    auto& voxelB = info.d_hashTable[rootB];
//
//    const float maxDivergence = 0.3f; // radians 단위로 약 17도
//    if (voxelA.divergence > maxDivergence || voxelB.divergence > maxDivergence)
//        return;
//
//    if (rootA < rootB)
//    {
//        atomicMin(&info.d_hashTable[rootB].label, rootA);
//    }
//    else
//    {
//        atomicMin(&info.d_hashTable[rootA].label, rootB);
//    }
//}

__device__ __forceinline__ void UnionVoxel(HashMapInfo info, unsigned int a, unsigned int b)
{
    unsigned int rootA = FindRootVoxel(info, a);
    unsigned int rootB = FindRootVoxel(info, b);

    if (rootA == rootB) return;

    auto& voxelA = info.d_hashTable[rootA];
    auto& voxelB = info.d_hashTable[rootB];

    Eigen::Vector3f nA = voxelA.normal;
    Eigen::Vector3f nB = voxelB.normal;

    //const float minDot = 0.425f;
    const float minDot = 0.1f;
    if (nA.dot(nB) < minDot) return;

    //// 조건 2: 주변 smoothness 조건
    //const float maxDivergence = 0.2f; // radians (~17도)
    //if (voxelA.divergence > maxDivergence || voxelB.divergence > maxDivergence)
    //    return;

    //// (선택) 조건 3: divergence 불일치 방지
    //if (fabsf(voxelA.divergence - voxelB.divergence) > 0.15f)
    //    return;

    // 병합: label을 더 작은 쪽으로
    if (rootA < rootB)
    {
        atomicMin(&voxelB.label, rootA);
    }
    else
    {
        atomicMin(&voxelA.label, rootB);
    }
}

__global__ void Kernel_InterVoxelHashMerge26Way(HashMapInfo info)
{
    unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
    if (threadid >= *info.d_numberOfOccupiedVoxels) return;

    int3 coord = info.d_occupiedVoxelIndices[threadid];
    size_t h = voxel_hash(coord, info.capacity);

    // Linear probing to find the voxel in the hash table
    for (int i = 0; i < info.maxProbe; ++i)
    {
        size_t probe = (h + i) % info.capacity;
        auto& entry = info.d_hashTable[probe];

        if (entry.coord.x == coord.x &&
            entry.coord.y == coord.y &&
            entry.coord.z == coord.z)
        {
            if (entry.label == 0) return; // This should never happen unless corrupted

            const int3 offsets[26] =
            {
                {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
                {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
                {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
                {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
                {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
                {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
            };

#pragma unroll
            for (int ni = 0; ni < 26; ++ni)
            {
                int3 neighborCoord = make_int3(
                    coord.x + offsets[ni].x,
                    coord.y + offsets[ni].y,
                    coord.z + offsets[ni].z);

                size_t nh = voxel_hash(neighborCoord, info.capacity);

                for (int j = 0; j < info.maxProbe; ++j)
                {
                    size_t nprobe = (nh + j) % info.capacity;
                    auto& neighbor = info.d_hashTable[nprobe];

                    if (neighbor.label == 0)
                        break;  // Stop probing if slot is empty (assuming no tombstones)

                    if (neighbor.coord.x == neighborCoord.x &&
                        neighbor.coord.y == neighborCoord.y &&
                        neighbor.coord.z == neighborCoord.z)
                    {
                        UnionVoxel(info, probe, nprobe);
                        break;
                    }
                }
            }

            break;  // Stop probing once the original voxel is found
        }

        // Optional: break early if we hit an empty slot (assumes no deletion holes)
        if (entry.label == 0)
            break;
    }
}

__global__ void Kernel_CompressVoxelHashLabels(HashMapInfo info)
{
    unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
    if (threadid >= *info.d_numberOfOccupiedVoxels) return;

    int3 coord = info.d_occupiedVoxelIndices[threadid];

    size_t h = voxel_hash(coord, info.capacity);
    for (int i = 0; i < info.maxProbe; ++i)
    {
        size_t probe = (h + i) % info.capacity;
        auto& voxel = info.d_hashTable[probe];

        if (0 == voxel.label) break;

        voxel.label = FindRootVoxel(info, voxel.label);
    }
}

__global__ void Kernel_GetLabels(HashMapInfo info, Eigen::Vector3f* points, unsigned int* labels, unsigned int numberOfPoints)
{
    unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
    if (threadid >= numberOfPoints) return;

    auto& p = points[threadid];
    int3 coord = make_int3(
        floorf(p.x() / info.voxelSize),
        floorf(p.y() / info.voxelSize),
        floorf(p.z() / info.voxelSize));

    size_t h = voxel_hash(coord, info.capacity);
    labels[threadid] = UINT_MAX;

    for (int i = 0; i < info.maxProbe; ++i)
    {
        size_t probe = (h + i) % info.capacity;
        if (info.d_hashTable[probe].label == 0) break;

        auto& voxel = info.d_hashTable[probe];

        if (voxel.coord.x == coord.x &&
            voxel.coord.y == coord.y &&
            voxel.coord.z == coord.z)
        {
            labels[threadid] = voxel.label;
            return;
        }
    }
}

vector<unsigned int> HashMap::Clustering(Host_PointCloud pointCloud)
{
    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    alog("numberOfOccupiedVoxels : %d\n", numberOfOccupiedVoxels);

    {
        unsigned int blockSize = 256;
        unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

        Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (info);

        cudaDeviceSynchronize();
    }

    {
        unsigned int blockSize = 256;
        unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
        
        Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (info);

        cudaDeviceSynchronize();
    }

    vector<unsigned int> labels;
    labels.resize(pointCloud.numberOfPoints);
    {
        Eigen::Vector3f* d_points = nullptr;
        cudaMalloc(&d_points, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints);
        cudaMemcpy(d_points, pointCloud.points, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);

        unsigned int* d_labels = nullptr;
        cudaMalloc(&d_labels, sizeof(unsigned int) * pointCloud.numberOfPoints);

        unsigned int blockSize = 256;
        unsigned int gridOccupied = (pointCloud.numberOfPoints + blockSize - 1) / blockSize;

        Kernel_GetLabels << <gridOccupied, blockSize >> > (info, d_points, d_labels, pointCloud.numberOfPoints);

        cudaDeviceSynchronize();

        cudaMemcpy(labels.data(), d_labels, sizeof(unsigned int) * pointCloud.numberOfPoints, cudaMemcpyDeviceToHost);

        cudaFree(d_points);
        cudaFree(d_labels);
    }

    return labels;
}
