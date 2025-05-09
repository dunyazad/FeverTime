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

__device__ __host__ inline size_t voxel_hash(int3 coord, size_t tableSize)
{
    return ((size_t)(coord.x * 73856093) ^ (coord.y * 19349663) ^ (coord.z * 83492791)) % tableSize;
}

__global__ void Kernel_InsertPoints(HashMapInfo info, Device_PointCloud pointCloud)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pointCloud.numberOfPoints) return;

    auto p = pointCloud.points[idx];
    auto n = pointCloud.normals[idx];
    auto c = pointCloud.colors[idx];

    int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));

    size_t h = voxel_hash(coord, info.capacity);
    for (int i = 0; i < info.maxProbe; ++i) {
        size_t slot = (h + i) % info.capacity;
        if (atomicCAS(&(info.d_hashTable[slot].label), 0, slot) == 0)
        {
            //alog("%d, %d, %d\n", coord.x, coord.y, coord.z);

            info.d_hashTable[slot].label = slot;
            info.d_hashTable[slot].coord = coord;
            info.d_hashTable[slot].position = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);
            info.d_hashTable[slot].normal = Eigen::Vector3f(n.x(), n.y(), n.z());
            info.d_hashTable[slot].color = Eigen::Vector3b(c.x(), c.y(), c.z());

            auto oldIndex = atomicAdd(info.d_numberOfOccupiedVoxels, 1);
            info.d_occupiedVoxelIndices[oldIndex] = coord;

            return;
        }
    }
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

void HashMap::Serialize(const std::string& filename)
{
    PLYFormat ply;

    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    alog("numberOfOccupiedVoxels : %d\n", numberOfOccupiedVoxels);

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

void HashMap::Clustering()
{

}
