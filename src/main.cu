#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>
#include <cudaHeaderFiles.h>

#include <Serialization.hpp>

#include "main.cuh"

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

__device__ float3 EigenDecomposition3x3Symmetric(const float A[3][3])
{
    // Based on: https://www.cs.princeton.edu/~smr/papers/eigen.pdf

    float m = (A[0][0] + A[1][1] + A[2][2]) / 3.0f;

    float B[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            B[i][j] = A[i][j] - (i == j ? m : 0.0f);

    float p1 = B[0][1] * B[0][1] + B[0][2] * B[0][2] + B[1][2] * B[1][2];
    if (p1 == 0.0f) {
        return make_float3(A[0][0], A[1][1], A[2][2]); // Diagonal matrix
    }

    float q = (B[0][0] * B[1][1] * B[2][2]
        + 2.0f * B[0][1] * B[0][2] * B[1][2]
        - B[0][0] * B[1][2] * B[1][2]
        - B[1][1] * B[0][2] * B[0][2]
        - B[2][2] * B[0][1] * B[0][1]) / 2.0f;

    float p2 = (B[0][0] * B[0][0] + B[1][1] * B[1][1] + B[2][2] * B[2][2] + 2 * p1) / 6.0f;
    float p = sqrtf(p2);

    float phi = acosf(fminf(fmaxf(q / (p * p * p), -1.0f), 1.0f)) / 3.0f;

    float eig1 = m + 2.0f * p * cosf(phi);
    float eig2 = m + 2.0f * p * cosf(phi + (2.0f * CUDART_PI_F / 3.0f));
    float eig3 = m + 2.0f * p * cosf(phi + (4.0f * CUDART_PI_F / 3.0f));

    // Sort in ascending order
    float3 ev = make_float3(eig1, eig2, eig3);
    if (ev.x > ev.y) { float t = ev.x; ev.x = ev.y; ev.y = t; }
    if (ev.x > ev.z) { float t = ev.x; ev.x = ev.z; ev.z = t; }
    if (ev.y > ev.z) { float t = ev.y; ev.y = ev.z; ev.z = t; }

    return ev;
}

__global__ void Kernel_ComputeCurvatureFromCovariance(HashMapInfo info)
{
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= *info.d_numberOfOccupiedVoxels) return;

    int3 coord = info.d_occupiedVoxelIndices[threadid];
    size_t h = voxel_hash(coord, info.capacity);

    HashMapVoxel* voxel = nullptr;

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

    const int3 offsets[6] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };

    Eigen::Vector3f neighbors[6];
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    int count = 0;

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
                neighbors[count] = neighbor.position;
                mean += neighbor.position;
                ++count;
                break;
            }
        }
    }

    if (count < 3)
    {
        voxel->divergence = 0.0f;
        return;
    }

    mean /= float(count);

    float cov[3][3] = { 0 };

    for (int i = 0; i < count; ++i)
    {
        Eigen::Vector3f d = neighbors[i] - mean;
        cov[0][0] += d.x() * d.x(); cov[0][1] += d.x() * d.y(); cov[0][2] += d.x() * d.z();
        cov[1][0] += d.y() * d.x(); cov[1][1] += d.y() * d.y(); cov[1][2] += d.y() * d.z();
        cov[2][0] += d.z() * d.x(); cov[2][1] += d.z() * d.y(); cov[2][2] += d.z() * d.z();
    }

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            cov[r][c] /= float(count);

    float3 eigenvalues = EigenDecomposition3x3Symmetric(cov);
    float sum = eigenvalues.x + eigenvalues.y + eigenvalues.z + 1e-6f;
    float curvature = eigenvalues.x / sum;

    voxel->divergence = curvature;
}

void HashMap::ComputeCurvatureFromCovariance()
{
    unsigned int numberOfOccupiedVoxels = 0;
    cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

    Kernel_ComputeNormalDivergence << <gridOccupied, blockSize >> > (info);

    cudaDeviceSynchronize();
}