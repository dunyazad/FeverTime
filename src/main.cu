#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>
#include <cudaHeaderFiles.h>

#include <Serialization.hpp>

#include "main.cuh"

__device__ __host__ inline size_t voxel_hash(int3 coord, size_t tableSize)
{
    return ((size_t)(coord.x * 73856093) ^ (coord.y * 19349663) ^ (coord.z * 83492791)) % tableSize;
}

__global__ void Kernel_InsertPoints(Eigen::Vector3f* points, Eigen::Vector3f* normals, Eigen::Vector3b* colors, int numberOfPoints, float voxelSize, HashMapVoxel* table, size_t tableSize, unsigned int maxProbe)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numberOfPoints) return;

    auto p = points[idx];
    auto n = normals[idx];
    auto c = colors[idx];

    int3 coord = make_int3(floorf(p.x() / voxelSize), floorf(p.y() / voxelSize), floorf(p.z() / voxelSize));

    size_t h = voxel_hash(coord, tableSize);
    for (int i = 0; i < maxProbe; ++i) {
        size_t slot = (h + i) % tableSize;
        if (atomicCAS(&table[slot].label, 0, slot) == 0)
        {
            //alog("%d, %d, %d\n", coord.x, coord.y, coord.z);

            table[slot].label = slot;
            table[slot].position = Eigen::Vector3f((float)coord.x * voxelSize, (float)coord.y * voxelSize, (float)coord.z * voxelSize);
            table[slot].normal = Eigen::Vector3f(n.x(), n.y(), n.z());
            table[slot].color = Eigen::Vector3b(c.x(), c.y(), c.z());
            return;
        }
    }
}

__global__ void Kernel_Serialize(HashMapVoxel* d_table, size_t tableSize,
    Eigen::Vector3f* d_points, Eigen::Vector3f* d_normals, Eigen::Vector3b* d_colors,
    unsigned int* numberOfOccupiedVoxels)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= tableSize) return;

    auto& voxel = d_table[idx];

    if (0 != voxel.label)
    {
        //alog("%f, %f, %f\n", voxel.position.x(), voxel.position.y(), voxel.position.z());

        auto oldIndex = atomicAdd(numberOfOccupiedVoxels, 1);
        d_points[oldIndex] = voxel.position;
        d_normals[oldIndex] = voxel.normal;
        d_colors[oldIndex] = voxel.color;
    }
}

void HashMap::Initialize()
{
    cudaMalloc(&d_table, sizeof(HashMapVoxel) * tableSize);
    cudaMemset(d_table, 0, sizeof(HashMapVoxel) * tableSize);
}

void HashMap::Terminate()
{
    cudaFree(d_table);
}

void HashMap::InsertHPoints(Eigen::Vector3f* h_points, Eigen::Vector3f* h_normals, Eigen::Vector3b* h_colors, unsigned numberOfPoints)
{
    Eigen::Vector3f* d_points = nullptr;
    Eigen::Vector3f* d_normals = nullptr;
    Eigen::Vector3b* d_colors = nullptr;

    cudaMalloc(&d_points, sizeof(Eigen::Vector3f) * numberOfPoints);
    cudaMemcpy(d_points, h_points, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);

    cudaMalloc(&d_normals, sizeof(Eigen::Vector3f) * numberOfPoints);
    cudaMemcpy(d_normals, h_normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);

    cudaMalloc(&d_colors, sizeof(Eigen::Vector3b) * numberOfPoints);
    cudaMemcpy(d_colors, d_colors, sizeof(Eigen::Vector3b) * numberOfPoints, cudaMemcpyHostToDevice);

    InsertDPoints(d_points, d_normals, d_colors, numberOfPoints);

    cudaFree(d_points);
    cudaFree(d_normals);
    cudaFree(d_colors);
}

void HashMap::InsertDPoints(Eigen::Vector3f* d_points, Eigen::Vector3f* d_normals, Eigen::Vector3b* d_colors, unsigned numberOfPoints)
{
    unsigned int blockSize = 256;
    unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

    Kernel_InsertPoints << <gridOccupied, blockSize >> > (
        d_points,
        d_normals,
        d_colors,
        numberOfPoints,
        0.1f, d_table, tableSize, maxProbe);

    cudaDeviceSynchronize();
}

void HashMap::Serialize(const std::string& filename)
{
    PLYFormat ply;

    unsigned int* d_numberOfOccupiedVoxels = nullptr;
    cudaMalloc(&d_numberOfOccupiedVoxels, sizeof(unsigned int));

    Eigen::Vector3f* d_points = nullptr;
    Eigen::Vector3f* d_normals = nullptr;
    Eigen::Vector3b* d_colors = nullptr;

    cudaMalloc(&d_points, sizeof(Eigen::Vector3f) * tableSize);
    cudaMalloc(&d_normals, sizeof(Eigen::Vector3f) * tableSize);
    cudaMalloc(&d_colors, sizeof(Eigen::Vector3b) * tableSize);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (tableSize + blockSize - 1) / blockSize;

    Kernel_Serialize << <gridOccupied, blockSize >> > (
        d_table,
        tableSize,
        d_points,
        d_normals,
        d_colors,
        d_numberOfOccupiedVoxels);

    cudaDeviceSynchronize();

    unsigned int h_numberOfOccupiedVoxels = 0;
    cudaMemcpy(&h_numberOfOccupiedVoxels, d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

    Eigen::Vector3f* h_points = new Eigen::Vector3f[h_numberOfOccupiedVoxels];
    Eigen::Vector3f* h_normals = new Eigen::Vector3f[h_numberOfOccupiedVoxels];
    Eigen::Vector3b* h_colors = new Eigen::Vector3b[h_numberOfOccupiedVoxels];

    cudaMemcpy(h_points, d_points, sizeof(Eigen::Vector3f) * h_numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_normals, d_normals, sizeof(Eigen::Vector3f) * h_numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_colors, d_colors, sizeof(Eigen::Vector3b) * h_numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < h_numberOfOccupiedVoxels; i++)
    {
        auto& p = h_points[i];
        auto& n = h_normals[i];
        auto& c = h_colors[i];

        ply.AddPoint(p.x(), p.y(), p.z());
        ply.AddNormal(n.x(), n.y(), n.z());
        ply.AddColor(c.x() / 255.0f, c.y() / 255.0f, c.z() / 255.0f);
    }

    ply.Serialize(filename);

    cudaFree(d_numberOfOccupiedVoxels);
    cudaFree(d_points);
    cudaFree(d_normals);
    cudaFree(d_colors);

    delete[] h_points;
    delete[] h_normals;
    delete[] h_colors;
}
