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

__global__ void Kernel_InsertPoints(Device_PointCloud pointCloud,
    float voxelSize, HashMapVoxel* table, size_t tableSize, unsigned int maxProbe)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pointCloud.numberOfPoints) return;

    auto p = pointCloud.points[idx];
    auto n = pointCloud.normals[idx];
    auto c = pointCloud.colors[idx];

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

__global__ void Kernel_Serialize(HashMapVoxel* d_hashTable, size_t capacity,
    Eigen::Vector3f* d_points, Eigen::Vector3f* d_normals, Eigen::Vector3b* d_colors,
    unsigned int* numberOfOccupiedVoxels)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= capacity) return;

    auto& voxel = d_hashTable[idx];

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
    cudaMalloc(&d_hashTable, sizeof(HashMapVoxel) * capacity);
    cudaMemset(d_hashTable, 0, sizeof(HashMapVoxel) * capacity);

    cudaMalloc(&d_numberOfOccupiedVoxels, sizeof(unsigned int) * capacity);
    cudaMemset(d_numberOfOccupiedVoxels, 0, sizeof(unsigned int) * capacity);

    cudaMalloc(&d_occupiedVoxelIndices, sizeof(int3) * capacity);
    cudaMemset(d_occupiedVoxelIndices, 0, sizeof(int3) * capacity);
}

void HashMap::Terminate()
{
    cudaFree(d_hashTable);
    cudaFree(d_numberOfOccupiedVoxels);
    cudaFree(d_occupiedVoxelIndices);
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

    Kernel_InsertPoints << <gridOccupied, blockSize >> > (
        pointCloud,
        0.1f, d_hashTable, capacity, maxProbe);

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

    cudaMalloc(&d_points, sizeof(Eigen::Vector3f) * capacity);
    cudaMalloc(&d_normals, sizeof(Eigen::Vector3f) * capacity);
    cudaMalloc(&d_colors, sizeof(Eigen::Vector3b) * capacity);

    unsigned int blockSize = 256;
    unsigned int gridOccupied = (capacity + blockSize - 1) / blockSize;

    Kernel_Serialize << <gridOccupied, blockSize >> > (
        d_hashTable,
        capacity,
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

void HashMap::Clustering()
{

}
