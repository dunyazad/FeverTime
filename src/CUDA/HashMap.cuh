#pragma once

#include <CUDA/cudaCommon.cuh>
#include <CUDA/PointCloudBuffers.cuh>
#include <CUDA/LabelCounter.cuh>

struct HashMapVoxel
{
    int3 coord = make_int3(0, 0, 0);
    uint8_t reservedToDeleted = 0;
    uint8_t deleted = 0;
    unsigned int label = 0;
    unsigned int subLabel = 0;
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector4b color = Eigen::Vector4b(255, 255, 255, 255);
    float divergence = 0.0f;
    unsigned int pointCount = 0;
    unsigned int neighborCount = 0;
    unsigned int emptyNeighborCount = 0;
    Eigen::Vector3f gradient = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    float colorDistance = 0.0f;
    uint8_t normalDiscontinue = 0;
};

struct HashMapInfo
{
    size_t capacity = 1024 * 1024 * 100;
    unsigned int maxProbe = 32;
    float voxelSize = 0.1f;

    HashMapVoxel* d_hashTable = nullptr;
    unsigned int* d_numberOfOccupiedVoxels = nullptr;
    unsigned int h_numberOfOccupiedVoxels = 0;
    int3* d_occupiedVoxelIndices = nullptr;

    LabelCounter labelCounter;
    LabelCounter subLabelCounter;
};

struct HashMap
{
    HashMapInfo info;

    void Initialize();
    void Terminate();

    void InsertPoints(PointCloudBuffers buffers);

    void CountLabels();

    void SerializeToPLY(const string& filename);
};

__device__ size_t GetHashMapVoxelSlot(HashMapInfo& info, int3 coord);
__device__ HashMapVoxel* GetHashMapVoxel(HashMapInfo& info, size_t slot);

__device__ size_t InsertHashMapVoxel(HashMapInfo& info, int3 coord);
__device__ bool DeleteHashMapVoxel(HashMapInfo& info, int3 coord);