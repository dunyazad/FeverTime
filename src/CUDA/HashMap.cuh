#pragma once

#include <CUDA/cudaCommon.cuh>
#include <CUDA/PointCloudBuffers.cuh>
#include <CUDA/LabelCounter.cuh>

#define INVALID_VOXEL_SLOT UINT64_MAX
#define INVALID_VOXEL nullptr

struct HashMapVoxel
{
    int3 coord = make_int3(0, 0, 0);
    uint8_t reservedToDeleted = 0;
    uint8_t deleted = 0;
    unsigned int label = 0;
    unsigned int subLabel = 0;
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f positionToMove = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector4b color = Eigen::Vector4b(255, 255, 255, 255);
    float divergence = 0.0f;
    unsigned int pointCount = 0;
    unsigned int neighborCount = 0;
    unsigned int emptyNeighborCount = 0;
    float sdf = 0.0f;
    float sdfSum = 0.0f;
    float weightSum = 0.0f;
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
    void Clear(size_t capacity);

    void InsertPoints(float3* positions, float3* normals, uchar4* colors, size_t numberOfPoints);

    void CountLabels();

    void SerializeToPLY(const string& filename);
};

__device__ size_t GetHashMapVoxelSlot(HashMapInfo& info, int3 coord);
__device__ HashMapVoxel* GetHashMapVoxel(HashMapInfo& info, size_t slot);

__device__ size_t InsertHashMapVoxel(HashMapInfo& info, int3 coord);
__device__ bool DeleteHashMapVoxel(HashMapInfo& info, int3 coord);
