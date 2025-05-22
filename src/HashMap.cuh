#pragma once

#include <cudaCommon.h>
#include <stdHeaderFiles.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace Eigen
{
    using Vector4b = Eigen::Vector<unsigned char, 4>;
    using Vector4ui = Eigen::Vector<unsigned int, 4>;
}

#include <LabelCounter.cuh>

struct PointPNC
{
    float3 position;
    float3 normal;
    float3 color;
};

struct HashMapVoxel
{
    int3 coord = make_int3(0, 0, 0);
    unsigned int label = 0;
    unsigned int subLabel = 0;
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector4b color = Eigen::Vector4b(255, 255, 255, 255);
    float divergence = 0.0f;
    unsigned int pointCount = 0;
    unsigned int neighborCount = 0;
    Eigen::Vector3f gradient = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    float colorDistance = 0.0f;
    uint8_t normalDiscontinue = 0;
};

struct PointCloudBuffers
{
    Eigen::Vector3f* positions = nullptr;
    Eigen::Vector3f* normals = nullptr;
    Eigen::Vector4b* colors = nullptr;
    unsigned int numberOfPoints = 0;
    Eigen::AlignedBox3f aabb =
        Eigen::AlignedBox3f(
            Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
            Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

    bool isHostBuffer = true;

    void Initialize(unsigned int numberOfPoints, bool isHostBuffer);
    void Terminate();

    void CopyTo(PointCloudBuffers& other);
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

__device__ __host__ float hashToFloat(uint32_t seed);
__device__ __host__ size_t voxel_hash(int3 coord, size_t tableSize);

__device__ size_t GetVoxelSlot(HashMapInfo& info, int3 coord);
__device__ HashMapVoxel* GetVoxel(HashMapInfo& info, size_t slot);
