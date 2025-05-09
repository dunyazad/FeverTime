#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
namespace Eigen {
    using Vector3b = Eigen::Vector<unsigned char, 3>;
    using Vector3ui = Eigen::Vector<unsigned int, 3>;
}

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)

struct HashMapVoxel
{
    unsigned int label = 0;
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3b color = Eigen::Vector3b(255, 255, 255);
};

struct Host_PointCloud
{
    Eigen::Vector3f* points = nullptr;
    Eigen::Vector3f* normals = nullptr;
    Eigen::Vector3b* colors = nullptr;
    unsigned int numberOfPoints = 0;

    void Initialize(unsigned int numberOfPoints);
    void Terminate();
};

struct Device_PointCloud
{
    Eigen::Vector3f* points = nullptr;
    Eigen::Vector3f* normals = nullptr;
    Eigen::Vector3b* colors = nullptr;
    unsigned int numberOfPoints = 0;

    void Initialize(unsigned int numberOfPoints);
    void Terminate();
};

struct PointPNC
{
    float3 position;
    float3 normal;
    float3 color;
};

struct HashMap
{
    size_t capacity = 1024 * 1024 * 100;
    unsigned int maxProbe = 32;
    unsigned int blockSize = 256;

    HashMapVoxel* d_hashTable = nullptr;
    unsigned int* d_numberOfOccupiedVoxels = nullptr;
    int3* d_occupiedVoxelIndices = nullptr;

    void Initialize();
    void Terminate();

    void InsertHPoints(Host_PointCloud pointCloud);
    void InsertDPoints(Device_PointCloud pointCloud);

    void Serialize(const std::string& filename);

    void Clustering();
};
