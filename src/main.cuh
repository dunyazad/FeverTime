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

struct PointCloud
{
    Eigen::Vector3f* d_points = nullptr;
    Eigen::Vector3f* d_normals = nullptr;
    Eigen::Vector3b* d_colors = nullptr;
    unsigned int numberOfPoints = 0;
};

struct PointPNC
{
    float3 position;
    float3 normal;
    float3 color;
};

struct HashMap
{
    size_t tableSize = 10485760;
    unsigned int maxProbe = 32;
    unsigned int blockSize = 256;

    HashMapVoxel* d_table = nullptr;

    void Initialize();
    void Terminate();

    void InsertHPoints(Eigen::Vector3f* h_points, Eigen::Vector3f* h_normals, Eigen::Vector3b* h_colors, unsigned numberOfPoints);
    void InsertDPoints(Eigen::Vector3f* d_points, Eigen::Vector3f* d_normals, Eigen::Vector3b* d_colors, unsigned numberOfPoints);

    void Serialize(const std::string& filename);
};
