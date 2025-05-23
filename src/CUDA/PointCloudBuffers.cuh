#pragma once

#include <CUDA/cudaCommon.cuh>

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
