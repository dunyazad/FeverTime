#pragma once

#define NOMINMAX

#include <cudaHeaderFiles.h>
#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace Eigen
{
    using Vector4b = Eigen::Vector<unsigned char, 4>;
    using Vector4ui = Eigen::Vector<unsigned int, 4>;
}

namespace Time
{
    chrono::steady_clock::time_point Now();

    uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now);

    chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message = "", int number = -1);

    string DateTime();
}

Eigen::Vector3f Transform(const Eigen::Matrix4f& tm, const Eigen::Vector3f& p);

Eigen::Matrix4f vtkToEigen(const vtkMatrix4x4* vtkMat);
vtkSmartPointer<vtkMatrix4x4> eigenToVtk(const Eigen::Matrix4f& eigenMat);
