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

Eigen::Vector3f Transform(const Eigen::Matrix4f& tm, const Eigen::Vector3f& p);

Eigen::Matrix4f vtkToEigen(const vtkMatrix4x4* vtkMat);
vtkSmartPointer<vtkMatrix4x4> eigenToVtk(const Eigen::Matrix4f& eigenMat);
