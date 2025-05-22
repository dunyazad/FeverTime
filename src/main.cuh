#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace Eigen {
    using Vector4b = Eigen::Vector<unsigned char, 4>;
    using Vector3ui = Eigen::Vector<unsigned int, 4>;
}

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)

#define CUDART_PI_F 3.1415927f

#include <PointCloud.cuh>
