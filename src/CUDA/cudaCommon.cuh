#pragma once

#include <cudaHeaderFiles.h>

#include <thrustHeaderFiles.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace Eigen
{
	using Vector4b = Eigen::Vector<unsigned char, 4>;
	using Vector4ui = Eigen::Vector<unsigned int, 4>;
}

#include <stdHeaderFiles.h>

#include <Serialization.hpp>

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)

namespace Time
{
	chrono::steady_clock::time_point Now();

	uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now);

	chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message = "", int number = -1);

	string DateTime();
}

string Miliseconds(const chrono::steady_clock::time_point beginTime, const char* tag = nullptr);
#define TS(name) auto time_##name = chrono::high_resolution_clock::now();
#define TE(name) std::cout << Miliseconds(time_##name, #name) << std::endl;

#define CUDART_PI_F 3.1415927f

__device__ __constant__ const int3 neighbor_offsets_6[6] = {
	{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
};

__device__ __constant__ const int3 neighbor_offsets_26[26] = {
	{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
	{1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
	{1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
	{0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
	{1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
	{-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
};

__device__ __constant__ const int3 neighbor_offsets_124[124] = {
	{-2,-2,-2}, {-2,-2,-1}, {-2,-2, 0}, {-2,-2, 1}, {-2,-2, 2},
	{-2,-1,-2}, {-2,-1,-1}, {-2,-1, 0}, {-2,-1, 1}, {-2,-1, 2},
	{-2, 0,-2}, {-2, 0,-1}, {-2, 0, 1}, {-2, 0, 2},
	{-2, 1,-2}, {-2, 1,-1}, {-2, 1, 0}, {-2, 1, 1}, {-2, 1, 2},
	{-2, 2,-2}, {-2, 2,-1}, {-2, 2, 0}, {-2, 2, 1}, {-2, 2, 2},

	{-1,-2,-2}, {-1,-2,-1}, {-1,-2, 0}, {-1,-2, 1}, {-1,-2, 2},
	{-1,-1,-2}, {-1,-1,-1}, {-1,-1, 0}, {-1,-1, 1}, {-1,-1, 2},
	{-1, 0,-2}, {-1, 0,-1}, {-1, 0, 1}, {-1, 0, 2},
	{-1, 1,-2}, {-1, 1,-1}, {-1, 1, 0}, {-1, 1, 1}, {-1, 1, 2},
	{-1, 2,-2}, {-1, 2,-1}, {-1, 2, 0}, {-1, 2, 1}, {-1, 2, 2},

	{ 0,-2,-2}, { 0,-2,-1}, { 0,-2, 0}, { 0,-2, 1}, { 0,-2, 2},
	{ 0,-1,-2}, { 0,-1,-1}, { 0,-1, 0}, { 0,-1, 1}, { 0,-1, 2},
	{ 0, 0,-2}, { 0, 0,-1},            { 0, 0, 1}, { 0, 0, 2},
	{ 0, 1,-2}, { 0, 1,-1}, { 0, 1, 0}, { 0, 1, 1}, { 0, 1, 2},
	{ 0, 2,-2}, { 0, 2,-1}, { 0, 2, 0}, { 0, 2, 1}, { 0, 2, 2},

	{ 1,-2,-2}, { 1,-2,-1}, { 1,-2, 0}, { 1,-2, 1}, { 1,-2, 2},
	{ 1,-1,-2}, { 1,-1,-1}, { 1,-1, 0}, { 1,-1, 1}, { 1,-1, 2},
	{ 1, 0,-2}, { 1, 0,-1}, { 1, 0, 1}, { 1, 0, 2},
	{ 1, 1,-2}, { 1, 1,-1}, { 1, 1, 0}, { 1, 1, 1}, { 1, 1, 2},
	{ 1, 2,-2}, { 1, 2,-1}, { 1, 2, 0}, { 1, 2, 1}, { 1, 2, 2},

	{ 2,-2,-2}, { 2,-2,-1}, { 2,-2, 0}, { 2,-2, 1}, { 2,-2, 2},
	{ 2,-1,-2}, { 2,-1,-1}, { 2,-1, 0}, { 2,-1, 1}, { 2,-1, 2},
	{ 2, 0,-2}, { 2, 0,-1}, { 2, 0, 1}, { 2, 0, 2},
	{ 2, 1,-2}, { 2, 1,-1}, { 2, 1, 0}, { 2, 1, 1}, { 2, 1, 2},
	{ 2, 2,-2}, { 2, 2,-1}, { 2, 2, 0}, { 2, 2, 1}, { 2, 2, 2}
};

//__device__ __constant__ const int3 neighbor_offsets_342[342] = {
//    {-3,-3,-3}, {-3,-3,-2}, {-3,-3,-1}, {-3,-3,  0}, {-3,-3,  1}, {-3,-3,  2}, {-3,-3,  3},
//    {-3,-2,-3}, {-3,-2,-2}, {-3,-2,-1}, {-3,-2,  0}, {-3,-2,  1}, {-3,-2,  2}, {-3,-2,  3},
//    {-3,-1,-3}, {-3,-1,-2}, {-3,-1,-1}, {-3,-1,  0}, {-3,-1,  1}, {-3,-1,  2}, {-3,-1,  3},
//    {-3, 0,-3}, {-3, 0,-2}, {-3, 0,-1}, {-3, 0,  0}, {-3, 0,  1}, {-3, 0,  2}, {-3, 0,  3},
//    {-3, 1,-3}, {-3, 1,-2}, {-3, 1,-1}, {-3, 1,  0}, {-3, 1,  1}, {-3, 1,  2}, {-3, 1,  3},
//    {-3, 2,-3}, {-3, 2,-2}, {-3, 2,-1}, {-3, 2,  0}, {-3, 2,  1}, {-3, 2,  2}, {-3, 2,  3},
//    {-3, 3,-3}, {-3, 3,-2}, {-3, 3,-1}, {-3, 3,  0}, {-3, 3,  1}, {-3, 3,  2}, {-3, 3,  3},
//
//    {-2,-3,-3}, {-2,-3,-2}, {-2,-3,-1}, {-2,-3,  0}, {-2,-3,  1}, {-2,-3,  2}, {-2,-3,  3},
//    {-2,-2,-3}, {-2,-2,-2}, {-2,-2,-1}, {-2,-2,  0}, {-2,-2,  1}, {-2,-2,  2}, {-2,-2,  3},
//    {-2,-1,-3}, {-2,-1,-2}, {-2,-1,-1}, {-2,-1,  0}, {-2,-1,  1}, {-2,-1,  2}, {-2,-1,  3},
//    {-2, 0,-3}, {-2, 0,-2}, {-2, 0,-1}, {-2, 0,  0}, {-2, 0,  1}, {-2, 0,  2}, {-2, 0,  3},
//    {-2, 1,-3}, {-2, 1,-2}, {-2, 1,-1}, {-2, 1,  0}, {-2, 1,  1}, {-2, 1,  2}, {-2, 1,  3},
//    {-2, 2,-3}, {-2, 2,-2}, {-2, 2,-1}, {-2, 2,  0}, {-2, 2,  1}, {-2, 2,  2}, {-2, 2,  3},
//    {-2, 3,-3}, {-2, 3,-2}, {-2, 3,-1}, {-2, 3,  0}, {-2, 3,  1}, {-2, 3,  2}, {-2, 3,  3},
//
//    {-1,-3,-3}, {-1,-3,-2}, {-1,-3,-1}, {-1,-3,  0}, {-1,-3,  1}, {-1,-3,  2}, {-1,-3,  3},
//    {-1,-2,-3}, {-1,-2,-2}, {-1,-2,-1}, {-1,-2,  0}, {-1,-2,  1}, {-1,-2,  2}, {-1,-2,  3},
//    {-1,-1,-3}, {-1,-1,-2}, {-1,-1,-1}, {-1,-1,  0}, {-1,-1,  1}, {-1,-1,  2}, {-1,-1,  3},
//    {-1, 0,-3}, {-1, 0,-2}, {-1, 0,-1}, {-1, 0,  0}, {-1, 0,  1}, {-1, 0,  2}, {-1, 0,  3},
//    {-1, 1,-3}, {-1, 1,-2}, {-1, 1,-1}, {-1, 1,  0}, {-1, 1,  1}, {-1, 1,  2}, {-1, 1,  3},
//    {-1, 2,-3}, {-1, 2,-2}, {-1, 2,-1}, {-1, 2,  0}, {-1, 2,  1}, {-1, 2,  2}, {-1, 2,  3},
//    {-1, 3,-3}, {-1, 3,-2}, {-1, 3,-1}, {-1, 3,  0}, {-1, 3,  1}, {-1, 3,  2}, {-1, 3,  3},
//
//    { 0,-3,-3}, { 0,-3,-2}, { 0,-3,-1}, { 0,-3,  0}, { 0,-3,  1}, { 0,-3,  2}, { 0,-3,  3},
//    { 0,-2,-3}, { 0,-2,-2}, { 0,-2,-1}, { 0,-2,  0}, { 0,-2,  1}, { 0,-2,  2}, { 0,-2,  3},
//    { 0,-1,-3}, { 0,-1,-2}, { 0,-1,-1}, { 0,-1,  0}, { 0,-1,  1}, { 0,-1,  2}, { 0,-1,  3},
//    { 0, 0,-3}, { 0, 0,-2}, { 0, 0,-1},            { 0, 0,  1}, { 0, 0,  2}, { 0, 0,  3},
//    { 0, 1,-3}, { 0, 1,-2}, { 0, 1,-1}, { 0, 1,  0}, { 0, 1,  1}, { 0, 1,  2}, { 0, 1,  3},
//    { 0, 2,-3}, { 0, 2,-2}, { 0, 2,-1}, { 0, 2,  0}, { 0, 2,  1}, { 0, 2,  2}, { 0, 2,  3},
//    { 0, 3,-3}, { 0, 3,-2}, { 0, 3,-1}, { 0, 3,  0}, { 0, 3,  1}, { 0, 3,  2}, { 0, 3,  3},
//
//    { 1,-3,-3}, { 1,-3,-2}, { 1,-3,-1}, { 1,-3,  0}, { 1,-3,  1}, { 1,-3,  2}, { 1,-3,  3},
//    { 1,-2,-3}, { 1,-2,-2}, { 1,-2,-1}, { 1,-2,  0}, { 1,-2,  1}, { 1,-2,  2}, { 1,-2,  3},
//    { 1,-1,-3}, { 1,-1,-2}, { 1,-1,-1}, { 1,-1,  0}, { 1,-1,  1}, { 1,-1,  2}, { 1,-1,  3},
//    { 1, 0,-3}, { 1, 0,-2}, { 1, 0,-1}, { 1, 0,  0}, { 1, 0,  1}, { 1, 0,  2}, { 1, 0,  3},
//    { 1, 1,-3}, { 1, 1,-2}, { 1, 1,-1}, { 1, 1,  0}, { 1, 1,  1}, { 1, 1,  2}, { 1, 1,  3},
//    { 1, 2,-3}, { 1, 2,-2}, { 1, 2,-1}, { 1, 2,  0}, { 1, 2,  1}, { 1, 2,  2}, { 1, 2,  3},
//    { 1, 3,-3}, { 1, 3,-2}, { 1, 3,-1}, { 1, 3,  0}, { 1, 3,  1}, { 1, 3,  2}, { 1, 3,  3},
//
//    { 2,-3,-3}, { 2,-3,-2}, { 2,-3,-1}, { 2,-3,  0}, { 2,-3,  1}, { 2,-3,  2}, { 2,-3,  3},
//    { 2,-2,-3}, { 2,-2,-2}, { 2,-2,-1}, { 2,-2,  0}, { 2,-2,  1}, { 2,-2,  2}, { 2,-2,  3},
//    { 2,-1,-3}, { 2,-1,-2}, { 2,-1,-1}, { 2,-1,  0}, { 2,-1,  1}, { 2,-1,  2}, { 2,-1,  3},
//    { 2, 0,-3}, { 2, 0,-2}, { 2, 0,-1}, { 2, 0,  0}, { 2, 0,  1}, { 2, 0,  2}, { 2, 0,  3},
//    { 2, 1,-3}, { 2, 1,-2}, { 2, 1,-1}, { 2, 1,  0}, { 2, 1,  1}, { 2, 1,  2}, { 2, 1,  3},
//    { 2, 2,-3}, { 2, 2,-2}, { 2, 2,-1}, { 2, 2,  0}, { 2, 2,  1}, { 2, 2,  2}, { 2, 2,  3},
//    { 2, 3,-3}, { 2, 3,-2}, { 2, 3,-1}, { 2, 3,  0}, { 2, 3,  1}, { 2, 3,  2}, { 2, 3,  3},
//
//    { 3,-3,-3}, { 3,-3,-2}, { 3,-3,-1}, { 3,-3,  0}, { 3,-3,  1}, { 3,-3,  2}, { 3,-3,  3},
//    { 3,-2,-3}, { 3,-2,-2}, { 3,-2,-1}, { 3,-2,  0}, { 3,-2,  1}, { 3,-2,  2}, { 3,-2,  3},
//    { 3,-1,-3}, { 3,-1,-2}, { 3,-1,-1}, { 3,-1,  0}, { 3,-1,  1}, { 3,-1,  2}, { 3,-1,  3},
//    { 3, 0,-3}, { 3, 0,-2}, { 3, 0,-1}, { 3, 0,  0}, { 3, 0,  1}, { 3, 0,  2}, { 3, 0,  3},
//    { 3, 1,-3}, { 3, 1,-2}, { 3, 1,-1}, { 3, 1,  0}, { 3, 1,  1}, { 3, 1,  2}, { 3, 1,  3},
//    { 3, 2,-3}, { 3, 2,-2}, { 3, 2,-1}, { 3, 2,  0}, { 3, 2,  1}, { 3, 2,  2}, { 3, 2,  3},
//    { 3, 3,-3}, { 3, 3,-2}, { 3, 3,-1}, { 3, 3,  0}, { 3, 3,  1}, { 3, 3,  2}, { 3, 3,  3}
//};


struct PointPNC
{
	float3 position;
	float3 normal;
	float3 color;
};

__host__ __device__ float3 operator+(const float3& a, const float3& b);
__host__ __device__ float3& operator+=(float3& a, const float3& b);

__host__ __device__ float3 operator-(const float3& a, const float3& b);
__host__ __device__ float3& operator-=(float3& a, const float3& b);

__host__ __device__ float3 operator*(const float3& a, float b);
__host__ __device__ float3& operator*=(float3& a, float b);
__host__ __device__ float3 operator*(float b, const float3& a);
__host__ __device__ float3& operator*=(float b, float3& a);

__host__ __device__ float3 operator/(const float3& a, float b);
__host__ __device__ float3& operator/=(float3& a, float b);
__host__ __device__ float3 operator/(float b, const float3& a);
__host__ __device__ float3& operator/=(float b, float3& a);

__host__ __device__ float dot(const float3& a, const float3& b);
__host__ __device__ float3 cross(const float3& a, const float3& b);
__host__ __device__ float length(const float3& v);
__host__ __device__ float lengthSquared(const float3& v);
__host__ __device__ float3 normalize(const float3& v);

__device__ __host__ float hashToFloat(uint32_t seed);
__device__ __host__ size_t voxel_hash(int3 coord, size_t tableSize);

__device__ float atomicMinFloat(float* address, float value);