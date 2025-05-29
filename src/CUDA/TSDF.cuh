#pragma once

#include <CUDA/cudaCommon.cuh>
#include <CUDA/PointCloudBuffers.cuh>
#include <CUDA/LabelCounter.cuh>
//
//#define INVALID_VOXEL_SLOT UINT64_MAX
//#define INVALID_VOXEL nullptr
//
//struct TSDFVoxel
//{
//    int3 coord = make_int3(0, 0, 0);
//    unsigned int label = 0;
//    unsigned int subLabel = 0;
//    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
//    Eigen::Vector4b color = Eigen::Vector4b(255, 255, 255, 255);
//    float value = 0.0f;
//    float weight = 0.0f;
//    unsigned int pointCount = 0;
//};
//
//struct TSDFInfo
//{
//    size_t capacity = 1024 * 1024 * 100;
//    unsigned int maxProbe = 32;
//    float voxelSize = 0.1f;
//    float truncation = 0.4f;
//
//    TSDFVoxel* d_hashTable = nullptr;
//    unsigned int* d_numberOfOccupiedVoxels = nullptr;
//    unsigned int h_numberOfOccupiedVoxels = 0;
//    int3* d_occupiedVoxelIndices = nullptr;
//
//    LabelCounter labelCounter;
//    LabelCounter subLabelCounter;
//};
//
//struct TSDF
//{
//    TSDFInfo info;
//
//    void Initialize(size_t capacity);
//    void Terminate();
//
//    //void InsertPoints(PointCloudBuffers buffers);
//
//    void CountLabels();
//
//    void SerializeToPLY(const string& filename);
//};
//
//__device__ size_t GetTSDFVoxelSlot(TSDFInfo& info, int3 coord);
//__device__ TSDFVoxel* GetTSDFVoxel(TSDFInfo& info, size_t slot);
//__device__ void InsertTSDFVoxel(TSDFInfo& info, const int3& coord, const Eigen::Vector3f& normal, const Eigen::Vector4b& color);