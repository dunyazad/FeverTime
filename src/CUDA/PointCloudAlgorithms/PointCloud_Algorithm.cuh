#pragma once

#include <CUDA/HashMap.cuh>

class DevicePointCloud;
class HostPointCloud;

class PointCloudAlgorithm
{
public:
	PointCloudAlgorithm() = default;
	virtual ~PointCloudAlgorithm() = default;

	virtual void RunAlgorithm(DevicePointCloud* pointCloud) = 0;
	virtual void RunAlgorithm(HostPointCloud* pointCloud) = 0;

	virtual void IncreaseParameter() = 0;
	virtual void DecreaseParameter() = 0;
private:
};





__device__ __forceinline__ unsigned int FindRootVoxel(HashMapInfo info, unsigned int idx);
__device__ __forceinline__ unsigned int FindRootVoxelSub(HashMapInfo info, unsigned int idx);
__device__ __forceinline__ void UnionVoxel(HashMapInfo info, unsigned int a, unsigned int b);
__device__ __forceinline__ void UnionVoxelSub(HashMapInfo info, unsigned int a, unsigned int b);

__global__ void Kernel_ClearLabels(HashMapInfo info);
__global__ void Kernel_InterVoxelHashMerge6Way(HashMapInfo info, float normalDegreeThreshold);
__global__ void Kernel_InterVoxelHashMerge26Way(HashMapInfo info, float normalDegreeThreshold);
__global__ void Kernel_CompressVoxelHashLabels(HashMapInfo info);
__global__ void Kernel_GetLabels(HashMapInfo info, Eigen::Vector3f* points, uint3* labels, unsigned int numberOfPoints);

__global__ void Kernel_SerializeColoringByLabel(HashMapInfo info, float3* positions, float3* normals, uchar4* colors, size_t numberOfPoints);