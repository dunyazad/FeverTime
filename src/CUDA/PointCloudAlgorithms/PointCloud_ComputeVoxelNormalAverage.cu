#include <CUDA/PointCloud.cuh>

__global__ void Kernel_ComputeVoxelNormalAverage(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	Eigen::Vector3f normal = Eigen::Vector3f::Zero();
	unsigned int neighborCount = 0;
#pragma unroll
	for (int ni = 0; ni < 124; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_124[ni].x,
			coord.y + neighbor_offsets_124[ni].y,
			coord.z + neighbor_offsets_124[ni].z);

		size_t neighborSlot = GetHashMapVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetHashMapVoxel(info, neighborSlot);

		if (neighborVoxel == nullptr) continue;

		auto neighborNormal = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();

		normal += neighborNormal;
		neighborCount++;
	}

	centerVoxel->normal = normal / (float)neighborCount++;
}

void PointCloud::ComputeVoxelNormalAverage()
{
	nvtxRangePushA("Compute Voxel Normal Average");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeVoxelNormalAverage << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();

	nvtxRangePop();
}
