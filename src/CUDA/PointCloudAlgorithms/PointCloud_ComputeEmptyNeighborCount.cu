#include <CUDA/PointCloud.cuh>

__global__ void Kernel_ComputeEmptyNeighborCount(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	unsigned int emptyNeighborCount = 0;

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetHashMapVoxelSlot(info, neighborCoord);
		if (INVALID_VOXEL_SLOT == neighborSlot)
		{
			emptyNeighborCount++;
			continue;
		}

		HashMapVoxel* neighborVoxel = GetHashMapVoxel(info, neighborSlot);

		if (neighborVoxel == nullptr || neighborVoxel->label == 0)
		{
			emptyNeighborCount++;
		}
	}

	if (centerVoxel->emptyNeighborCount < emptyNeighborCount)
	{
		centerVoxel->emptyNeighborCount = emptyNeighborCount;
	}
}

void PointCloud::ComputeEmptyNeighborCount()
{
	nvtxRangePushA("EmptyNeighborCount");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeEmptyNeighborCount << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();
}

__global__ void Kernel_SerializeColoringByEmptyNeighborCount(HashMapInfo info, PointCloudBuffers buffers, unsigned int countThreshold)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetHashMapVoxelSlot(info, coord);
	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
	if (voxel == nullptr || voxel->label == 0) return;

	buffers.positions[idx] = voxel->position;
	buffers.normals[idx] = voxel->normal;

	//const Eigen::Vector4b COLORS[26] = {
	//{255, 0, 0},
	//{233, 0, 21},
	//{212, 0, 42},
	//{191, 0, 63},
	//{170, 0, 85},
	//{148, 0, 106},
	//{127, 0, 127},
	//{106, 0, 148},
	//{85, 0, 170},
	//{63, 0, 191},
	//{42, 0, 212},
	//{21, 0, 233},
	//{0, 0, 255},
	//{0, 21, 233},
	//{0, 42, 212},
	//{0, 63, 191},
	//{0, 85, 170},
	//{0, 106, 148},
	//{0, 127, 127},
	//{0, 148, 106},
	//{0, 170, 85},
	//{0, 191, 63},
	//{0, 212, 42},
	//{0, 233, 21},
	//{0, 255, 0},
	//{0, 255, 0}  // 마지막 중복은 종료 강조용 (선택사항)
	//};

	//buffers.colors[idx] = COLORS[25 - voxel->neighborCount];

	if (countThreshold < voxel->emptyNeighborCount)
	{
		buffers.colors[idx] = Eigen::Vector4b(255, 0, 0, 255);
	}
	else
	{
		buffers.colors[idx] = Eigen::Vector4b(100, 100, 100, 255);
	}
}

void PointCloud::SerializeColoringByEmptyNeighborCount(unsigned int countThreshold, PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByEmptyNeighborCount << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, countThreshold);

	cudaDeviceSynchronize();
}
