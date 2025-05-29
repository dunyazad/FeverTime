#include <CUDA/PointCloud.cuh>

//__global__ void Kernel_ComputeColorMultiplication(HashMapInfo info)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
//	if (centerVoxel == nullptr || centerVoxel->label == 0) return;
//
//#pragma unroll
//	for (int ni = 0; ni < 26; ++ni)
//	{
//		int3 neighborCoord = make_int3(
//			coord.x + neighbor_offsets_26[ni].x,
//			coord.y + neighbor_offsets_26[ni].y,
//			coord.z + neighbor_offsets_26[ni].z);
//
//		size_t neighborSlot = GetHashMapVoxelSlot(info, neighborCoord);
//		HashMapVoxel* neighborVoxel = GetHashMapVoxel(info, neighborSlot);
//
//		if (neighborVoxel == nullptr || neighborVoxel->label == 0) return;
//
//		float distance = std::sqrt(
//			static_cast<float>(neighborVoxel->color.x()) * centerVoxel->color.x() +
//			static_cast<float>(neighborVoxel->color.y()) * centerVoxel->color.y() +
//			static_cast<float>(neighborVoxel->color.z()) * centerVoxel->color.z());
//
//		//printf("distance : %f\n", distance);
//
//		if (centerVoxel->colorDistance < distance) centerVoxel->colorDistance = distance;
//	}
//}
//
//void PointCloud::ComputeColorMultiplication()
//{
//	nvtxRangePushA("ComputeColorMultiplication");
//
//	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_ComputeColorMultiplication << <gridOccupied, blockSize >> > (hashmap.info);
//
//	cudaDeviceSynchronize();
//}
//
//__global__ void Kernel_SerializeColoringByColorMultiplication(HashMapInfo info, PointCloudBuffers buffers, float threshold)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//	if (idx >= buffers.numberOfPoints) return;
//
//	auto& p = buffers.positions[idx];
//	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
//	if (voxel == nullptr || voxel->label == 0) return;
//
//	buffers.positions[idx] = voxel->position;
//	buffers.normals[idx] = voxel->normal;
//
//	if (voxel->colorDistance > threshold)
//	{
//		buffers.colors[idx] = Eigen::Vector4b(255, 0, 0, 255);
//	}
//	else
//	{
//		buffers.colors[idx] = Eigen::Vector4b(100, 100, 100, 255);
//	}
//}
//
//void PointCloud::SerializeColoringByColorMultiplication(float threshold, PointCloudBuffers& d_tempBuffers)
//{
//	d_buffers.CopyTo(d_tempBuffers);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
//
//	Kernel_SerializeColoringByColorMultiplication << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, threshold);
//
//	cudaDeviceSynchronize();
//}
