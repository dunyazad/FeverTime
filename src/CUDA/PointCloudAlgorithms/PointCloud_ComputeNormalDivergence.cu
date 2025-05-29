#include <CUDA/PointCloud.cuh>

//__global__ void Kernel_ComputeNormalDivergence(HashMapInfo info)
//{
//	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//	auto voxelSlot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == voxelSlot) return;
//	auto voxel = GetHashMapVoxel(info, voxelSlot);
//	if (voxel == nullptr || voxel->label == 0) return;
//
//	const Eigen::Vector3f n0 = voxel->normal.normalized();
//
//	const int3 offsets[6] = {
//		{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
//	};
//
//	float divergenceSum = 0.0f;
//	int validNeighbors = 0;
//
//	for (int ni = 0; ni < 6; ++ni)
//	{
//		int3 neighborCoord = make_int3(
//			coord.x + offsets[ni].x,
//			coord.y + offsets[ni].y,
//			coord.z + offsets[ni].z);
//
//		auto neighborVoxelSlot = GetHashMapVoxelSlot(info, neighborCoord);
//		if (INVALID_VOXEL_SLOT == neighborVoxelSlot) continue;
//
//		auto neighborVoxel = GetHashMapVoxel(info, neighborVoxelSlot);
//		if (neighborVoxel == nullptr || neighborVoxel->label == 0) continue;
//
//		Eigen::Vector3f n1 = neighborVoxel->normal.normalized();
//		float dot = fminf(fmaxf(n0.dot(n1), -1.0f), 1.0f);
//		float angle = acosf(dot); // radians
//		divergenceSum += angle;
//		++validNeighbors;
//	}
//
//	voxel->divergence = (validNeighbors > 0) ? (divergenceSum / validNeighbors) : 0.0f;
//
//	//printf("voxel->divergence : %f\n", voxel->divergence);
//}
//
//void PointCloud::ComputeNormalDivergence()
//{
//	nvtxRangePushA("ComputeNormalDivergence");
//
//	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_ComputeNormalDivergence << <gridOccupied, blockSize >> > (hashmap.info);
//
//	cudaDeviceSynchronize();
//}
//
//__global__ void Kernel_SerializeColoringByNormalDivergence(HashMapInfo info, PointCloudBuffers buffers, float threshold)
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
//	//printf("voxel->divergence : %f\n", voxel->divergence);
//
//	if (voxel->divergence > threshold)
//	{
//		buffers.colors[idx] = Eigen::Vector4b(255, 0, 0, 255);
//	}
//	else
//	{
//		buffers.colors[idx] = Eigen::Vector4b(100, 100, 100, 255);
//	}
//}
//
//void PointCloud::SerializeColoringByNormalDivergence(float threshold, PointCloudBuffers& d_tempBuffers)
//{
//	d_buffers.CopyTo(d_tempBuffers);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
//
//	Kernel_SerializeColoringByNormalDivergence << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, threshold);
//
//	cudaDeviceSynchronize();
//}
