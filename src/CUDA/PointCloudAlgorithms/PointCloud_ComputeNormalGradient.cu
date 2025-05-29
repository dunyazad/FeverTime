#include <CUDA/PointCloud.cuh>

//__global__ void Kernel_ComputeNormalGradient(HashMapInfo info)
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
//	int count = 0;
//	centerVoxel->gradient = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
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
//		if (neighborVoxel == nullptr || 0 == neighborVoxel->label) continue;
//
//		centerVoxel->gradient +=
//			(neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized() -
//			(centerVoxel->normal / (float)centerVoxel->pointCount).normalized();
//
//		count++;
//	}
//
//	if (0 < count)
//	{
//		centerVoxel->gradient /= (float)count;
//		//printf("%f, %f, %f\n", centerVoxel->gradient.x(), centerVoxel->gradient.y(), centerVoxel->gradient.z());
//	}
//}
//
//void PointCloud::ComputeNormalGradient()
//{
//	nvtxRangePushA("ComputeNormalGradient");
//
//	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_ComputeNormalGradient << <gridOccupied, blockSize >> > (hashmap.info);
//
//	cudaDeviceSynchronize();
//}
//
//__global__ void Kernel_SerializeColoringByNormalGradient(HashMapInfo info, PointCloudBuffers buffers, float threshold)
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
//	auto g = voxel->gradient.normalized();
//
//	float length = voxel->gradient.norm();
//	if (length > threshold)
//	{
//		buffers.colors[idx] = Eigen::Vector4b(255, 0, 0, 255);
//	}
//	else
//	{
//		buffers.colors[idx] = Eigen::Vector4b(100, 100, 100, 255);
//	}
//}
//
//void PointCloud::SerializeColoringByNormalGradient(float threshold, PointCloudBuffers& d_tempBuffers)
//{
//	d_buffers.CopyTo(d_tempBuffers);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
//
//	Kernel_SerializeColoringByNormalGradient << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, threshold);
//
//	cudaDeviceSynchronize();
//}
