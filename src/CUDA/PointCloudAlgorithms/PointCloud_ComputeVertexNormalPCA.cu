#include <CUDA/PointCloud.cuh>

__global__ void Kernel_ComputeVoxelNormalPCA(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	auto centerPosition = Eigen::Vector3f(
		coord.x * info.voxelSize,
		coord.y * info.voxelSize,
		coord.z * info.voxelSize);

	Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
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

		auto neighborPosition = Eigen::Vector3f(
			neighborCoord.x * info.voxelSize,
			neighborCoord.y * info.voxelSize,
			neighborCoord.z * info.voxelSize);

		Eigen::Vector3f d = neighborPosition - centerPosition;
		cov += d * d.transpose();
		neighborCount++;
	}

	cov /= (float)neighborCount;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
	centerVoxel->normal = solver.eigenvectors().col(0);
}

void PointCloud::ComputeVoxelNormalPCA()
{
	nvtxRangePushA("Compute Voxel Normal PCA");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeVoxelNormalPCA << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();

	nvtxRangePop();
}
