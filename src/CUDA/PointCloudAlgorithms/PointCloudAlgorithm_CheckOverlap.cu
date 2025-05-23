#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_CheckOverlap.cuh>
#include <CUDA/PointCloud.cuh>

PointCloudAlgorithm_CheckOverlap::PointCloudAlgorithm_CheckOverlap()
{

}

PointCloudAlgorithm_CheckOverlap::~PointCloudAlgorithm_CheckOverlap()
{

}

__global__ void Kernel_Prepare_CheckOverlap(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	centerVoxel->color = Eigen::Vector4b(100, 100, 100, 255);
}

__global__ void Kernel_CheckOverlap(HashMapInfo info, float angleThreshold)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr) return;

	auto centerNormal = (centerVoxel->normal / (float)centerVoxel->pointCount).normalized();
	auto centerPosition = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);

	int step = 50;
	for (int i = 2; i < step; i++)
	{
		auto position = centerPosition + centerNormal * info.voxelSize * (float)i;

		auto targetCoord = make_int3(
			floorf(position.x() / info.voxelSize),
			floorf(position.y() / info.voxelSize),
			floorf(position.z() / info.voxelSize));

		size_t targetSlot = GetHashMapVoxelSlot(info, targetCoord);
		if (UINT64_MAX == targetSlot) continue;
		HashMapVoxel* targetVoxel = GetHashMapVoxel(info, targetSlot);
		if (targetVoxel == nullptr) continue;

		auto targetNormal = (targetVoxel->normal / (float)targetVoxel->pointCount).normalized();

		if (angleThreshold * M_PI / 180.0f > acosf(centerNormal.dot(targetNormal)))
		{
			targetVoxel->color = Eigen::Vector4b(255, 0, 0, 255);
		}
	}
}

__global__ void Kernel_Serialize_CheckOverlapResult(HashMapInfo info, PointCloudBuffers buffers)
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
	buffers.colors[idx] = voxel->color;

	//printf("%3d, %3d, %3d\n", voxel->color.x(), voxel->color.y(), voxel->color.z());
}

void PointCloudAlgorithm_CheckOverlap::RunAlgorithm(PointCloud* pointCloud)
{
	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

	Kernel_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, angleThreshold);

	gridOccupied = (pointCloud->GetNumberOfPoints() + blockSize - 1) / blockSize;

	Kernel_Serialize_CheckOverlapResult << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, pointCloud->GetDeviceBuffers());

	cudaDeviceSynchronize();
}

void PointCloudAlgorithm_CheckOverlap::IncreaseParameter()
{
	angleThreshold += 1.0f;
}

void PointCloudAlgorithm_CheckOverlap::DecreaseParameter()
{
	angleThreshold -= 1.0f;
}
