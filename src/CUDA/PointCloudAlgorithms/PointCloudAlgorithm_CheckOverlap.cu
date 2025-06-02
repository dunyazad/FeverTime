#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_CheckOverlap.cuh>
#include <CUDA/PCD.cuh>

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
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	centerVoxel->color = Eigen::Vector4b(100, 100, 100, 255);
}

__global__ void Kernel_CheckOverlap(
	HashMapInfo info,
	float angleThreshold,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints,
	int step,
	bool applyColor)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= numberOfPoints) return;

	auto p = positions[threadid];
	auto n = normals[threadid];
	auto c = colors[threadid];

	int3 coord = make_int3(floorf(p.x / info.voxelSize), floorf(p.y / info.voxelSize), floorf(p.z / info.voxelSize));

	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr) return;

	auto centerNormal = (centerVoxel->normal / (float)centerVoxel->pointCount).normalized();
	auto centerPosition = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);
	auto centerColor = centerVoxel->color;

	int count = 1;
	Eigen::Vector3f positionSum = centerPosition;
	Eigen::Vector3f normalSum = centerNormal;
	Eigen::Vector3f colorSum = Eigen::Vector3f((float)centerColor.x() / 255.0f, (float)centerColor.y() / 255.0f, (float)centerColor.z() / 255.0f);

	for (int i = 2; i < step; i++)
	{
		auto position = centerPosition + centerNormal * info.voxelSize * (float)i;

		auto targetCoord = make_int3(
			floorf(position.x() / info.voxelSize),
			floorf(position.y() / info.voxelSize),
			floorf(position.z() / info.voxelSize));

		size_t targetSlot = GetHashMapVoxelSlot(info, targetCoord);
		if (INVALID_VOXEL_SLOT == targetSlot) continue;
		HashMapVoxel* targetVoxel = GetHashMapVoxel(info, targetSlot);
		if (targetVoxel == nullptr) continue;

		auto targetNormal = (targetVoxel->normal / (float)targetVoxel->pointCount).normalized();

		if (angleThreshold * M_PI / 180.0f > acosf(centerNormal.dot(targetNormal)))
		{
			//targetVoxel->color = Eigen::Vector4b(255, 0, 0, 255);
			count++;
		}
	}

	if (1 < count)
	{
		if (applyColor)
		{
			centerVoxel->color = Eigen::Vector4b(255, 0, 0, 255);
		}

		positions[threadid].x = FLT_MAX;
		positions[threadid].y = FLT_MAX;
		positions[threadid].z = FLT_MAX;
	}

	if (applyColor)
	{
		colors[threadid].x = centerVoxel->color.x();
		colors[threadid].y = centerVoxel->color.y();
		colors[threadid].z = centerVoxel->color.z();
		colors[threadid].w = centerVoxel->color.w();
	}
}

void PointCloudAlgorithm_CheckOverlap::RunAlgorithm(DevicePointCloud* pointCloud)
{
	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;
	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		auto d_positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
		auto d_normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
		auto d_colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

		Kernel_CheckOverlap << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			step,
			applyColor);
	}

	cudaDeviceSynchronize();
}

void PointCloudAlgorithm_CheckOverlap::RunAlgorithm(HostPointCloud* pointCloud)
{
	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;
	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		thrust::device_vector<float3> dv_positions(pointCloud->GetPositions());
		thrust::device_vector<float3> dv_normals(pointCloud->GetNormals());
		thrust::device_vector<uchar4> dv_colors(pointCloud->GetColors());

		auto d_positions = thrust::raw_pointer_cast(dv_positions.data());
		auto d_normals = thrust::raw_pointer_cast(dv_normals.data());
		auto d_colors = thrust::raw_pointer_cast(dv_colors.data());

		Kernel_CheckOverlap << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			step,
			applyColor);
	}

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
