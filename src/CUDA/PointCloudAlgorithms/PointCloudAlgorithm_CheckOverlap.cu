#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_CheckOverlap.cuh>
#include <CUDA/PCD.cuh>

PointCloudAlgorithm_CheckOverlap::PointCloudAlgorithm_CheckOverlap()
{

}

PointCloudAlgorithm_CheckOverlap::~PointCloudAlgorithm_CheckOverlap()
{

}

__global__ void Kernel_Prepare_CheckOverlap(
	HashMapInfo info,
	float angleThreshold,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints)
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
	if (INVALID_VOXEL == centerVoxel) return;

	centerVoxel->color = Eigen::Vector4b(100, 100, 100, 255);
	centerVoxel->reservedToDeleted = 0;
}

__global__ void Kernel_CheckOverlap(
	HashMapInfo info,
	float angleThreshold,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints,
	int step,
	bool removeCheckedPoints)
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
	if (INVALID_VOXEL == centerVoxel) return;

	auto centerNormal = (centerVoxel->normal / (float)centerVoxel->pointCount).normalized();
	auto centerPosition = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);
	auto centerColor = centerVoxel->color;

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
		if (INVALID_VOXEL == targetVoxel) continue;

		auto targetNormal = (targetVoxel->normal / (float)targetVoxel->pointCount).normalized();

		if (angleThreshold * M_PI / 180.0f > acosf(centerNormal.dot(targetNormal)))
		{
			//targetVoxel->color = Eigen::Vector4b(255, 0, 0, 255);
			targetVoxel->reservedToDeleted = 1;
			targetVoxel->positionToMove = centerPosition;

//#pragma unroll
//			for (int ni = 0; ni < 26; ++ni)
//			{
//				int3 neighborCoord = make_int3(
//					targetCoord.x + neighbor_offsets_26[ni].x,
//					targetCoord.y + neighbor_offsets_26[ni].y,
//					targetCoord.z + neighbor_offsets_26[ni].z);
//
//				size_t neighborSlot = GetHashMapVoxelSlot(info, neighborCoord);
//				if (INVALID_VOXEL_SLOT == slot) continue;
//
//				HashMapVoxel* neighborVoxel = GetHashMapVoxel(info, neighborSlot);
//				if (INVALID_VOXEL == neighborVoxel) continue;
//
//				auto neighborNormal = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();
//				if (angleThreshold * M_PI / 180.0f > acosf(centerNormal.dot(neighborNormal)))
//				{
//					neighborVoxel->reservedToDeleted = 1;
//				}
//			}
		}
	}
}

__global__ void Kernel_ApplyColor(
	HashMapInfo info,
	float angleThreshold,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints,
	int step,
	bool removeCheckedPoints)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= numberOfPoints) return;

	auto p = positions[threadid];
	auto n = normals[threadid];
	auto c = colors[threadid];

	int3 coord = make_int3(floorf(p.x / info.voxelSize), floorf(p.y / info.voxelSize), floorf(p.z / info.voxelSize));

	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
	if (INVALID_VOXEL == voxel) return;

	if (1 == voxel->reservedToDeleted)
	{
		if (true == removeCheckedPoints)
		{
			//positions[threadid].x = FLT_MAX;
			//positions[threadid].y = FLT_MAX;
			//positions[threadid].z = FLT_MAX;

			positions[threadid].x = voxel->positionToMove.x();
			positions[threadid].y = voxel->positionToMove.y();
			positions[threadid].z = voxel->positionToMove.z();

			//voxel->deleted = 1;
		}
		else
		{
			colors[threadid].x = 255;
			colors[threadid].y = 0;
			colors[threadid].z = 0;
			colors[threadid].w = 255;
		}
	}
}

void PointCloudAlgorithm_CheckOverlap::RunAlgorithm(DevicePointCloud* pointCloud)
{
	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	auto d_positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
	auto d_normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
	auto d_colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints);
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		Kernel_CheckOverlap << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			step,
			removeCheckedPoints);
	}

	cudaDeviceSynchronize();

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		Kernel_ApplyColor << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			step,
			removeCheckedPoints);
	}

	cudaDeviceSynchronize();
}

void PointCloudAlgorithm_CheckOverlap::RunAlgorithm(HostPointCloud* pointCloud)
{
	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	thrust::device_vector<float3> dv_positions(pointCloud->GetPositions());
	thrust::device_vector<float3> dv_normals(pointCloud->GetNormals());
	thrust::device_vector<uchar4> dv_colors(pointCloud->GetColors());

	auto d_positions = thrust::raw_pointer_cast(dv_positions.data());
	auto d_normals = thrust::raw_pointer_cast(dv_normals.data());
	auto d_colors = thrust::raw_pointer_cast(dv_colors.data());

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints);
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		Kernel_CheckOverlap << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			step,
			removeCheckedPoints);
	}

	cudaDeviceSynchronize();

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		Kernel_ApplyColor << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			angleThreshold,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			step,
			removeCheckedPoints);
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
