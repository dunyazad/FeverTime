#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Smoothing.cuh>
#include <CUDA/PCD.cuh>

PointCloudAlgorithm_Smoothing::PointCloudAlgorithm_Smoothing()
{

}

PointCloudAlgorithm_Smoothing::~PointCloudAlgorithm_Smoothing()
{

}

__global__ void Kernel_Smoothing(
	HashMapInfo info,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints,
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
	if (INVALID_VOXEL == centerVoxel) return;

	Eigen::Vector3f psum = Eigen::Vector3f::Zero();
	int count = 0;

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetHashMapVoxelSlot(info, neighborCoord);
		if (INVALID_VOXEL_SLOT == neighborSlot) continue;

		HashMapVoxel* neighborVoxel = GetHashMapVoxel(info, neighborSlot);
		if (INVALID_VOXEL == neighborVoxel) continue;

		psum += neighborVoxel->position;
		count++;
	}

	centerVoxel->position = psum / (float)count;
}

__global__ void Kernel_Apply(
	HashMapInfo info,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints,
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
	if (INVALID_VOXEL == centerVoxel) return;

	positions[threadid] = make_float3(centerVoxel->position.x(), centerVoxel->position.y(), centerVoxel->position.z());
}

void PointCloudAlgorithm_Smoothing::RunAlgorithm(DevicePointCloud* pointCloud)
{
	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;
	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		auto d_positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
		auto d_normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
		auto d_colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

		Kernel_Smoothing << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			applyColor);

		Kernel_Apply << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			applyColor);
	}

	cudaDeviceSynchronize();
}

void PointCloudAlgorithm_Smoothing::RunAlgorithm(HostPointCloud* pointCloud)
{
	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;
	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		auto d_positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
		auto d_normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
		auto d_colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

		Kernel_Smoothing << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info,
			d_positions,
			d_normals,
			d_colors,
			numberOfPoints,
			applyColor);
	}

	cudaDeviceSynchronize();
}

void PointCloudAlgorithm_Smoothing::IncreaseParameter()
{
}

void PointCloudAlgorithm_Smoothing::DecreaseParameter()
{
}
