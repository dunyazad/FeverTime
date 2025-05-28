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
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	centerVoxel->color = Eigen::Vector4b(100, 100, 100, 255);
}

__global__ void Kernel_CheckOverlap(
	HashMapInfo info,
	float angleThreshold,
	Eigen::Vector3f* d_newPoints,
	Eigen::Vector3f* d_newNormals,
	Eigen::Vector4b* d_newColors,
	unsigned int* numberOfNewPoints)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
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

	int step = 30;
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
			targetVoxel->color = Eigen::Vector4b(255, 0, 0, 255);

			//positionSum += Eigen::Vector3f((float)targetCoord.x * info.voxelSize, (float)targetCoord.y * info.voxelSize, (float)targetCoord.z * info.voxelSize);
			//normalSum += targetNormal;
			//colorSum += Eigen::Vector3f((float)targetVoxel->color.x() / 255.0f, (float)targetVoxel->color.y() / 255.0f, (float)targetVoxel->color.z() / 255.0f);
			//count++;

			//targetVoxel->reservedToDeleted = 1;
		}
	}

	if (1 < count)
	{
		centerVoxel->color = Eigen::Vector4b(255, 0, 0, 255);

	//	centerVoxel->reservedToDeleted = 1;

	//	auto meanPosition = positionSum / (float)count;
	//	int3 meanCoord = make_int3(floorf(meanPosition.x() / info.voxelSize), floorf(meanPosition.y() / info.voxelSize), floorf(meanPosition.z() / info.voxelSize));
	//	auto meanNormal = normalSum / (float)count;
	//	auto meanColor = colorSum / (float)count;

	//	auto index = atomicAdd(numberOfNewPoints, 1);
	//	d_newPoints[index] = meanPosition;
	//	d_newNormals[index] = meanNormal;
	//	d_newColors[index] = Eigen::Vector4b(meanColor.x() * 255.0f, meanColor.y() * 255.0f, meanColor.z() * 255.0f, 255);
	}
}

__global__ void Kernel_Delete_Reserved(HashMapInfo info, unsigned int* d_numberOfDeletedVoxels)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;

	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
	if (centerVoxel == nullptr) return;

	if (1 == centerVoxel->reservedToDeleted)
	{
		DeleteHashMapVoxel(info, coord);

		atomicAdd(d_numberOfDeletedVoxels, 1);
	}
}

__global__ void Kernel_Serialize_CheckOverlapResult(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot)
	{
		buffers.positions[idx] = Eigen::Vector3f::Zero();
		buffers.normals[idx] = Eigen::Vector3f::Zero();
		buffers.colors[idx] = Eigen::Vector4b(255, 255, 255, 255);

		return;
	}

	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
	if (voxel == nullptr || 1 == voxel->deleted || 1 == voxel->reservedToDeleted)
	{
		buffers.positions[idx] = Eigen::Vector3f::Zero();
		buffers.normals[idx] = Eigen::Vector3f::Zero();
		buffers.colors[idx] = Eigen::Vector4b(255, 255, 255, 255);

		return;
	}

	buffers.positions[idx] = voxel->position;
	buffers.normals[idx] = voxel->normal;
	buffers.colors[idx] = voxel->color;

	//printf("%3d, %3d, %3d\n", voxel->color.x(), voxel->color.y(), voxel->color.z());
}

void PointCloudAlgorithm_CheckOverlap::RunAlgorithm(PointCloud* pointCloud)
{
	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;

	Eigen::Vector3f* d_newPoints;
	Eigen::Vector3f* d_newNormals;
	Eigen::Vector4b* d_newColors;
	unsigned int* d_numberOfNewPoints;

	cudaMalloc(&d_newPoints, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels / 2);
	cudaMalloc(&d_newNormals, sizeof(Eigen::Vector3f) * numberOfOccupiedVoxels / 2);
	cudaMalloc(&d_newColors, sizeof(Eigen::Vector4b) * numberOfOccupiedVoxels / 2);
	cudaMalloc(&d_numberOfNewPoints, sizeof(unsigned int));

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

	Kernel_CheckOverlap << <gridOccupied, blockSize >> > (
		pointCloud->GetHashMap().info,
		angleThreshold,
		d_newPoints,
		d_newNormals,
		d_newColors,
		d_numberOfNewPoints);

	unsigned int* d_numberOfDeletedVoxels;
	cudaMalloc(&d_numberOfDeletedVoxels, sizeof(unsigned int));

	Kernel_Delete_Reserved << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, d_numberOfDeletedVoxels);

	//pointCloud->GetHashMap().SerializeToPLY("C:\\Resources\\Debug\\Serialized\\HashMap.ply");

	gridOccupied = (pointCloud->GetNumberOfPoints() + blockSize - 1) / blockSize;

	Kernel_Serialize_CheckOverlapResult << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, pointCloud->GetDeviceBuffers());

	cudaDeviceSynchronize();

	cudaFree(d_newPoints);
	cudaFree(d_newNormals);
	cudaFree(d_newColors);
	cudaFree(d_numberOfNewPoints);

	cudaFree(d_numberOfDeletedVoxels);
}

void PointCloudAlgorithm_CheckOverlap::IncreaseParameter()
{
	angleThreshold += 1.0f;
}

void PointCloudAlgorithm_CheckOverlap::DecreaseParameter()
{
	angleThreshold -= 1.0f;
}
