#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_ClusteringFilter.cuh>

#include <CUDA/PCD.cuh>

PointCloudAlgorithm_ClusteringFilter::PointCloudAlgorithm_ClusteringFilter()
{

}

PointCloudAlgorithm_ClusteringFilter::~PointCloudAlgorithm_ClusteringFilter()
{

}

//vector<uint3> PointCloud::Clustering(float normalDegreeThreshold)
//{
//	nvtxRangePushA("Clustering");
//
//	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
//	
//	vector<uint3> labels;
//
//	{
//		unsigned int blockSize = 256;
//		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//		Kernel_ClearLabels << <gridOccupied, blockSize >> > (hashmap.info);
//
//		cudaDeviceSynchronize();
//	}
//
//	{
//		unsigned int blockSize = 256;
//		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//		Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (hashmap.info, normalDegreeThreshold);
//
//		cudaDeviceSynchronize();
//	}
//
//	{
//		unsigned int blockSize = 256;
//		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//		Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (hashmap.info);
//
//		cudaDeviceSynchronize();
//	}
//
//	hashmap.CountLabels();
//
//	labels.resize(h_buffers.numberOfPoints);
//	{
//		uint3* d_labels = nullptr;
//		cudaMalloc(&d_labels, sizeof(uint3) * h_buffers.numberOfPoints);
//
//		unsigned int blockSize = 256;
//		unsigned int gridOccupied = (h_buffers.numberOfPoints + blockSize - 1) / blockSize;
//
//		Kernel_GetLabels << <gridOccupied, blockSize >> > (hashmap.info, d_buffers.positions, d_labels, h_buffers.numberOfPoints);
//
//		cudaDeviceSynchronize();
//
//		cudaMemcpy(labels.data(), d_labels, sizeof(unsigned int) * h_buffers.numberOfPoints, cudaMemcpyDeviceToHost);
//
//		cudaFree(d_labels);
//	}
//
//	nvtxRangePop();
//
//	return labels;
//}

//void PointCloud::SerializeColoringByLabel(PointCloudBuffers& d_tempBuffers)
//{
//	d_buffers.CopyTo(d_tempBuffers);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
//
//	Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);
//
//	cudaDeviceSynchronize();
//}
//
//__global__ void Kernel_SerializeColoringBySubLabel(HashMapInfo info, PointCloudBuffers buffers)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//	if (idx >= buffers.numberOfPoints) return;
//
//	auto& p = buffers.positions[idx];
//	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
//	size_t h = voxel_hash(coord, info.capacity);
//
//	for (unsigned int i = 0; i < info.maxProbe; ++i)
//	{
//		size_t slot = (h + i) % info.capacity;
//		auto& voxel = info.d_hashTable[slot];
//
//		if (0 == voxel.subLabel) return;
//
//		if (voxel.coord.x == coord.x &&
//			voxel.coord.y == coord.y &&
//			voxel.coord.z == coord.z)
//		{
//			buffers.positions[idx] = voxel.position;
//			buffers.normals[idx] = voxel.normal;
//
//			float r = hashToFloat(voxel.subLabel * 3 + 0);
//			float g = hashToFloat(voxel.subLabel * 3 + 1);
//			float b = hashToFloat(voxel.subLabel * 3 + 2);
//
//			auto subLabelCount = info.labelCounter.GetCount(voxel.subLabel);
//			if (100000 > subLabelCount)
//			{
//				buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 0);
//			}
//			else
//			{
//				buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//			}
//
//			return;
//		}
//	}
//}
//
//void PointCloud::SerializeColoringBySubLabel(PointCloudBuffers& d_tempBuffers)
//{
//	d_buffers.CopyTo(d_tempBuffers);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
//
//	Kernel_SerializeColoringBySubLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);
//
//	cudaDeviceSynchronize();
//}

__global__ void Kernel_SerializeFilteringColoringByLabel(
	HashMapInfo info,
	float3* positions,
	float3* normals,
	uchar4* colors,
	size_t numberOfPoints,
	bool applyColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numberOfPoints) return;

	auto& p = positions[idx];
	int3 coord = make_int3(floorf(p.x / info.voxelSize), floorf(p.y / info.voxelSize), floorf(p.z / info.voxelSize));

	auto slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;

	auto voxel = GetHashMapVoxel(info, slot);
	if (INVALID_VOXEL == voxel) return;

	if (0 == voxel->label) return;

	if (voxel->coord.x == coord.x &&
		voxel->coord.y == coord.y &&
		voxel->coord.z == coord.z)
	{
		if (applyColor)
		{
			normals[idx].x = voxel->normal.x() / (float)voxel->pointCount;
			normals[idx].y = voxel->normal.y() / (float)voxel->pointCount;
			normals[idx].z = voxel->normal.z() / (float)voxel->pointCount;

			float r = hashToFloat(voxel->label * 3 + 0);
			float g = hashToFloat(voxel->label * 3 + 1);
			float b = hashToFloat(voxel->label * 3 + 2);

			colors[idx] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, 255);
		}

		auto labelCount = info.labelCounter.GetCount(voxel->label);
		
		if (20000 > labelCount)
		{
			positions[idx].x = FLT_MAX;
			positions[idx].y = FLT_MAX;
			positions[idx].z = FLT_MAX;
		}

		//{
		//	buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 0);
		//}
		//else
		//{
		//	buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
		//}
	}
}

void PointCloudAlgorithm_ClusteringFilter::RunAlgorithm(DevicePointCloud* pointCloud)
{
	nvtxRangePushA("Clustering");

	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_ClearLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, angleThreshold);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

		cudaDeviceSynchronize();
	}

	pointCloud->GetHashMap().CountLabels();

	{
		unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		auto positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
		auto normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
		auto colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

		Kernel_SerializeFilteringColoringByLabel << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info, positions, normals, colors, numberOfPoints, applyColor);

		cudaDeviceSynchronize();
	}

	/*

	labels.resize(h_buffers.numberOfPoints);
	{
		uint3* d_labels = nullptr;
		cudaMalloc(&d_labels, sizeof(uint3) * h_buffers.numberOfPoints);

		unsigned int blockSize = 256;
		unsigned int gridOccupied = (h_buffers.numberOfPoints + blockSize - 1) / blockSize;

		Kernel_GetLabels << <gridOccupied, blockSize >> > (hashmap.info, d_buffers.positions, d_labels, h_buffers.numberOfPoints);

		cudaDeviceSynchronize();

		cudaMemcpy(labels.data(), d_labels, sizeof(unsigned int) * h_buffers.numberOfPoints, cudaMemcpyDeviceToHost);

		cudaFree(d_labels);
	}*/

	nvtxRangePop();
}

void PointCloudAlgorithm_ClusteringFilter::RunAlgorithm(HostPointCloud* pointCloud)
{
	nvtxRangePushA("Clustering");

	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_ClearLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, angleThreshold);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

		cudaDeviceSynchronize();
	}

	pointCloud->GetHashMap().CountLabels();

	{
		unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		thrust::device_vector<float3> h_positions(pointCloud->GetPositions());
		thrust::device_vector<float3> h_normals(pointCloud->GetNormals());
		thrust::device_vector<uchar4> h_colors(pointCloud->GetColors());

		auto positions = thrust::raw_pointer_cast(h_positions.data());
		auto normals = thrust::raw_pointer_cast(h_normals.data());
		auto colors = thrust::raw_pointer_cast(h_colors.data());

		Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info, positions, normals, colors, numberOfPoints);

		cudaDeviceSynchronize();
	}
}

void PointCloudAlgorithm_ClusteringFilter::IncreaseParameter()
{
	angleThreshold += 1.0f;
}

void PointCloudAlgorithm_ClusteringFilter::DecreaseParameter()
{
	angleThreshold -= 1.0f;
}
