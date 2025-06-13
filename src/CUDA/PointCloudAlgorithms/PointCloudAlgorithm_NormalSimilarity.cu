#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_NormalSimilarity.cuh>

#include <CUDA/PCD.cuh>

PointCloudAlgorithm_NormalSimilarity::PointCloudAlgorithm_NormalSimilarity()
{

}

PointCloudAlgorithm_NormalSimilarity::~PointCloudAlgorithm_NormalSimilarity()
{

}

//__device__ __forceinline__ unsigned int FindRootVoxel(HashMapInfo info, unsigned int idx)
//{
//	while (info.d_hashTable[idx].label != idx)
//	{
//		unsigned int parent = info.d_hashTable[idx].label;
//		unsigned int grandparent = info.d_hashTable[parent].label;
//
//		auto& voxelA = info.d_hashTable[idx];
//		auto& voxelB = info.d_hashTable[parent];
//
//		if (parent != grandparent)
//			info.d_hashTable[idx].label = grandparent;
//
//		idx = info.d_hashTable[idx].label;
//	}
//	return idx;
//}
//
//__device__ __forceinline__ unsigned int FindRootVoxelSub(HashMapInfo info, unsigned int idx)
//{
//	while (info.d_hashTable[idx].subLabel != idx)
//	{
//		unsigned int parent = info.d_hashTable[idx].subLabel;
//		unsigned int grandparent = info.d_hashTable[parent].subLabel;
//
//		auto& voxelA = info.d_hashTable[idx];
//		auto& voxelB = info.d_hashTable[parent];
//
//		if (parent != grandparent)
//			info.d_hashTable[idx].subLabel = grandparent;
//
//		idx = info.d_hashTable[idx].subLabel;
//	}
//	return idx;
//}
//
//__device__ __forceinline__ void UnionVoxel(HashMapInfo info, unsigned int a, unsigned int b)
//{
//	unsigned int rootA = FindRootVoxel(info, a);
//	unsigned int rootB = FindRootVoxel(info, b);
//	if (rootA == rootB) return;
//
//	auto& voxelA = info.d_hashTable[rootA];
//	auto& voxelB = info.d_hashTable[rootB];
//
//	if (rootA < rootB)
//		atomicMin(&voxelB.label, rootA);
//	else
//		atomicMin(&voxelA.label, rootB);
//}
//
//__device__ __forceinline__ void UnionVoxelSub(HashMapInfo info, unsigned int a, unsigned int b)
//{
//	unsigned int rootA = FindRootVoxelSub(info, a);
//	unsigned int rootB = FindRootVoxelSub(info, b);
//	if (rootA == rootB) return;
//
//	auto& voxelA = info.d_hashTable[rootA];
//	auto& voxelB = info.d_hashTable[rootB];
//
//	if (rootA < rootB)
//		atomicMin(&voxelB.subLabel, rootA);
//	else
//		atomicMin(&voxelA.subLabel, rootB);
//}
//
//__global__ void Kernel_ClearLabels(HashMapInfo info)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	auto voxel = GetHashMapVoxel(info, slot);
//
//	voxel->label = slot;
//	voxel->subLabel = slot;
//}
//
//__global__ void Kernel_InterVoxelHashMerge26Way(HashMapInfo info, float normalDegreeThreshold)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	HashMapVoxel* centerVoxel = GetHashMapVoxel(info, slot);
//	//if (centerVoxel == nullptr || centerVoxel->label == 0) return;
//	if (INVALID_VOXEL == centerVoxel) return;
//
//	if (18 <= centerVoxel->emptyNeighborCount) return;
//
//	auto centerNormal = (centerVoxel->normal / (float)centerVoxel->pointCount).normalized();
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
//		//if (neighborVoxel && neighborVoxel->label != 0)
//		if (neighborVoxel)
//		{
//			UnionVoxelSub(info, slot, neighborSlot);
//
//			auto neighborNormal = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();
//			if (normalDegreeThreshold * M_PI / 180.f < acosf(centerNormal.dot(neighborNormal))) continue;
//
//			UnionVoxel(info, slot, neighborSlot);
//		}
//	}
//}
//
//__global__ void Kernel_CompressVoxelHashLabels(HashMapInfo info)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
//	if (INVALID_VOXEL == voxel) return;
//	if (voxel->label == 0) return;
//
//	voxel->label = FindRootVoxel(info, voxel->label);
//}
//
//__global__ void Kernel_GetLabels(HashMapInfo info, Eigen::Vector3f* points, uint3* labels, unsigned int numberOfPoints)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= numberOfPoints) return;
//
//	auto& p = points[threadid];
//	int3 coord = make_int3(
//		floorf(p.x() / info.voxelSize),
//		floorf(p.y() / info.voxelSize),
//		floorf(p.z() / info.voxelSize));
//
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
//	if (INVALID_VOXEL == voxel) return;
//	if (voxel->label == 0) return;
//
//	labels[threadid].x = voxel->label;
//	labels[threadid].y = info.labelCounter.GetCount(voxel->label);
//	labels[threadid].z = info.subLabelCounter.GetCount(voxel->label);
//}
//
////vector<uint3> PointCloud::Clustering(float normalDegreeThreshold)
////{
////	nvtxRangePushA("Clustering");
////
////	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
////	
////	vector<uint3> labels;
////
////	{
////		unsigned int blockSize = 256;
////		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
////
////		Kernel_ClearLabels << <gridOccupied, blockSize >> > (hashmap.info);
////
////		cudaDeviceSynchronize();
////	}
////
////	{
////		unsigned int blockSize = 256;
////		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
////
////		Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (hashmap.info, normalDegreeThreshold);
////
////		cudaDeviceSynchronize();
////	}
////
////	{
////		unsigned int blockSize = 256;
////		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
////
////		Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (hashmap.info);
////
////		cudaDeviceSynchronize();
////	}
////
////	hashmap.CountLabels();
////
////	labels.resize(h_buffers.numberOfPoints);
////	{
////		uint3* d_labels = nullptr;
////		cudaMalloc(&d_labels, sizeof(uint3) * h_buffers.numberOfPoints);
////
////		unsigned int blockSize = 256;
////		unsigned int gridOccupied = (h_buffers.numberOfPoints + blockSize - 1) / blockSize;
////
////		Kernel_GetLabels << <gridOccupied, blockSize >> > (hashmap.info, d_buffers.positions, d_labels, h_buffers.numberOfPoints);
////
////		cudaDeviceSynchronize();
////
////		cudaMemcpy(labels.data(), d_labels, sizeof(unsigned int) * h_buffers.numberOfPoints, cudaMemcpyDeviceToHost);
////
////		cudaFree(d_labels);
////	}
////
////	nvtxRangePop();
////
////	return labels;
////}
//
//__global__ void Kernel_SerializeColoringByLabel(HashMapInfo info, float3* positions, float3* normals, uchar4* colors, size_t numberOfPoints)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//	if (idx >= numberOfPoints) return;
//
//	auto& p = positions[idx];
//	int3 coord = make_int3(floorf(p.x / info.voxelSize), floorf(p.y / info.voxelSize), floorf(p.z / info.voxelSize));
//
//	auto slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	auto voxel = GetHashMapVoxel(info, slot);
//	if (INVALID_VOXEL == voxel) return;
//
//	if (0 == voxel->label) return;
//
//	//positions[idx].x = p.x;
//	//positions[idx].y = p.y;
//	//positions[idx].z = p.z;
//
//	normals[idx].x = voxel->normal.x() / (float)voxel->pointCount;
//	normals[idx].y = voxel->normal.y() / (float)voxel->pointCount;
//	normals[idx].z = voxel->normal.z() / (float)voxel->pointCount;
//
//	float r = hashToFloat(voxel->label * 3 + 0);
//	float g = hashToFloat(voxel->label * 3 + 1);
//	float b = hashToFloat(voxel->label * 3 + 2);
//
//	colors[idx] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//
//	//auto labelCount = info.labelCounter.GetCount(voxel.label);
//	//if (200000 > labelCount)
//	//{
//	//	buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 0);
//	//}
//	//else
//	//{
//	//	buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//	//}
//}
//
////void PointCloud::SerializeColoringByLabel(PointCloudBuffers& d_tempBuffers)
////{
////	d_buffers.CopyTo(d_tempBuffers);
////
////	unsigned int blockSize = 256;
////	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
////
////	Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);
////
////	cudaDeviceSynchronize();
////}
////
////__global__ void Kernel_SerializeColoringBySubLabel(HashMapInfo info, PointCloudBuffers buffers)
////{
////	int idx = blockIdx.x * blockDim.x + threadIdx.x;
////	if (idx >= buffers.numberOfPoints) return;
////
////	auto& p = buffers.positions[idx];
////	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
////	size_t h = voxel_hash(coord, info.capacity);
////
////	for (unsigned int i = 0; i < info.maxProbe; ++i)
////	{
////		size_t slot = (h + i) % info.capacity;
////		auto& voxel = info.d_hashTable[slot];
////
////		if (0 == voxel.subLabel) return;
////
////		if (voxel.coord.x == coord.x &&
////			voxel.coord.y == coord.y &&
////			voxel.coord.z == coord.z)
////		{
////			buffers.positions[idx] = voxel.position;
////			buffers.normals[idx] = voxel.normal;
////
////			float r = hashToFloat(voxel.subLabel * 3 + 0);
////			float g = hashToFloat(voxel.subLabel * 3 + 1);
////			float b = hashToFloat(voxel.subLabel * 3 + 2);
////
////			auto subLabelCount = info.labelCounter.GetCount(voxel.subLabel);
////			if (100000 > subLabelCount)
////			{
////				buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 0);
////			}
////			else
////			{
////				buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
////			}
////
////			return;
////		}
////	}
////}
////
////void PointCloud::SerializeColoringBySubLabel(PointCloudBuffers& d_tempBuffers)
////{
////	d_buffers.CopyTo(d_tempBuffers);
////
////	unsigned int blockSize = 256;
////	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;
////
////	Kernel_SerializeColoringBySubLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);
////
////	cudaDeviceSynchronize();
////}

__global__ void NormalSimilarity(HashMapInfo info, float3* positions, float3* normals, uchar4* colors, size_t numberOfPoints, bool removeCheckedPoints)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numberOfPoints) return;
	
	auto& p = positions[idx];
	auto& n = normals[idx];
	int3 coord = make_int3(floorf(p.x / info.voxelSize), floorf(p.y / info.voxelSize), floorf(p.z / info.voxelSize));
	
	auto slot = GetHashMapVoxelSlot(info, coord);
	if (INVALID_VOXEL_SLOT == slot) return;
	
	auto voxel = GetHashMapVoxel(info, slot);
	if (INVALID_VOXEL == voxel) return;
	
	float similaritySum = 0.f;
	unsigned int count = 0;

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetHashMapVoxelSlot(info, neighborCoord);
		if (INVALID_VOXEL_SLOT == slot) continue;

		HashMapVoxel* neighborVoxel = GetHashMapVoxel(info, neighborSlot);
		if (INVALID_VOXEL == neighborVoxel) continue;

		float3 neighborCenter = make_float3(
			((float)neighborCoord.x + 0.5f) * info.voxelSize,
			((float)neighborCoord.y + 0.5f) * info.voxelSize,
			((float)neighborCoord.z + 0.5f) * info.voxelSize
		);

		auto neighborNormal_ = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();
		auto nn = make_float3(neighborNormal_.x(), neighborNormal_.y(), neighborNormal_.z());

		similaritySum += dot(n, nn);
		count++;
	}

	float similarity = similaritySum / (float)count;

	if (0.95f > similarity)
	{
		if (removeCheckedPoints)
		{
			positions[idx].x = FLT_MAX;
			positions[idx].y = FLT_MAX;
			positions[idx].z = FLT_MAX;
		}
		else
		{
			colors[idx].x = 255;
			colors[idx].y = 0;
			colors[idx].z = 0;
			colors[idx].w = 255;
		}
	}
}

void PointCloudAlgorithm_NormalSimilarity::RunAlgorithm(DevicePointCloud* pointCloud)
{
	nvtxRangePushA("Laplacian");

	unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;

	{
		unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

		auto positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
		auto normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
		auto colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

		NormalSimilarity << <gridOccupied, blockSize >> > (
			pointCloud->GetHashMap().info, positions, normals, colors, numberOfPoints, removeCheckedPoints);

		cudaDeviceSynchronize();

		if (removeCheckedPoints)
		{
			pointCloud->Compact();
		}
	}

	//{
	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	//	Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, angleThreshold);

	//	cudaDeviceSynchronize();
	//}

	//{
	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	//	Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

	//	cudaDeviceSynchronize();
	//}

	//pointCloud->GetHashMap().CountLabels();

	//{
	//	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

	//	auto positions = thrust::raw_pointer_cast(pointCloud->GetPositions().data());
	//	auto normals = thrust::raw_pointer_cast(pointCloud->GetNormals().data());
	//	auto colors = thrust::raw_pointer_cast(pointCloud->GetColors().data());

	//	Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (
	//		pointCloud->GetHashMap().info, positions, normals, colors, numberOfPoints);

	//	cudaDeviceSynchronize();
	//}

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
	}
	*/

	nvtxRangePop();
}

void PointCloudAlgorithm_NormalSimilarity::RunAlgorithm(HostPointCloud* pointCloud)
{
	//nvtxRangePushA("Clustering");

	//unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;

	//{
	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	//	Kernel_ClearLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

	//	cudaDeviceSynchronize();
	//}

	//{
	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	//	Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, angleThreshold);

	//	cudaDeviceSynchronize();
	//}

	//{
	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	//	Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);

	//	cudaDeviceSynchronize();
	//}

	//pointCloud->GetHashMap().CountLabels();

	//{
	//	unsigned int numberOfPoints = pointCloud->GetNumberOfElements();

	//	unsigned int blockSize = 256;
	//	unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

	//	thrust::device_vector<float3> h_positions(pointCloud->GetPositions());
	//	thrust::device_vector<float3> h_normals(pointCloud->GetNormals());
	//	thrust::device_vector<uchar4> h_colors(pointCloud->GetColors());

	//	auto positions = thrust::raw_pointer_cast(h_positions.data());
	//	auto normals = thrust::raw_pointer_cast(h_normals.data());
	//	auto colors = thrust::raw_pointer_cast(h_colors.data());

	//	Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (
	//		pointCloud->GetHashMap().info, positions, normals, colors, numberOfPoints);

	//	cudaDeviceSynchronize();
	//}
}

void PointCloudAlgorithm_NormalSimilarity::IncreaseParameter()
{
}

void PointCloudAlgorithm_NormalSimilarity::DecreaseParameter()
{
}
