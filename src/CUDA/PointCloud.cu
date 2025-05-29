#include <CUDA/PointCloud.cuh>

//PointCloud::PointCloud()
//{
//}
//
//PointCloud::~PointCloud()
//{
//}
//
//void PointCloud::Initialize(unsigned int numberOfPoints)
//{
//	h_buffers.Initialize(numberOfPoints, true);
//	d_buffers.Initialize(numberOfPoints, false);
//
//	hashmap.Initialize();
//}
//
//void PointCloud::Terminate()
//{
//	h_buffers.Terminate();
//	d_buffers.Terminate();
//
//	hashmap.Terminate();
//}
//
//void PointCloud::CopyTo(PointCloud& other)
//{
//	if (0 == h_buffers.numberOfPoints && 0 == d_buffers.numberOfPoints) return;
//
//	h_buffers.CopyTo(other.h_buffers);
//	d_buffers.CopyTo(other.d_buffers);
//
//	if (0 == other.h_buffers.numberOfPoints)
//	{
//		other.DtoH();
//	}
//
//	if (0 == other.d_buffers.numberOfPoints)
//	{
//		other.HtoD();
//	}
//
//	other.hashmap.InsertPoints(other.d_buffers);
//}
//
//void PointCloud::HtoD()
//{
//	h_buffers.CopyTo(d_buffers);
//}
//
//void PointCloud::DtoH()
//{
//	d_buffers.CopyTo(h_buffers);
//}
//
//bool PointCloud::LoadFromPLY(const std::string& filename)
//{
//	PLYFormat ply;
//	if (false == ply.Deserialize(filename))
//	{
//		return false;
//	}
//
//	Initialize(ply.GetPoints().size() / 3);
//
//	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
//	{
//		auto x = ply.GetPoints()[i * 3];
//		auto y = ply.GetPoints()[i * 3 + 1];
//		auto z = ply.GetPoints()[i * 3 + 2];
//
//		auto nx = ply.GetNormals()[i * 3];
//		auto ny = ply.GetNormals()[i * 3 + 1];
//		auto nz = ply.GetNormals()[i * 3 + 2];
//
//		auto r = ply.GetColors()[i * 3];
//		auto g = ply.GetColors()[i * 3 + 1];
//		auto b = ply.GetColors()[i * 3 + 2];
//
//		h_buffers.positions[i] = Eigen::Vector3f(x, y, z);
//		h_buffers.normals[i] = Eigen::Vector3f(nx, ny, nz);
//		h_buffers.colors[i] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//
//		h_buffers.aabb.extend(Eigen::Vector3f(x, y, z));
//	}
//
//	HtoD();
//
//	hashmap.InsertPoints(d_buffers);
//
//	return true;
//}
//
//bool PointCloud::LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi)
//{
//	PLYFormat ply;
//	if (false == ply.Deserialize(filename))
//	{
//		return false;
//	}
//
//	unsigned int numberOfPoints = 0;
//
//	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
//	{
//		auto x = ply.GetPoints()[i * 3];
//		auto y = ply.GetPoints()[i * 3 + 1];
//		auto z = ply.GetPoints()[i * 3 + 2];
//
//		if (roi.contains(Eigen::Vector3f(x, y, z))) numberOfPoints++;
//	}
//
//	Initialize(numberOfPoints);
//
//	unsigned int bufferIndex = 0;
//	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
//	{
//		auto x = ply.GetPoints()[i * 3];
//		auto y = ply.GetPoints()[i * 3 + 1];
//		auto z = ply.GetPoints()[i * 3 + 2];
//
//		if(false == roi.contains(Eigen::Vector3f(x,y,z))) continue;
//
//		auto nx = ply.GetNormals()[i * 3];
//		auto ny = ply.GetNormals()[i * 3 + 1];
//		auto nz = ply.GetNormals()[i * 3 + 2];
//
//		auto r = ply.GetColors()[i * 3];
//		auto g = ply.GetColors()[i * 3 + 1];
//		auto b = ply.GetColors()[i * 3 + 2];
//
//		h_buffers.positions[bufferIndex] = Eigen::Vector3f(x, y, z);
//		h_buffers.normals[bufferIndex] = Eigen::Vector3f(nx, ny, nz);
//		h_buffers.colors[bufferIndex] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//
//		h_buffers.aabb.extend(Eigen::Vector3f(x, y, z));
//
//		bufferIndex++;
//	}
//
//	HtoD();
//
//	hashmap.InsertPoints(d_buffers);
//
//	return true;
//}
//
//bool PointCloud::SaveToPLY(const std::string& filename)
//{
//	PLYFormat ply;
//
//	for (size_t i = 0; i < h_buffers.numberOfPoints; i++)
//	{
//		ply.AddPointFloat3(h_buffers.positions[i].data());
//		ply.AddNormalFloat3(h_buffers.normals[i].data());
//		ply.AddColor(h_buffers.colors[i].x(), h_buffers.colors[i].y(), h_buffers.colors[i].z());
//	}
//
//	ply.Serialize(filename);
//
//	return true;
//}
//
//bool PointCloud::LoadFromALP(const std::string& filename)
//{
//	ALPFormat<PointPNC> alp;
//	if (false == alp.Deserialize(filename))
//	{
//		return false;
//	}
//
//	//printf("min: %f, %f, %f\n", get<0>(alp.GetAABBMin()), get<1>(alp.GetAABBMin()), get<2>(alp.GetAABBMin()));
//	//printf("max: %f, %f, %f\n", get<0>(alp.GetAABBMax()), get<1>(alp.GetAABBMax()), get<2>(alp.GetAABBMax()));
//
//	Initialize(alp.GetPoints().size());
//
//	for (size_t i = 0; i < alp.GetPoints().size(); i++)
//	{
//		auto& p = alp.GetPoints()[i];
//
//		h_buffers.positions[i] = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
//		h_buffers.normals[i] = Eigen::Vector3f(p.normal.x, p.normal.y, p.normal.z);
//		h_buffers.colors[i] = Eigen::Vector4b(p.color.x * 255.0f, p.color.y * 255.0f, p.color.z * 255.0f, 255);
//
//		h_buffers.aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
//	}
//
//	HtoD();
//
//	hashmap.InsertPoints(d_buffers);
//
//	//hashmap.SerializeToPLY("../../res/test.ply");
//
//	return true;
//}
//
//bool PointCloud::LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi)
//{
//	ALPFormat<PointPNC> alp;
//	if (false == alp.Deserialize(filename))
//	{
//		return false;
//	}
//
//	unsigned int numberOfPoints = 0;
//
//	for (size_t i = 0; i < alp.GetPoints().size(); i++)
//	{
//		auto& p = alp.GetPoints()[i];
//
//		if (roi.contains(Eigen::Vector3f(p.position.x, p.position.y, p.position.z))) numberOfPoints++;
//	}
//
//	Initialize(numberOfPoints);
//
//	unsigned int bufferIndex = 0;
//	for (size_t i = 0; i < alp.GetPoints().size(); i++)
//	{
//		auto& p = alp.GetPoints()[i];
//
//		if (false == roi.contains(Eigen::Vector3f(p.position.x, p.position.y, p.position.z))) continue;
//
//		h_buffers.positions[bufferIndex] = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
//		h_buffers.normals[bufferIndex] = Eigen::Vector3f(p.normal.x, p.normal.y, p.normal.z);
//		h_buffers.colors[bufferIndex] = Eigen::Vector4b(p.color.x * 255.0f, p.color.y * 255.0f, p.color.z * 255.0f, 255);
//
//		h_buffers.aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
//
//		bufferIndex++;
//	}
//
//	HtoD();
//
//	hashmap.InsertPoints(d_buffers);
//
//	//hashmap.SerializeToPLY("../../res/test.ply");
//
//	return true;
//}
//
//bool PointCloud::SaveToALP(const std::string& filename)
//{
//	ALPFormat<PointPNC> alp;
//	
//	for (size_t i = 0; i < h_buffers.numberOfPoints; i++)
//	{
//		PointPNC p;
//		p.position = make_float3(h_buffers.positions[i].x(), h_buffers.positions[i].y(), h_buffers.positions[i].z());
//		p.normal = make_float3(h_buffers.normals[i].x(), h_buffers.normals[i].y(), h_buffers.normals[i].z());
//		p.color = make_float3((float)h_buffers.colors[i].x() / 255.0f, (float)h_buffers.colors[i].y() / 255.0f, (float)h_buffers.colors[i].z() / 255.0f);
//		
//		alp.AddPoint(p);
//	}
//
//	alp.Serialize(filename);
//
//	return true;
//}
//
//__global__ void Kernel_SerializeVoxels(HashMapInfo info, PointCloudBuffers buffers)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == INVALID_VOXEL_SLOT == slot) return;
//
//	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
//	if (voxel == nullptr) return;
//
//	auto position = Eigen::Vector3f(coord.x * info.voxelSize, coord.y * info.voxelSize, coord.z * info.voxelSize);
//	auto normal = (voxel->normal / (float)voxel->pointCount).normalized();
//	auto color = voxel->color;
//
//	buffers.positions[threadid] = position;
//	buffers.normals[threadid] = normal;
//	buffers.colors[threadid] = color;
//}
//
//void PointCloud::SerializeVoxels(PointCloudBuffers& d_tempBuffers)
//{
//	nvtxRangePushA("SerializeVoxels");
//
//	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_SerializeVoxels << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);
//
//	cudaDeviceSynchronize();
//}
//
//__global__ void Kernel_SerializeVoxelsColoringByLabel(HashMapInfo info, PointCloudBuffers buffers)
//{
//	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
//	if (threadid >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[threadid];
//	size_t slot = GetHashMapVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == slot) return;
//
//	HashMapVoxel* voxel = GetHashMapVoxel(info, slot);
//	if (voxel == nullptr) return;
//
//	auto position = Eigen::Vector3f(coord.x * info.voxelSize, coord.y * info.voxelSize, coord.z * info.voxelSize);
//	auto normal = (voxel->normal / (float)voxel->pointCount).normalized();
//	
//	float r = hashToFloat(voxel->label * 3 + 0);
//	float g = hashToFloat(voxel->label * 3 + 1);
//	float b = hashToFloat(voxel->label * 3 + 2);
//
//	auto color = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//
//	buffers.positions[threadid] = position;
//	buffers.normals[threadid] = normal;
//	buffers.colors[threadid] = color;
//}
//
//void PointCloud::SerializeVoxelsColoringByLabel(PointCloudBuffers& d_tempBuffers)
//{
//	nvtxRangePushA("SerializeVoxelsColoringByLabel");
//
//	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_SerializeVoxelsColoringByLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);
//
//	cudaDeviceSynchronize();
//}
//
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
//	if (centerVoxel == nullptr) return;
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
//	size_t h = voxel_hash(coord, info.capacity);
//	for (int i = 0; i < info.maxProbe; ++i)
//	{
//		size_t probe = (h + i) % info.capacity;
//		auto& voxel = info.d_hashTable[probe];
//
//		if (0 == voxel.label) break;
//
//		voxel.label = FindRootVoxel(info, voxel.label);
//	}
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
//	size_t h = voxel_hash(coord, info.capacity);
//	labels[threadid].x = UINT_MAX;
//	labels[threadid].y = 0;
//	labels[threadid].z = 0;
//
//	for (int i = 0; i < info.maxProbe; ++i)
//	{
//		size_t probe = (h + i) % info.capacity;
//		if (info.d_hashTable[probe].label == 0) break;
//
//		auto& voxel = info.d_hashTable[probe];
//
//		if (voxel.coord.x == coord.x &&
//			voxel.coord.y == coord.y &&
//			voxel.coord.z == coord.z)
//		{
//			labels[threadid].x = voxel.label;
//			labels[threadid].y = info.labelCounter.GetCount(voxel.label);
//			labels[threadid].z = info.subLabelCounter.GetCount(voxel.label);
//			return;
//		}
//	}
//}
//
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
//
//__global__ void Kernel_SerializeColoringByLabel(HashMapInfo info, PointCloudBuffers buffers)
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
//		if (0 == voxel.label) return;
//
//		if (voxel.coord.x == coord.x &&
//			voxel.coord.y == coord.y &&
//			voxel.coord.z == coord.z)
//		{
//			buffers.positions[idx] = voxel.position;
//			buffers.normals[idx] = voxel.normal;
//
//			float r = hashToFloat(voxel.label * 3 + 0);
//			float g = hashToFloat(voxel.label * 3 + 1);
//			float b = hashToFloat(voxel.label * 3 + 2);
//
//			buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//
//			//auto labelCount = info.labelCounter.GetCount(voxel.label);
//			//if (200000 > labelCount)
//			//{
//			//	buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 0);
//			//}
//			//else
//			//{
//			//	buffers.colors[idx] = Eigen::Vector4b(r * 255.0f, g * 255.0f, b * 255.0f, 255);
//			//}
//
//			return;
//		}
//	}
//}
//
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
