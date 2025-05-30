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
