#include <PointCloud.cuh>

#include <Serialization.hpp>

PointCloud::PointCloud()
{
}

PointCloud::~PointCloud()
{
}

void PointCloud::Initialize(unsigned int numberOfPoints)
{
	h_buffers.Initialize(numberOfPoints, true);
	d_buffers.Initialize(numberOfPoints, false);

	hashmap.Initialize();
}

void PointCloud::Terminate()
{
	h_buffers.Terminate();
	d_buffers.Terminate();

	hashmap.Terminate();
}

void PointCloud::HtoD()
{
	h_buffers.CopyTo(d_buffers);
}

void PointCloud::DtoH()
{
	d_buffers.CopyTo(h_buffers);
}

bool PointCloud::LoadFromPLY(const std::string& filename)
{
	PLYFormat ply;
	if (false == ply.Deserialize(filename))
	{
		return false;
	}

	Initialize(ply.GetPoints().size() / 3);

	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];

		auto nx = ply.GetNormals()[i * 3];
		auto ny = ply.GetNormals()[i * 3 + 1];
		auto nz = ply.GetNormals()[i * 3 + 2];

		auto r = ply.GetColors()[i * 3];
		auto g = ply.GetColors()[i * 3 + 1];
		auto b = ply.GetColors()[i * 3 + 2];

		h_buffers.positions[i] = Eigen::Vector3f(x, y, z);
		h_buffers.normals[i] = Eigen::Vector3f(nx, ny, nz);
		h_buffers.colors[i] = Eigen::Vector3b(r * 255.0f, g * 255.0f, b * 255.0f);

		h_buffers.aabb.extend(Eigen::Vector3f(x, y, z));
	}

	HtoD();

	hashmap.InsertPoints(d_buffers);

	return true;
}

bool PointCloud::LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi)
{
	PLYFormat ply;
	if (false == ply.Deserialize(filename))
	{
		return false;
	}

	unsigned int numberOfPoints = 0;

	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];

		if (roi.contains(Eigen::Vector3f(x, y, z))) numberOfPoints++;
	}

	Initialize(numberOfPoints);

	unsigned int bufferIndex = 0;
	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];

		if(false == roi.contains(Eigen::Vector3f(x,y,z))) continue;

		auto nx = ply.GetNormals()[i * 3];
		auto ny = ply.GetNormals()[i * 3 + 1];
		auto nz = ply.GetNormals()[i * 3 + 2];

		auto r = ply.GetColors()[i * 3];
		auto g = ply.GetColors()[i * 3 + 1];
		auto b = ply.GetColors()[i * 3 + 2];

		h_buffers.positions[bufferIndex] = Eigen::Vector3f(x, y, z);
		h_buffers.normals[bufferIndex] = Eigen::Vector3f(nx, ny, nz);
		h_buffers.colors[bufferIndex] = Eigen::Vector3b(r * 255.0f, g * 255.0f, b * 255.0f);

		h_buffers.aabb.extend(Eigen::Vector3f(x, y, z));

		bufferIndex++;
	}

	HtoD();

	hashmap.InsertPoints(d_buffers);

	return true;
}

bool PointCloud::SaveToPLY(const std::string& filename)
{
	PLYFormat ply;

	for (size_t i = 0; i < h_buffers.numberOfPoints; i++)
	{
		ply.AddPointFloat3(h_buffers.positions[i].data());
		ply.AddNormalFloat3(h_buffers.normals[i].data());
		ply.AddColor(h_buffers.colors[i].x(), h_buffers.colors[i].y(), h_buffers.colors[i].z());
	}

	ply.Serialize(filename);

	return true;
}

bool PointCloud::LoadFromALP(const std::string& filename)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}

	//printf("min: %f, %f, %f\n", get<0>(alp.GetAABBMin()), get<1>(alp.GetAABBMin()), get<2>(alp.GetAABBMin()));
	//printf("max: %f, %f, %f\n", get<0>(alp.GetAABBMax()), get<1>(alp.GetAABBMax()), get<2>(alp.GetAABBMax()));

	Initialize(alp.GetPoints().size());

	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];

		h_buffers.positions[i] = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
		h_buffers.normals[i] = Eigen::Vector3f(p.normal.x, p.normal.y, p.normal.z);
		h_buffers.colors[i] = Eigen::Vector3b(p.color.x * 255.0f, p.color.y * 255.0f, p.color.z * 255.0f);

		h_buffers.aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
	}

	HtoD();

	hashmap.InsertPoints(d_buffers);

	//hashmap.SerializeToPLY("../../res/test.ply");

	return true;
}

bool PointCloud::LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}

	unsigned int numberOfPoints = 0;

	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];

		if (roi.contains(Eigen::Vector3f(p.position.x, p.position.y, p.position.z))) numberOfPoints++;
	}

	Initialize(numberOfPoints);

	unsigned int bufferIndex = 0;
	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];

		if (false == roi.contains(Eigen::Vector3f(p.position.x, p.position.y, p.position.z))) continue;

		h_buffers.positions[bufferIndex] = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
		h_buffers.normals[bufferIndex] = Eigen::Vector3f(p.normal.x, p.normal.y, p.normal.z);
		h_buffers.colors[bufferIndex] = Eigen::Vector3b(p.color.x * 255.0f, p.color.y * 255.0f, p.color.z * 255.0f);

		h_buffers.aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));

		bufferIndex++;
	}

	HtoD();

	hashmap.InsertPoints(d_buffers);

	//hashmap.SerializeToPLY("../../res/test.ply");

	return true;
}

bool PointCloud::SaveToALP(const std::string& filename)
{
	ALPFormat<PointPNC> alp;
	
	for (size_t i = 0; i < h_buffers.numberOfPoints; i++)
	{
		PointPNC p;
		p.position = make_float3(h_buffers.positions[i].x(), h_buffers.positions[i].y(), h_buffers.positions[i].z());
		p.normal = make_float3(h_buffers.normals[i].x(), h_buffers.normals[i].y(), h_buffers.normals[i].z());
		p.color = make_float3((float)h_buffers.colors[i].x() / 255.0f, (float)h_buffers.colors[i].y() / 255.0f, (float)h_buffers.colors[i].z() / 255.0f);
		
		alp.AddPoint(p);
	}

	alp.Serialize(filename);

	return true;
}

__global__ void Kernel_ComputeVoxelNormalPCA(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
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

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);

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

__global__ void Kernel_ComputeVoxelNormalAverage(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	Eigen::Vector3f normal = Eigen::Vector3f::Zero();
	unsigned int neighborCount = 0;
#pragma unroll
	for (int ni = 0; ni < 124; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_124[ni].x,
			coord.y + neighbor_offsets_124[ni].y,
			coord.z + neighbor_offsets_124[ni].z);

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);

		if (neighborVoxel == nullptr) continue;

		auto neighborNormal = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();

		normal += neighborNormal;
		neighborCount++;
	}

	centerVoxel->normal = normal / (float)neighborCount++;
}

void PointCloud::ComputeVoxelNormalAverage()
{
	nvtxRangePushA("Compute Voxel Normal Average");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeVoxelNormalAverage << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();

	nvtxRangePop();
}

__global__ void Kernel_SerializeVoxels(HashMapInfo info, PointCloudBuffers buffers)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr) return;

	auto position = Eigen::Vector3f(coord.x * info.voxelSize, coord.y * info.voxelSize, coord.z * info.voxelSize);
	auto normal = (voxel->normal / (float)voxel->pointCount).normalized();
	auto color = voxel->color;

	buffers.positions[threadid] = position;
	buffers.normals[threadid] = normal;
	buffers.colors[threadid] = color;
}

void PointCloud::SerializeVoxels(PointCloudBuffers& d_tempBuffers)
{
	nvtxRangePushA("SerializeVoxels");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_SerializeVoxels << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);

	cudaDeviceSynchronize();
}

__global__ void Kernel_SerializeVoxelsColoringByLabel(HashMapInfo info, PointCloudBuffers buffers)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr) return;

	auto position = Eigen::Vector3f(coord.x * info.voxelSize, coord.y * info.voxelSize, coord.z * info.voxelSize);
	auto normal = (voxel->normal / (float)voxel->pointCount).normalized();
	
	float r = hashToFloat(voxel->label * 3 + 0);
	float g = hashToFloat(voxel->label * 3 + 1);
	float b = hashToFloat(voxel->label * 3 + 2);

	auto color = Eigen::Vector3b(r * 255.0f, g * 255.0f, b * 255.0f);

	buffers.positions[threadid] = position;
	buffers.normals[threadid] = normal;
	buffers.colors[threadid] = color;
}

void PointCloud::SerializeVoxelsColoringByLabel(PointCloudBuffers& d_tempBuffers)
{
	nvtxRangePushA("SerializeVoxelsColoringByLabel");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_SerializeVoxelsColoringByLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);

	cudaDeviceSynchronize();
}






__global__ void Kernel_ComputeNeighborCount(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);

		//if (neighborVoxel && neighborVoxel->label != 0)
		//{
		//	centerVoxel->neighborCount++;
		//}

		if (neighborVoxel == nullptr || neighborVoxel->label == 0)
		{
			centerVoxel->neighborCount++;
		}
	}
}

void PointCloud::ComputeNeighborCount()
{
	nvtxRangePushA("NeighborCount");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeNeighborCount << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();
}

__global__ void Kernel_SerializeColoringByNeighborCount(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetVoxelSlot(info, coord);
	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr || voxel->label == 0) return;

	buffers.positions[idx] = voxel->position;
	buffers.normals[idx] = voxel->normal;

	//const Eigen::Vector3b COLORS[26] = {
	//{255, 0, 0},
	//{233, 0, 21},
	//{212, 0, 42},
	//{191, 0, 63},
	//{170, 0, 85},
	//{148, 0, 106},
	//{127, 0, 127},
	//{106, 0, 148},
	//{85, 0, 170},
	//{63, 0, 191},
	//{42, 0, 212},
	//{21, 0, 233},
	//{0, 0, 255},
	//{0, 21, 233},
	//{0, 42, 212},
	//{0, 63, 191},
	//{0, 85, 170},
	//{0, 106, 148},
	//{0, 127, 127},
	//{0, 148, 106},
	//{0, 170, 85},
	//{0, 191, 63},
	//{0, 212, 42},
	//{0, 233, 21},
	//{0, 255, 0},
	//{0, 255, 0}  // 마지막 중복은 종료 강조용 (선택사항)
	//};

	//buffers.colors[idx] = COLORS[25 - voxel->neighborCount];

	if (19 <= voxel->neighborCount && voxel->neighborCount <= 25)
	{
		buffers.colors[idx] = Eigen::Vector3b(255, 0, 0);
	}
	else
	{
		buffers.colors[idx] = Eigen::Vector3b(100, 100, 100);
	}
}

void PointCloud::SerializeColoringByNeighborCount(PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByNeighborCount << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);

	cudaDeviceSynchronize();
}





__global__ void Kernel_ClearNormalDiscontinuity(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	auto voxel = GetVoxel(info, slot);

	voxel->normalDiscontinue = 0;
}

__global__ void Kernel_ComputeNormalDiscontinuity(HashMapInfo info, float normalDiscontinuityThreshold)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	auto centerNormal = (centerVoxel->normal / (float)centerVoxel->pointCount).normalized();

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);

		if (neighborVoxel == nullptr) continue;

		auto neighborNormal = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();

		if (cosf(normalDiscontinuityThreshold * M_PI / 180.0f) < centerNormal.dot(neighborNormal)) continue;

		centerVoxel->normalDiscontinue = 1;
		
		return;
	}
}

void PointCloud::ComputeNormalDiscontinuity(float normalDiscontinuityThreshold)
{
	nvtxRangePushA("NormalDiscontinuity");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_ClearNormalDiscontinuity << <gridOccupied, blockSize >> > (hashmap.info);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_ComputeNormalDiscontinuity << <gridOccupied, blockSize >> > (hashmap.info, normalDiscontinuityThreshold);

		cudaDeviceSynchronize();
	}
}

__global__ void Kernel_SerializeColoringByNormalDiscontinuity(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetVoxelSlot(info, coord);
	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr || voxel->label == 0) return;

	if (0 != voxel->normalDiscontinue)
	{
		buffers.colors[idx] = Eigen::Vector3b(255, 0, 0);
	}
	else
	{
		buffers.colors[idx] = Eigen::Vector3b(100, 100, 100);
	}
}

void PointCloud::SerializeColoringByNormalDiscontinuity(PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByNormalDiscontinuity << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);

	cudaDeviceSynchronize();
}







__global__ void Kernel_ComputeNormalGradient(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

	int count = 0;
	centerVoxel->gradient = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);

		if (neighborVoxel == nullptr || 0 == neighborVoxel->label) continue;

		centerVoxel->gradient +=
			(neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized() -
			(centerVoxel->normal / (float)centerVoxel->pointCount).normalized();

		count++;
	}

	if (0 < count)
	{
		centerVoxel->gradient /= (float)count;
		//printf("%f, %f, %f\n", centerVoxel->gradient.x(), centerVoxel->gradient.y(), centerVoxel->gradient.z());
	}
}

void PointCloud::ComputeNormalGradient()
{
	nvtxRangePushA("ComputeNormalGradient");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
	
	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeNormalGradient << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();
}

__global__ void Kernel_SerializeColoringByNormalGradient(HashMapInfo info, PointCloudBuffers buffers, float threshold)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetVoxelSlot(info, coord);
	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr || voxel->label == 0) return;

	buffers.positions[idx] = voxel->position;
	buffers.normals[idx] = voxel->normal;
	auto g = voxel->gradient.normalized();

	float length = voxel->gradient.norm();
	if (length > threshold)
	{
		buffers.colors[idx] = Eigen::Vector3b(255, 0, 0);
	}
	else
	{
		buffers.colors[idx] = Eigen::Vector3b(100, 100, 100);
	}
}

void PointCloud::SerializeColoringByNormalGradient(float threshold, PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByNormalGradient << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, threshold);

	cudaDeviceSynchronize();
}







__global__ void Kernel_ComputeNormalDivergence(HashMapInfo info)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	auto voxelSlot = GetVoxelSlot(info, coord);
	if (voxelSlot == UINT64_MAX) return;
	auto voxel = GetVoxel(info, voxelSlot);
	if (voxel == nullptr || voxel->label == 0) return;

	const Eigen::Vector3f n0 = voxel->normal.normalized();

	const int3 offsets[6] = {
		{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
	};

	float divergenceSum = 0.0f;
	int validNeighbors = 0;

	for (int ni = 0; ni < 6; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + offsets[ni].x,
			coord.y + offsets[ni].y,
			coord.z + offsets[ni].z);

		auto neighborVoxelSlot = GetVoxelSlot(info, neighborCoord);
		if (neighborVoxelSlot == UINT64_MAX) continue;

		auto neighborVoxel = GetVoxel(info, neighborVoxelSlot);
		if (neighborVoxel == nullptr || neighborVoxel->label == 0) continue;

		Eigen::Vector3f n1 = neighborVoxel->normal.normalized();
		float dot = fminf(fmaxf(n0.dot(n1), -1.0f), 1.0f);
		float angle = acosf(dot); // radians
		divergenceSum += angle;
		++validNeighbors;
	}

	voxel->divergence = (validNeighbors > 0) ? (divergenceSum / validNeighbors) : 0.0f;

	//printf("voxel->divergence : %f\n", voxel->divergence);
}

void PointCloud::ComputeNormalDivergence()
{
	nvtxRangePushA("ComputeNormalDivergence");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
	
	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeNormalDivergence << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();
}

__global__ void Kernel_SerializeColoringByNormalDivergence(HashMapInfo info, PointCloudBuffers buffers, float threshold)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetVoxelSlot(info, coord);
	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr || voxel->label == 0) return;

	buffers.positions[idx] = voxel->position;
	buffers.normals[idx] = voxel->normal;

	//printf("voxel->divergence : %f\n", voxel->divergence);

	if (voxel->divergence > threshold)
	{
		buffers.colors[idx] = Eigen::Vector3b(255, 0, 0);
	}
	else
	{
		buffers.colors[idx] = Eigen::Vector3b(100, 100, 100);
	}
}

void PointCloud::SerializeColoringByNormalDivergence(float threshold, PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByNormalDivergence << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, threshold);

	cudaDeviceSynchronize();
}







__global__ void Kernel_ComputeColorMultiplication(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
	if (centerVoxel == nullptr || centerVoxel->label == 0) return;

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);

		if (neighborVoxel == nullptr || neighborVoxel->label == 0) return;

		float distance = std::sqrt(
			static_cast<float>(neighborVoxel->color.x()) * centerVoxel->color.x() +
			static_cast<float>(neighborVoxel->color.y()) * centerVoxel->color.y() +
			static_cast<float>(neighborVoxel->color.z()) * centerVoxel->color.z());

		//printf("distance : %f\n", distance);

		if (centerVoxel->colorDistance < distance) centerVoxel->colorDistance = distance;
	}
}

void PointCloud::ComputeColorMultiplication()
{
	nvtxRangePushA("ComputeColorMultiplication");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
	
	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_ComputeColorMultiplication << <gridOccupied, blockSize >> > (hashmap.info);

	cudaDeviceSynchronize();
}

__global__ void Kernel_SerializeColoringByColorMultiplication(HashMapInfo info, PointCloudBuffers buffers, float threshold)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t slot = GetVoxelSlot(info, coord);
	HashMapVoxel* voxel = GetVoxel(info, slot);
	if (voxel == nullptr || voxel->label == 0) return;

	buffers.positions[idx] = voxel->position;
	buffers.normals[idx] = voxel->normal;

	if (voxel->colorDistance > threshold)
	{
		buffers.colors[idx] = Eigen::Vector3b(255, 0, 0);
	}
	else
	{
		buffers.colors[idx] = Eigen::Vector3b(100, 100, 100);
	}
}

void PointCloud::SerializeColoringByColorMultiplication(float threshold, PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByColorMultiplication << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers, threshold);

	cudaDeviceSynchronize();
}







__device__ __forceinline__ unsigned int FindRootVoxel(HashMapInfo info, unsigned int idx)
{
	while (info.d_hashTable[idx].label != idx)
	{
		unsigned int parent = info.d_hashTable[idx].label;
		unsigned int grandparent = info.d_hashTable[parent].label;

		auto& voxelA = info.d_hashTable[idx];
		auto& voxelB = info.d_hashTable[parent];

		//// 거리 조건
		//Eigen::Vector3f centerA = voxelA.position / voxelA.pointCount;
		//Eigen::Vector3f centerB = voxelB.position / voxelB.pointCount;
		//float distSq = (centerA - centerB).squaredNorm();
		//if (distSq > kMaxMergeDistance * kMaxMergeDistance) break;

		//// 법선 조건
		//Eigen::Vector3f nA = (voxelA.normal / voxelA.pointCount).normalized();
		//Eigen::Vector3f nB = (voxelB.normal / voxelB.pointCount).normalized();
		//if (nA.dot(nB) < kMinNormalDot) break;

		// 경로 압축
		if (parent != grandparent)
			info.d_hashTable[idx].label = grandparent;

		idx = info.d_hashTable[idx].label;
	}
	return idx;
}

__device__ __forceinline__ void UnionVoxel(HashMapInfo info, unsigned int a, unsigned int b)
{
	unsigned int rootA = FindRootVoxel(info, a);
	unsigned int rootB = FindRootVoxel(info, b);
	if (rootA == rootB) return;

	auto& voxelA = info.d_hashTable[rootA];
	auto& voxelB = info.d_hashTable[rootB];

	//auto dx = abs(voxelA.coord.x - voxelB.coord.x);
	//auto dy = abs(voxelA.coord.y - voxelB.coord.y);
	//auto dz = abs(voxelA.coord.z - voxelB.coord.z);
	//if (dx >= 2 || dy >= 2 || dz >= 2)
	//{
	//	printf("??????????\n");
	//	return;
	//}



	//// 중심 좌표 계산
	//Eigen::Vector3f centerA = voxelA.position / voxelA.pointCount;
	//Eigen::Vector3f centerB = voxelB.position / voxelB.pointCount;
	//float distSq = (centerA - centerB).squaredNorm();

	//if (distSq > kMaxMergeDistance * kMaxMergeDistance) return;

	//// 법선 유사도 판단
	//Eigen::Vector3f nA = (voxelA.normal / voxelA.pointCount).normalized();
	//Eigen::Vector3f nB = (voxelB.normal / voxelB.pointCount).normalized();
	//if (nA.dot(nB) < kMinNormalDot) return;

	// 병합
	if (rootA < rootB)
		atomicMin(&voxelB.label, rootA);
	else
		atomicMin(&voxelA.label, rootB);
}

__global__ void Kernel_ClearLabels(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;
	
	auto voxel = GetVoxel(info, slot);

	voxel->label = slot;
}

__global__ void Kernel_InterVoxelHashMerge26Way(HashMapInfo info, float normalDegreeThreshold)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t slot = GetVoxelSlot(info, coord);
	if (slot == UINT64_MAX) return;

	HashMapVoxel* centerVoxel = GetVoxel(info, slot);
	//if (centerVoxel == nullptr || centerVoxel->label == 0) return;
	if (centerVoxel == nullptr) return;

	auto centerNormal = (centerVoxel->normal / (float)centerVoxel->pointCount).normalized();

#pragma unroll
	for (int ni = 0; ni < 26; ++ni)
	{
		int3 neighborCoord = make_int3(
			coord.x + neighbor_offsets_26[ni].x,
			coord.y + neighbor_offsets_26[ni].y,
			coord.z + neighbor_offsets_26[ni].z);

		size_t neighborSlot = GetVoxelSlot(info, neighborCoord);
		HashMapVoxel* neighborVoxel = GetVoxel(info, neighborSlot);
 
		//if (neighborVoxel && neighborVoxel->label != 0)
		if (neighborVoxel)
		{
			auto neighborNormal = (neighborVoxel->normal / (float)neighborVoxel->pointCount).normalized();

			if (cosf(normalDegreeThreshold * M_PI / 180.0f) > centerNormal.dot(neighborNormal)) continue;

			UnionVoxel(info, slot, neighborSlot);
		}
	}
}

__global__ void Kernel_CompressVoxelHashLabels(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];

	size_t h = voxel_hash(coord, info.capacity);
	for (int i = 0; i < info.maxProbe; ++i)
	{
		size_t probe = (h + i) % info.capacity;
		auto& voxel = info.d_hashTable[probe];

		if (0 == voxel.label) break;

		voxel.label = FindRootVoxel(info, voxel.label);
	}
}

__global__ void Kernel_GetLabels(HashMapInfo info, Eigen::Vector3f* points, unsigned int* labels, unsigned int numberOfPoints)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= numberOfPoints) return;

	auto& p = points[threadid];
	int3 coord = make_int3(
		floorf(p.x() / info.voxelSize),
		floorf(p.y() / info.voxelSize),
		floorf(p.z() / info.voxelSize));

	size_t h = voxel_hash(coord, info.capacity);
	labels[threadid] = UINT_MAX;

	for (int i = 0; i < info.maxProbe; ++i)
	{
		size_t probe = (h + i) % info.capacity;
		if (info.d_hashTable[probe].label == 0) break;

		auto& voxel = info.d_hashTable[probe];

		if (voxel.coord.x == coord.x &&
			voxel.coord.y == coord.y &&
			voxel.coord.z == coord.z)
		{
			labels[threadid] = voxel.label;
			return;
		}
	}
}

vector<unsigned int> PointCloud::Clustering(float normalDegreeThreshold)
{
	nvtxRangePushA("Clustering");

	unsigned int numberOfOccupiedVoxels = hashmap.info.h_numberOfOccupiedVoxels;
	
	vector<unsigned int> labels;

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_ClearLabels << <gridOccupied, blockSize >> > (hashmap.info);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (hashmap.info, normalDegreeThreshold);

		cudaDeviceSynchronize();
	}

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_CompressVoxelHashLabels << <gridOccupied, blockSize >> > (hashmap.info);

		cudaDeviceSynchronize();
	}

	labels.resize(h_buffers.numberOfPoints);
	{
		unsigned int* d_labels = nullptr;
		cudaMalloc(&d_labels, sizeof(unsigned int) * h_buffers.numberOfPoints);

		unsigned int blockSize = 256;
		unsigned int gridOccupied = (h_buffers.numberOfPoints + blockSize - 1) / blockSize;

		Kernel_GetLabels << <gridOccupied, blockSize >> > (hashmap.info, d_buffers.positions, d_labels, h_buffers.numberOfPoints);

		cudaDeviceSynchronize();

		cudaMemcpy(labels.data(), d_labels, sizeof(unsigned int) * h_buffers.numberOfPoints, cudaMemcpyDeviceToHost);

		cudaFree(d_labels);
	}

	nvtxRangePop();

	return labels;
}

__global__ void Kernel_SerializeColoringByLabel(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t h = voxel_hash(coord, info.capacity);

	for (unsigned int i = 0; i < info.maxProbe; ++i)
	{
		size_t slot = (h + i) % info.capacity;
		auto& voxel = info.d_hashTable[slot];

		if (0 == voxel.label) return;

		if (voxel.coord.x == coord.x &&
			voxel.coord.y == coord.y &&
			voxel.coord.z == coord.z)
		{
			buffers.positions[idx] = voxel.position;
			buffers.normals[idx] = voxel.normal;

			float r = hashToFloat(voxel.label * 3 + 0);
			float g = hashToFloat(voxel.label * 3 + 1);
			float b = hashToFloat(voxel.label * 3 + 2);

			buffers.colors[idx] = Eigen::Vector3b(r * 255.0f, g * 255.0f, b * 255.0f);

			return;
		}
	}
}

void PointCloud::SerializeColoringByLabel(PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SerializeColoringByLabel << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);

	cudaDeviceSynchronize();
}






__global__ void Kernel_SplitByNormal(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto& p = buffers.positions[idx];
	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
	size_t h = voxel_hash(coord, info.capacity);

	for (unsigned int i = 0; i < info.maxProbe; ++i)
	{
		size_t slot = (h + i) % info.capacity;
		auto& voxel = info.d_hashTable[slot];

		if (0 == voxel.label) return;

		if (voxel.coord.x == coord.x &&
			voxel.coord.y == coord.y &&
			voxel.coord.z == coord.z)
		{
			buffers.positions[idx] = voxel.position;
			buffers.normals[idx] = voxel.normal;

			float r = hashToFloat(voxel.label * 3 + 0);
			float g = hashToFloat(voxel.label * 3 + 1);
			float b = hashToFloat(voxel.label * 3 + 2);

			buffers.colors[idx] = Eigen::Vector3b(r * 255.0f, g * 255.0f, b * 255.0f);

			return;
		}
	}
}

void PointCloud::SplitByNormal(PointCloudBuffers& d_tempBuffers)
{
	d_buffers.CopyTo(d_tempBuffers);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (d_buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_SplitByNormal << <gridOccupied, blockSize >> > (hashmap.info, d_tempBuffers);

	cudaDeviceSynchronize();
}
