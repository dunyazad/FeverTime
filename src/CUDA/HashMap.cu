#include <CUDA/HashMap.cuh>

void HashMap::Initialize()
{
	cudaMalloc(&info.d_hashTable, sizeof(HashMapVoxel) * info.capacity);
	cudaMemset(info.d_hashTable, 0, sizeof(HashMapVoxel) * info.capacity);

	cudaMalloc(&info.d_numberOfOccupiedVoxels, sizeof(unsigned int));
	cudaMemset(info.d_numberOfOccupiedVoxels, 0, sizeof(unsigned int));

	cudaMalloc(&info.d_occupiedVoxelIndices, sizeof(int3) * info.capacity);
	cudaMemset(info.d_occupiedVoxelIndices, 0, sizeof(int3) * info.capacity);

	info.labelCounter.Initialize(info.capacity);
	info.subLabelCounter.Initialize(info.capacity);
}

void HashMap::Terminate()
{
	cudaFree(info.d_hashTable);
	cudaFree(info.d_numberOfOccupiedVoxels);
	cudaFree(info.d_occupiedVoxelIndices);

	info.labelCounter.Terminate();
	info.subLabelCounter.Terminate();
}

__global__ void Kernel_InsertPoints(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= buffers.numberOfPoints) return;

	auto p = buffers.positions[idx];
	auto n = buffers.normals[idx].normalized();
	auto c = buffers.colors[idx];

	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));

	size_t h = voxel_hash(coord, info.capacity);
	for (int i = 0; i < info.maxProbe; ++i) {
		size_t slot = (h + i) % info.capacity;
		int prev = atomicCAS(&(info.d_hashTable[slot].label), 0, slot);

		if (prev == 0) {
			// 새로운 슬롯에 삽입
			info.d_hashTable[slot].coord = coord;
			info.d_hashTable[slot].position = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);
			info.d_hashTable[slot].normal = n;
			info.d_hashTable[slot].color = c;
			info.d_hashTable[slot].pointCount = 1;

			auto oldIndex = atomicAdd(info.d_numberOfOccupiedVoxels, 1);
			info.d_occupiedVoxelIndices[oldIndex] = coord;
			return;
		}
		else {
			int3 existing = info.d_hashTable[slot].coord;
			if (existing.x == coord.x && existing.y == coord.y && existing.z == coord.z) {
				info.d_hashTable[slot].normal += n;
				info.d_hashTable[slot].color = c;
				info.d_hashTable[slot].pointCount++;
				return;
			}
		}
	}
}

void HashMap::InsertPoints(PointCloudBuffers buffers)
{
	unsigned int blockSize = 256;
	unsigned int gridOccupied = (buffers.numberOfPoints + blockSize - 1) / blockSize;

	Kernel_InsertPoints << <gridOccupied, blockSize >> > (info, buffers);

	cudaDeviceSynchronize();

	cudaMemcpy(&info.h_numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);
}

__global__ void Kernel_CountLabels(HashMapInfo info)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[idx];
	auto voxelSlot = GetHashMapVoxelSlot(info, coord);
	if (UINT64_MAX == voxelSlot) return;

	auto voxel = GetHashMapVoxel(info, voxelSlot);
	if (nullptr == voxel) return;

	info.labelCounter.IncreaseCount(voxel->label);
	info.subLabelCounter.IncreaseCount(voxel->subLabel);
}

void HashMap::CountLabels()
{
	info.labelCounter.Clear();
	info.subLabelCounter.Clear();

	//info.labelCounter.Resize(info.capacity * 2);
	//info.subLabelCounter.Resize(info.capacity * 2);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (info.h_numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_CountLabels << <gridOccupied, blockSize >> > (info);

	cudaDeviceSynchronize();
}

__global__ void Kernel_Serialize_HashMap(HashMapInfo info, PointCloudBuffers buffers)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[idx];
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
			buffers.normals[idx] = (voxel.normal / (float)voxel.pointCount).normalized();
			buffers.colors[idx] = voxel.color / voxel.pointCount;
			return;
		}
	}
}

void HashMap::SerializeToPLY(const std::string& filename)
{
	PLYFormat ply;

	unsigned int numberOfOccupiedVoxels = info.h_numberOfOccupiedVoxels;

	PointCloudBuffers d_buffers;
	d_buffers.Initialize(numberOfOccupiedVoxels, false);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_Serialize_HashMap << <gridOccupied, blockSize >> > (info, d_buffers);

	cudaDeviceSynchronize();

	PointCloudBuffers h_buffers;
	h_buffers.Initialize(numberOfOccupiedVoxels, true);

	d_buffers.CopyTo(h_buffers);

	for (size_t i = 0; i < numberOfOccupiedVoxels; i++)
	{
		auto& p = h_buffers.positions[i];
		auto& n = h_buffers.normals[i];
		auto& c = h_buffers.colors[i];

		ply.AddPoint(p.x(), p.y(), p.z());
		ply.AddNormal(n.x(), n.y(), n.z());
		ply.AddColor(
			fminf(1.0f, fmaxf(0.0f, c.x())) / 255.0f,
			fminf(1.0f, fmaxf(0.0f, c.y())) / 255.0f,
			fminf(1.0f, fmaxf(0.0f, c.z())) / 255.0f
		);
	}

	ply.Serialize(filename);

	d_buffers.Terminate();
	h_buffers.Terminate();
}

__device__ size_t GetHashMapVoxelSlot(HashMapInfo& info, int3 coord)
{
	size_t h = voxel_hash(coord, info.capacity);
	for (int i = 0; i < info.maxProbe; ++i) {
		size_t slot = (h + i) % info.capacity;

		if (1 == info.d_hashTable[slot].deleted) continue;

		if (info.d_hashTable[slot].coord.x == coord.x &&
			info.d_hashTable[slot].coord.y == coord.y &&
			info.d_hashTable[slot].coord.z == coord.z)
		{
			return slot;
		}
	}

	return UINT64_MAX;
}

__device__ HashMapVoxel* GetHashMapVoxel(HashMapInfo& info, size_t slot)
{
	if (UINT64_MAX == slot) return nullptr;
	return &(info.d_hashTable[slot]);
}

__device__ size_t InsertHashMapVoxel(HashMapInfo& info, int3 coord)
{
	size_t h = voxel_hash(coord, info.capacity);

	for (int i = 0; i < info.maxProbe; ++i)
	{
		size_t slot = (h + i) % info.capacity;
		HashMapVoxel* voxel = &info.d_hashTable[slot];

		// 빈 슬롯이면 원자적으로 점유 시도
		int prev = atomicCAS(&(voxel->label), 0, slot);
		if (prev == 0)
		{
			// 점유 성공, 초기화 수행
			voxel->coord = coord;
			voxel->position = Eigen::Vector3f(coord.x * info.voxelSize, coord.y * info.voxelSize, coord.z * info.voxelSize);
			voxel->pointCount = 0;
			voxel->neighborCount = 0;
			voxel->emptyNeighborCount = 0;
			voxel->normal = Eigen::Vector3f::Zero();
			voxel->gradient = Eigen::Vector3f::Zero();
			voxel->divergence = 0.0f;
			voxel->colorDistance = 0.0f;
			voxel->normalDiscontinue = 0;
			voxel->color = Eigen::Vector4b(255, 255, 255, 255);
			voxel->deleted = 0;

			unsigned int index = atomicAdd(info.d_numberOfOccupiedVoxels, 1);
			info.d_occupiedVoxelIndices[index] = coord;

			return slot;
		}
		else
		{
			// 이미 존재하는 경우 같은 좌표인지 확인
			if (voxel->coord.x == coord.x &&
				voxel->coord.y == coord.y &&
				voxel->coord.z == coord.z)
			{
				return slot;
			}
		}
	}

	// 삽입 실패
	return UINT64_MAX;
}

__device__ bool DeleteHashMapVoxel(HashMapInfo& info, int3 coord)
{
	size_t h = voxel_hash(coord, info.capacity);
	for (int i = 0; i < info.maxProbe; ++i)
	{
		size_t slot = (h + i) % info.capacity;
		HashMapVoxel* voxel = &info.d_hashTable[slot];

		// 좌표 일치 시 삭제 처리
		if (voxel->coord.x == coord.x &&
			voxel->coord.y == coord.y &&
			voxel->coord.z == coord.z &&
			voxel->deleted == 0)
		{
			voxel->deleted = 1;
			voxel->label = 0;  // 필요에 따라 reset
			voxel->subLabel = 0;
			voxel->reservedToDeleted = 0;
			return true;
		}
	}

	return false;
}
