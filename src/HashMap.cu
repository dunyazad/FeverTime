#include <HashMap.cuh>
#include <Serialization.hpp>

void PointCloudBuffers::Initialize(unsigned int numberOfPoints, bool isHostBuffer)
{
	this->isHostBuffer = isHostBuffer;
	this->numberOfPoints = numberOfPoints;

	if (isHostBuffer)
	{
		positions = new Eigen::Vector3f[numberOfPoints];
		normals = new Eigen::Vector3f[numberOfPoints];
		colors = new Eigen::Vector3b[numberOfPoints];
	}
	else
	{
		cudaMalloc(&positions, sizeof(Eigen::Vector3f) * numberOfPoints);
		cudaMalloc(&normals, sizeof(Eigen::Vector3f) * numberOfPoints);
		cudaMalloc(&colors, sizeof(Eigen::Vector3b) * numberOfPoints);
	}
}

void PointCloudBuffers::Terminate()
{
	if (isHostBuffer)
	{
		if (0 < numberOfPoints)
		{
			delete[] positions;
			delete[] normals;
			delete[] colors;
		}
	}
	else
	{
		if (0 < numberOfPoints)
		{
			cudaFree(positions);
			cudaFree(normals);
			cudaFree(colors);
		}
	}
}

void PointCloudBuffers::CopyTo(PointCloudBuffers& other)
{
	if (isHostBuffer && other.isHostBuffer)
	{
		other.numberOfPoints = numberOfPoints;
		other.aabb = aabb;
		memcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints);
		memcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints);
		memcpy(other.colors, colors, sizeof(Eigen::Vector3b) * numberOfPoints);
	}
	else if (false == isHostBuffer && other.isHostBuffer)
	{
		other.numberOfPoints = numberOfPoints;
		other.aabb = aabb;
		cudaMemcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToHost);
		cudaMemcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToHost);
		cudaMemcpy(other.colors, colors, sizeof(Eigen::Vector3b) * numberOfPoints, cudaMemcpyDeviceToHost);
	}
	else if (isHostBuffer && false == other.isHostBuffer)
	{
		other.numberOfPoints = numberOfPoints;
		other.aabb = aabb;
		cudaMemcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);
		cudaMemcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);
		cudaMemcpy(other.colors, colors, sizeof(Eigen::Vector3b) * numberOfPoints, cudaMemcpyHostToDevice);
	}
	else if (false == isHostBuffer && false == other.isHostBuffer)
	{
		other.numberOfPoints = numberOfPoints;
		other.aabb = aabb;
		cudaMemcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToDevice);
		cudaMemcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToDevice);
		cudaMemcpy(other.colors, colors, sizeof(Eigen::Vector3b) * numberOfPoints, cudaMemcpyDeviceToDevice);
	}
}

void HashMap::Initialize()
{
	cudaMalloc(&info.d_hashTable, sizeof(HashMapVoxel) * info.capacity);
	cudaMemset(info.d_hashTable, 0, sizeof(HashMapVoxel) * info.capacity);

	cudaMalloc(&info.d_numberOfOccupiedVoxels, sizeof(unsigned int));
	cudaMemset(info.d_numberOfOccupiedVoxels, 0, sizeof(unsigned int));

	cudaMalloc(&info.d_occupiedVoxelIndices, sizeof(int3) * info.capacity);
	cudaMemset(info.d_occupiedVoxelIndices, 0, sizeof(int3) * info.capacity);
}

void HashMap::Terminate()
{
	cudaFree(info.d_hashTable);
	cudaFree(info.d_numberOfOccupiedVoxels);
	cudaFree(info.d_occupiedVoxelIndices);
}

__device__ __host__ float hashToFloat(uint32_t seed)
{
    seed ^= seed >> 13;
    seed *= 0x5bd1e995;
    seed ^= seed >> 15;
    return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
};

__device__ __host__ size_t voxel_hash(int3 coord, size_t tableSize)
{
    return ((size_t)(coord.x * 73856093) ^ (coord.y * 19349663) ^ (coord.z * 83492791)) % tableSize;
}

__device__ size_t GetVoxelSlot(HashMapInfo& info, int3 coord)
{
	size_t h = voxel_hash(coord, info.capacity);
	for (int i = 0; i < info.maxProbe; ++i) {
		size_t slot = (h + i) % info.capacity;

		if (info.d_hashTable[slot].coord.x == coord.x &&
			info.d_hashTable[slot].coord.y == coord.y &&
			info.d_hashTable[slot].coord.z == coord.z)
		{
			return slot;
		}
	}

	return UINT64_MAX;
}

__device__ HashMapVoxel* GetVoxel(HashMapInfo& info, size_t slot)
{
	if (UINT64_MAX == slot) return nullptr;
	return &(info.d_hashTable[slot]);
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
			// »õ·Î¿î ½½·Ô¿¡ »ðÀÔ
			info.d_hashTable[slot].coord = coord;
			info.d_hashTable[slot].position = Eigen::Vector3f((float)coord.x * info.voxelSize, (float)coord.y * info.voxelSize, (float)coord.z * info.voxelSize);
			info.d_hashTable[slot].normal = Eigen::Vector3f(n.x(), n.y(), n.z());
			info.d_hashTable[slot].color = Eigen::Vector3b(c.x(), c.y(), c.z());
			info.d_hashTable[slot].pointCount = 1;

			auto oldIndex = atomicAdd(info.d_numberOfOccupiedVoxels, 1);
			info.d_occupiedVoxelIndices[oldIndex] = coord;
			return;
		}
		else {
			int3 existing = info.d_hashTable[slot].coord;
			if (existing.x == coord.x && existing.y == coord.y && existing.z == coord.z) {
				info.d_hashTable[slot].normal += Eigen::Vector3f(n.x(), n.y(), n.z());
				info.d_hashTable[slot].color += Eigen::Vector3b(c.x(), c.y(), c.z());
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

	unsigned int numberOfOccupiedVoxels = 0;
	cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);
}

__global__ void Kernel_Serialize(HashMapInfo info, PointCloudBuffers buffers)
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

	unsigned int numberOfOccupiedVoxels = 0;
	cudaMemcpy(&numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

	PointCloudBuffers d_buffers;
	d_buffers.Initialize(numberOfOccupiedVoxels, false);

	unsigned int blockSize = 256;
	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

	Kernel_Serialize << <gridOccupied, blockSize >> > (info, d_buffers);

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
