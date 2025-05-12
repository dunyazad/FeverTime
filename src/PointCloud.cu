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
	h_buffers.Terminate(true);
	d_buffers.Terminate(false);

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

bool PointCloud::LoadFromALP(const std::string& filename)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}

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

__device__ __forceinline__ unsigned int FindRootVoxel(HashMapInfo info, unsigned int idx)
{
	while (info.d_hashTable[idx].label != idx)
	{
		unsigned int parent = info.d_hashTable[idx].label;
		unsigned int grandparent = info.d_hashTable[parent].label;
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

	Eigen::Vector3f nA = voxelA.normal;
	Eigen::Vector3f nB = voxelB.normal;

	//auto cuA = voxelA.divergence;
	//auto cuB = voxelB.divergence;

	const float minDot = 0.425f;
	//const float minDot = 0.1f;
	if (nA.dot(nB) < minDot) return;

	//if (fabsf(cuA - cuB) > 0.02f) return;

	//// 조건 2: 주변 smoothness 조건
	//const float maxDivergence = 0.2f; // radians (~17도)
	//if (voxelA.divergence > maxDivergence || voxelB.divergence > maxDivergence)
	//    return;

	//// (선택) 조건 3: divergence 불일치 방지
	//if (fabsf(voxelA.divergence - voxelB.divergence) > 0.15f)
	//    return;

	// 병합: label을 더 작은 쪽으로
	if (rootA < rootB)
	{
		atomicMin(&voxelB.label, rootA);
	}
	else
	{
		atomicMin(&voxelA.label, rootB);
	}
}

__global__ void Kernel_InterVoxelHashMerge26Way(HashMapInfo info)
{
	unsigned int threadid = blockDim.x * blockIdx.x + threadIdx.x;
	if (threadid >= *info.d_numberOfOccupiedVoxels) return;

	int3 coord = info.d_occupiedVoxelIndices[threadid];
	size_t h = voxel_hash(coord, info.capacity);

	// Linear probing to find the voxel in the hash table
	for (int i = 0; i < info.maxProbe; ++i)
	{
		size_t probe = (h + i) % info.capacity;
		auto& entry = info.d_hashTable[probe];

		if (entry.coord.x == coord.x &&
			entry.coord.y == coord.y &&
			entry.coord.z == coord.z)
		{
			if (entry.label == 0) return; // This should never happen unless corrupted

			const int3 offsets[26] =
			{
				{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
				{1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
				{1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
				{0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
				{1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
				{-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
			};

#pragma unroll
			for (int ni = 0; ni < 26; ++ni)
			{
				int3 neighborCoord = make_int3(
					coord.x + offsets[ni].x,
					coord.y + offsets[ni].y,
					coord.z + offsets[ni].z);

				size_t nh = voxel_hash(neighborCoord, info.capacity);

				for (int j = 0; j < info.maxProbe; ++j)
				{
					size_t nprobe = (nh + j) % info.capacity;
					auto& neighbor = info.d_hashTable[nprobe];

					if (neighbor.label == 0)
						break;  // Stop probing if slot is empty (assuming no tombstones)

					if (neighbor.coord.x == neighborCoord.x &&
						neighbor.coord.y == neighborCoord.y &&
						neighbor.coord.z == neighborCoord.z)
					{
						UnionVoxel(info, probe, nprobe);
						break;
					}
				}
			}

			break;  // Stop probing once the original voxel is found
		}

		// Optional: break early if we hit an empty slot (assumes no deletion holes)
		if (entry.label == 0)
			break;
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

vector<unsigned int> PointCloud::Clustering()
{
	unsigned int numberOfOccupiedVoxels = 0;
	cudaMemcpy(&numberOfOccupiedVoxels, hashmap.info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

	vector<unsigned int> labels;

	{
		unsigned int blockSize = 256;
		unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;

		Kernel_InterVoxelHashMerge26Way << <gridOccupied, blockSize >> > (hashmap.info);

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
