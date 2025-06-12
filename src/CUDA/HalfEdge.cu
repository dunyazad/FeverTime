#include <CUDA/HalfEdge.cuh>
#include <Serialization.hpp>

__host__ __device__
HalfEdgeKey MakeHalfEdgeKey(VertexIndex from, VertexIndex to)
{
	return (static_cast<HalfEdgeKey>(from) << 32) | static_cast<HalfEdgeKey>(to);
}

__host__ __device__
HalfEdgeKey ReverseHalfEdgeKey(HalfEdgeKey key)
{
	VertexIndex from = key >> 32;
	VertexIndex to = static_cast<VertexIndex>(key);
	return MakeHalfEdgeKey(to, from);
}

__host__ __device__ __forceinline__ HalfEdgeHash GetHalfEdgeHash(HalfEdgeKey key, size_t capacity)
{
	key ^= (key >> 33);
	key *= 0xff51afd7ed558ccdULL;
	key ^= (key >> 33);
	key *= 0xc4ceb9fe1a85ec53ULL;
	key ^= (key >> 33);
	return key % capacity;
}

__device__ __forceinline__ bool IsEmptySlot(HalfEdgeKey key) { return key == INVALID_HASHMAP_SLOT; }
__device__ __forceinline__ bool IsDeletedSlot(HalfEdgeKey key) { return key == TOMBSTONE_HASHMAP_SLOT; }
__device__ __forceinline__ bool IsValidSlot(HalfEdgeKey key) { return !IsEmptySlot(key) && !IsDeletedSlot(key); }

__device__ void Kernel_InsertHalfEdge(DeviceHashMap<DeviceHashMapEntry> map, const DeviceHashMapEntry& entry)
{
	HalfEdgeHash hash = GetHalfEdgeHash(entry.key, map.capacity);
	for (int i = 0; i < map.maxProbe; ++i)
	{
		SizeType slot = (hash + i) % map.capacity;
		DeviceHashMapEntry* slotPtr = &map.d_hashTable[slot];

		HalfEdgeKey prevKey = atomicCAS((unsigned long long*) & slotPtr->key, INVALID_HASHMAP_SLOT, entry.key);
		if (prevKey == INVALID_HASHMAP_SLOT || prevKey == entry.key)
		{
			slotPtr->hi = entry.hi;
			return;
		}
	}
}

__global__ void Kernel_InsertHalfEdges(DeviceHashMap<DeviceHashMapEntry> map, const DeviceHashMapEntry* entries, int numberOfEntries)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numberOfEntries) return;
	Kernel_InsertHalfEdge(map, entries[idx]);
}

__device__ bool FindHalfEdge(const DeviceHashMap<DeviceHashMapEntry>& map, uint64_t key, uint32_t& out)
{
	HalfEdgeHash hash = GetHalfEdgeHash(key, map.capacity);
	for (int i = 0; i < map.maxProbe; ++i)
	{
		SizeType slot = (hash + i) % map.capacity;
		const DeviceHashMapEntry& slotEntry = map.d_hashTable[slot];
		if (slotEntry.key == key)
		{
			out = slotEntry.hi;
			return true;
		}
		if (IsEmptySlot(slotEntry.key)) return false;
	}
	return false;
}

__device__ bool DeleteHalfEdge(DeviceHashMap<DeviceHashMapEntry>& map, uint64_t key)
{
	HalfEdgeHash hash = GetHalfEdgeHash(key, map.capacity);
	for (int i = 0; i < map.maxProbe; ++i)
	{
		SizeType slot = (hash + i) % map.capacity;
		DeviceHashMapEntry& slotEntry = map.d_hashTable[slot];
		if (slotEntry.key == key)
		{
			slotEntry.key = TOMBSTONE_HASHMAP_SLOT;
			return true;
		}
		if (IsEmptySlot(slotEntry.key)) return false;
	}
	return false;
}

__global__ void Kernel_BuildHalfEdges(
	HalfEdge* halfEdges,
	Vertex* vertices,
	Face* faces,
	const uint32_t* faceIndices,
	SizeType numberOfFaces,
	DeviceHashMap<DeviceHashMapEntry> hashMap)
{
	SizeType faceIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (faceIdx >= numberOfFaces) return;

	const uint32_t i0 = faceIndices[faceIdx * 3 + 0];
	const uint32_t i1 = faceIndices[faceIdx * 3 + 1];
	const uint32_t i2 = faceIndices[faceIdx * 3 + 2];

	HalfEdgeIndex baseIdx = faceIdx * 3;

	HalfEdge& he0 = halfEdges[baseIdx + 0];
	HalfEdge& he1 = halfEdges[baseIdx + 1];
	HalfEdge& he2 = halfEdges[baseIdx + 2];

	// Setup half-edges
	he0.vi = i0; he0.fi = faceIdx; he0.ni = baseIdx + 1;
	he1.vi = i1; he1.fi = faceIdx; he1.ni = baseIdx + 2;
	he2.vi = i2; he2.fi = faceIdx; he2.ni = baseIdx + 0;

	// Register face to first half-edge
	faces[faceIdx].hi = baseIdx;

	// Register vertices to an incident half-edge
	vertices[i0].pi = i0; vertices[i0].hi = baseIdx + 0;
	vertices[i1].pi = i1; vertices[i1].hi = baseIdx + 1;
	vertices[i2].pi = i2; vertices[i2].hi = baseIdx + 2;

	// Insert each directed edge into the hash map
	HalfEdgeIndex localIdx[3] = { baseIdx + 0, baseIdx + 1, baseIdx + 2 };
	VertexIndex from[3] = { i0, i1, i2 };
	VertexIndex to[3] = { i1, i2, i0 };

	for (int k = 0; k < 3; ++k)
	{
		HalfEdgeKey fwdKey = MakeHalfEdgeKey(from[k], to[k]);
		HalfEdgeKey revKey = MakeHalfEdgeKey(to[k], from[k]);
		HalfEdgeIndex hi = localIdx[k];

		HalfEdgeHash hash = GetHalfEdgeHash(fwdKey, hashMap.capacity);

		for (int probe = 0; probe < hashMap.maxProbe; ++probe)
		{
			SizeType slot = (hash + probe) % hashMap.capacity;
			auto* entry = &hashMap.d_hashTable[slot];
			HalfEdgeKey old = atomicCAS((unsigned long long*) & entry->key, INVALID_HASHMAP_SLOT, fwdKey);

			if (old == INVALID_HASHMAP_SLOT || old == fwdKey)
			{
				entry->hi = hi;
				break;
			}
		}

		// Attempt to find the reverse edge (opposite)
		HalfEdgeHash revHash = GetHalfEdgeHash(revKey, hashMap.capacity);
		for (int probe = 0; probe < hashMap.maxProbe; ++probe)
		{
			SizeType slot = (revHash + probe) % hashMap.capacity;
			auto entry = hashMap.d_hashTable[slot];
			if (entry.key == revKey)
			{
				halfEdges[entry.hi].oi = hi;
				halfEdges[hi].oi = entry.hi;
				break;
			}
		}
	}
}

bool BuildHalfEdgeMeshFromMesh(
	const float* d_points, const float* d_normals, const float* d_colors, uint32_t numberOfPoints,
	const uint32_t* d_faces, uint32_t numberOfFaces,
	HalfEdgeMesh& result)
{
	result.numberOfPoints = numberOfPoints;
	cudaMalloc(&result.d_points, sizeof(float) * 3 * numberOfPoints);
	cudaMemcpy(result.d_points, d_points, sizeof(float) * 3 * numberOfPoints, cudaMemcpyDeviceToDevice);

	result.numberOfFaces = numberOfFaces;
	result.numberOfHalfEdges = numberOfFaces * 3;
	cudaMalloc(&result.d_faces, sizeof(Face) * numberOfFaces);
	cudaMalloc(&result.d_halfEdges, sizeof(HalfEdge) * result.numberOfHalfEdges);

	result.d_hashMap.capacity = result.numberOfHalfEdges * 2;
	cudaMalloc(&result.d_hashMap.d_hashTable, sizeof(DeviceHashMapEntry) * result.d_hashMap.capacity);
	cudaMemset(result.d_hashMap.d_hashTable, 0xFF, sizeof(DeviceHashMapEntry) * result.d_hashMap.capacity);

	result.numberOfVertices = numberOfPoints;
	cudaMalloc(&result.d_vertices, sizeof(Vertex) * numberOfPoints);
	cudaMemset(result.d_vertices, 0, sizeof(Vertex) * numberOfPoints);

	dim3 block(128);
	dim3 grid((numberOfFaces + block.x - 1) / block.x);

	Kernel_BuildHalfEdges << <grid, block >> > (
		result.d_halfEdges,
		result.d_vertices,
		result.d_faces,
		d_faces,
		numberOfFaces,
		result.d_hashMap);

	cudaDeviceSynchronize();

	return true;
}

void SerializeHalfEdgeMesh(const HalfEdgeMesh& mesh, const std::string& outputPath)
{
	// Copy data from device
	std::vector<float> h_points(mesh.numberOfPoints * 3);
	cudaMemcpy(h_points.data(), mesh.d_points, sizeof(float) * h_points.size(), cudaMemcpyDeviceToHost);

	std::vector<HalfEdge> h_halfEdges(mesh.numberOfHalfEdges);
	cudaMemcpy(h_halfEdges.data(), mesh.d_halfEdges, sizeof(HalfEdge) * mesh.numberOfHalfEdges, cudaMemcpyDeviceToHost);

	std::vector<Face> h_faces(mesh.numberOfFaces);
	cudaMemcpy(h_faces.data(), mesh.d_faces, sizeof(Face) * mesh.numberOfFaces, cudaMemcpyDeviceToHost);

	std::vector<Vertex> h_vertices(mesh.numberOfPoints);
	cudaMemcpy(h_vertices.data(), mesh.d_vertices, sizeof(Vertex) * mesh.numberOfPoints, cudaMemcpyDeviceToHost);

	// Output to text file
	std::ofstream file(outputPath);
	if (!file.is_open())
	{
		std::cerr << "Failed to open output file: " << outputPath << std::endl;
		return;
	}

	file << "# HalfEdge Mesh Serialization\n";
	file << "Vertices: " << mesh.numberOfPoints << "\n";
	file << "Faces: " << mesh.numberOfFaces << "\n";
	file << "HalfEdges: " << mesh.numberOfHalfEdges << "\n\n";

	file << "# Vertex positions\n";
	for (uint32_t i = 0; i < mesh.numberOfPoints; ++i)
	{
		file << "v " << h_points[i * 3 + 0] << " "
			<< h_points[i * 3 + 1] << " "
			<< h_points[i * 3 + 2] << "\n";
	}

	file << "\n# Faces (half-edge index)\n";
	for (uint32_t i = 0; i < mesh.numberOfFaces; ++i)
	{
		file << "f_hi " << h_faces[i].hi << "\n";
	}

	file << "\n# HalfEdges\n";
	for (uint32_t i = 0; i < mesh.numberOfHalfEdges; ++i)
	{
		const auto& he = h_halfEdges[i];
		file << "he " << i << " vi: " << he.vi << " fi: " << he.fi
			<< " ni: " << he.ni << " oi: " << he.oi << "\n";
	}

	file << "\n# Vertices (incident half-edge)\n";
	for (uint32_t i = 0; i < mesh.numberOfPoints; ++i)
	{
		file << "vertex " << i << " pi: " << h_vertices[i].pi << " hi: " << h_vertices[i].hi << "\n";
	}

	file.close();
	std::cout << "Serialized mesh to: " << outputPath << std::endl;
}

void TestHalfEdge()
{
	PLYFormat ply;

	ply.Deserialize("D:\\Resources\\3D\\PLY\\Boat.ply");

	bool colorComponent4 = false;
	if (ply.GetColors().size() / 4 == ply.GetPoints().size() / 3)
	{
		colorComponent4 = true;
	}

	for (size_t i = 0; i < ply.GetTriangleIndices().size() / 3; i++)
	{
		auto ti0 = ply.GetTriangleIndices()[i * 3 + 0];
		auto ti1 = ply.GetTriangleIndices()[i * 3 + 1];
		auto ti2 = ply.GetTriangleIndices()[i * 3 + 2];

		auto v0x = ply.GetPoints()[ti0 * 3 + 0];
		auto v0y = ply.GetPoints()[ti0 * 3 + 1];
		auto v0z = ply.GetPoints()[ti0 * 3 + 2];

		auto v1x = ply.GetPoints()[ti1 * 3 + 0];
		auto v1y = ply.GetPoints()[ti1 * 3 + 1];
		auto v1z = ply.GetPoints()[ti1 * 3 + 2];

		auto v2x = ply.GetPoints()[ti2 * 3 + 0];
		auto v2y = ply.GetPoints()[ti2 * 3 + 1];
		auto v2z = ply.GetPoints()[ti2 * 3 + 2];

		auto n0x = ply.GetNormals()[ti0 * 3 + 0];
		auto n0y = ply.GetNormals()[ti0 * 3 + 1];
		auto n0z = ply.GetNormals()[ti0 * 3 + 2];

		auto n1x = ply.GetNormals()[ti1 * 3 + 0];
		auto n1y = ply.GetNormals()[ti1 * 3 + 1];
		auto n1z = ply.GetNormals()[ti1 * 3 + 2];

		auto n2x = ply.GetNormals()[ti2 * 3 + 0];
		auto n2y = ply.GetNormals()[ti2 * 3 + 1];
		auto n2z = ply.GetNormals()[ti2 * 3 + 2];

		if (false == colorComponent4)
		{
			auto c0x = ply.GetColors()[ti0 * 3 + 0];
			auto c0y = ply.GetColors()[ti0 * 3 + 1];
			auto c0z = ply.GetColors()[ti0 * 3 + 2];

			auto c1x = ply.GetColors()[ti1 * 3 + 0];
			auto c1y = ply.GetColors()[ti1 * 3 + 1];
			auto c1z = ply.GetColors()[ti1 * 3 + 2];

			auto c2x = ply.GetColors()[ti2 * 3 + 0];
			auto c2y = ply.GetColors()[ti2 * 3 + 1];
			auto c2z = ply.GetColors()[ti2 * 3 + 2];
		}
		else
		{
			auto c0x = ply.GetColors()[ti0 * 4 + 0];
			auto c0y = ply.GetColors()[ti0 * 4 + 1];
			auto c0z = ply.GetColors()[ti0 * 4 + 2];
			auto c0w = ply.GetColors()[ti0 * 4 + 3];

			auto c1x = ply.GetColors()[ti1 * 4 + 0];
			auto c1y = ply.GetColors()[ti1 * 4 + 1];
			auto c1z = ply.GetColors()[ti1 * 4 + 2];
			auto c1w = ply.GetColors()[ti1 * 4 + 3];

			auto c2x = ply.GetColors()[ti2 * 4 + 0];
			auto c2y = ply.GetColors()[ti2 * 4 + 1];
			auto c2z = ply.GetColors()[ti2 * 4 + 2];
			auto c2w = ply.GetColors()[ti2 * 4 + 3];
		}

		//printf("v0: (%f, %f, %f), v1: (%f, %f, %f), v2: (%f, %f, %f)\n", 
		//	v0x, v0y, v0z, 
		//	v1x, v1y, v1z, 
		//	v2x, v2y, v2z);
	}

	float* d_points = nullptr;
	cudaMalloc(&d_points, sizeof(float) * ply.GetPoints().size());
	cudaMemcpy(d_points, ply.GetPoints().data(), sizeof(float) * ply.GetPoints().size(), cudaMemcpyHostToDevice);

	float* d_normals = nullptr;
	cudaMalloc(&d_normals, sizeof(float) * ply.GetNormals().size());
	cudaMemcpy(d_normals, ply.GetNormals().data(), sizeof(float) * ply.GetNormals().size(), cudaMemcpyHostToDevice);

	float* d_colors = nullptr;
	cudaMalloc(&d_colors, sizeof(float) * ply.GetColors().size());
	cudaMemcpy(d_colors, ply.GetColors().data(), sizeof(float) * ply.GetColors().size(), cudaMemcpyHostToDevice);

	HalfEdgeMesh mesh;

	BuildHalfEdgeMeshFromMesh(
		d_points, d_normals, d_colors, 
		static_cast<uint32_t>(ply.GetPoints().size() / 3), 
		ply.GetTriangleIndices().data(), 
		static_cast<uint32_t>(ply.GetTriangleIndices().size() / 3), 
		mesh);


	float* h_points = new float[ply.GetPoints().size()];
	cudaMemcpy(h_points, d_points, sizeof(float) * ply.GetPoints().size(), cudaMemcpyDeviceToHost);

	cudaDeviceSynchronize();

	for (int i = 0; i < 10; ++i) {
		std::cout << "Debug Point " << i << ": "
			<< h_points[i * 3 + 0] << ", "
			<< h_points[i * 3 + 1] << ", "
			<< h_points[i * 3 + 2] << "\n";
	}

	delete[] h_points;

	//SerializeHalfEdgeMesh(mesh, "D:/Resources/3D/PLY/Boat_halfedge.txt");

	//
	//ply.Serialize("D:\\Resources\\3D\\PLY\\Boat_out.ply");
}
