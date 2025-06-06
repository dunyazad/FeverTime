#include <CUDA/HalfEdge.cuh>
#include <Serialization.hpp>

__host__ __device__
uint64_t MakeHalfEdgeKey(uint32_t from, uint32_t to)
{
	return (static_cast<uint64_t>(from) << 32) | static_cast<uint64_t>(to);
}

__host__ __device__
uint64_t ReverseHalfEdgeKey(uint64_t key)
{
	uint32_t from = key >> 32;
	uint32_t to = static_cast<uint32_t>(key);
	return MakeHalfEdgeKey(to, from);
}

__device__ __forceinline__ size_t HalfEdgeHashKey(uint64_t key, size_t capacity)
{
	key ^= (key >> 33);
	key *= 0xff51afd7ed558ccdULL;
	key ^= (key >> 33);
	key *= 0xc4ceb9fe1a85ec53ULL;
	key ^= (key >> 33);
	return key % capacity;
}

__global__ void Kernel_InsertHalfEdges(DeviceHashMap<DeviceHashMapEntry> map, const DeviceHashMapEntry* items, int numItems)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numItems) return;

	DeviceHashMapEntry item = items[idx];
	size_t hash = HalfEdgeHashKey(item.key, map.capacity);

	for (uint32_t i = 0; i < map.maxProbe; ++i)
	{
		size_t slot = (hash + i) % map.capacity;

		uint64_t old = atomicCAS(
			reinterpret_cast<unsigned long long*>(&map.d_hashTable[slot].key),
			INVALID_HASHMAP_SLOT, item.key);
		if (old == INVALID_HASHMAP_SLOT || old == item.key)
		{
			map.d_hashTable[slot] = item;
			return;
		}
	}
}

__device__ void Kernel_InsertHalfEdge(DeviceHashMap<DeviceHashMapEntry> map, const DeviceHashMapEntry& item)
{
	size_t hash = HalfEdgeHashKey(item.key, map.capacity);

	for (uint32_t i = 0; i < map.maxProbe; ++i)
	{
		size_t slot = (hash + i) % map.capacity;

		uint64_t old = atomicCAS(
			reinterpret_cast<unsigned long long*>(&map.d_hashTable[slot].key),
			INVALID_HASHMAP_SLOT, item.key);
		if (old == INVALID_HASHMAP_SLOT || old == item.key)
		{
			map.d_hashTable[slot] = item;
			return;
		}
	}
}

__device__ bool FindHalfEdge(const DeviceHashMap<DeviceHashMapEntry>& map, uint64_t key, uint32_t& out)
{
	size_t hash = HalfEdgeHashKey(key, map.capacity);
	for (uint32_t i = 0; i < map.maxProbe; ++i)
	{
		size_t slot = (hash + i) % map.capacity;
		uint64_t slotKey = map.d_hashTable[slot].key;

		if (slotKey == key)
		{
			out = map.d_hashTable[slot].hi;
			return true;
		}
		if (slotKey == INVALID_HASHMAP_SLOT)
			return false;
	}
	return false;
}

__device__ bool DeleteHalfEdge(DeviceHashMap<DeviceHashMapEntry>& map, uint64_t key)
{
	size_t hash = HalfEdgeHashKey(key, map.capacity);
	for (unsigned int i = 0; i < map.maxProbe; ++i)
	{
		size_t slot = (hash + i) % map.capacity;
		uint64_t slotKey = map.d_hashTable[slot].key;

		if (slotKey == key)
		{
			map.d_hashTable[slot].key = INVALID_HASHMAP_SLOT;
			return true;
		}
		if (slotKey == INVALID_HASHMAP_SLOT)
			return false;
	}
	return false;
}

void TestHalfEdge()
{
	PLYFormat ply;

	ply.Deserialize("D:\\Resources\\3D\\PLY\\Boat.ply");

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

		auto n0 = ply.GetNormals()[ti0];
		auto n1 = ply.GetNormals()[ti1];
		auto n2 = ply.GetNormals()[ti2];

		auto c0 = ply.GetColors()[ti0];
		auto c1 = ply.GetColors()[ti1];
		auto c2 = ply.GetColors()[ti2];

		//printf("0: (%f, %f, %f), v1: (%f, %f, %f), v2: (%f, %f, %f)\n", 
		//	v0x, v0y, v0z, 
		//	v1x, v1y, v1z, 
		//	v2x, v2y, v2z);
	}

	ply.Serialize("D:\\Resources\\3D\\PLY\\Boat_out.ply");
}