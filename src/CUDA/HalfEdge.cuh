#pragma once

#include <CUDA/cudaCommon.cuh>

#define INVALID_HASHMAP_SLOT UINT64_MAX

template <typename T>
struct DeviceHashMap
{
    size_t capacity = 5000000;
    uint32_t maxProbe = 32;
    T* d_hashTable = nullptr;
};

struct DeviceHashMapEntry {
	uint64_t key;
	uint32_t hi; // Halfedge index
};

struct Vertex
{
	uint32_t pi; // Point index
	uint32_t hi; // Halfedge index
};

struct Face
{
	uint32_t hi; // Halfedge index
};

struct HalfEdge
{
	uint32_t vi; // Vertex index
	uint32_t fi; // Face index
	uint32_t ni; // Next halfedge index
	uint32_t oi; // Opposite halfedge index
};

void TestHalfEdge();