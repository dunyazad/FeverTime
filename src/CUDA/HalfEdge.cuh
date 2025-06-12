#pragma once

#include <CUDA/cudaCommon.cuh>

#define INVALID_HASHMAP_SLOT UINT64_MAX
#define TOMBSTONE_HASHMAP_SLOT (UINT64_MAX - 1)

using SizeType = uint32_t;
using HalfEdgeHash = uint64_t;
using HalfEdgeKey = uint64_t;
using HalfEdgeSlot = uint64_t;

using PointIndex = uint32_t;
using VertexIndex = uint32_t;
using HalfEdgeIndex = uint32_t;
using FaceIndex = uint32_t;

template <typename T>
struct DeviceHashMap
{
    SizeType capacity = 5000000;
    SizeType maxProbe = 32;
    T* d_hashTable = nullptr;
};

struct DeviceHashMapEntry
{
    HalfEdgeKey key = INVALID_HASHMAP_SLOT;
    HalfEdgeIndex hi = 0;
};

struct Vertex
{
    PointIndex pi = 0;
    HalfEdgeIndex hi = 0;
};

struct Face
{
    HalfEdgeIndex hi = 0;
};

struct HalfEdge
{
    VertexIndex vi = 0;
    FaceIndex fi = 0;
    HalfEdgeIndex ni = 0; // Next halfedge
    HalfEdgeIndex oi = INVALID_HASHMAP_SLOT; // Opposite halfedge (default invalid)
};

struct HalfEdgeMesh
{
    SizeType numberOfPoints = 0;
    float* d_points = nullptr;

    SizeType numberOfNormals = 0;
    float* d_normals = nullptr;

    SizeType numberOfColors = 0;
    uint8_t* d_colors = nullptr;

    SizeType numberOfVertices = 0;
    Vertex* d_vertices = nullptr;

    SizeType numberOfHalfEdges = 0;
    HalfEdge* d_halfEdges = nullptr;

    SizeType numberOfFaces = 0;
    Face* d_faces = nullptr;

    DeviceHashMap<DeviceHashMapEntry> d_hashMap;
};

__host__ __device__ __forceinline__ HalfEdgeHash GetHalfEdgeHash(HalfEdgeKey key, uint64_t capacity);

bool BuildHalfEdgeMeshFromMesh(
    const float* d_points, SizeType numberOfPoints,
    const uint32_t* d_faces, SizeType numberOfFaces,
    HalfEdgeMesh& result);

void TestHalfEdge();
