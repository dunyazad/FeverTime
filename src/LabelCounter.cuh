#include <cudaCommon.h>

struct LabelCounter
{
	const unsigned int EMPTY_LABEL = 0xFFFFFFFF;
	const unsigned int MAX_PROBE = 64;
	const unsigned int MAX_OCCUPIED_LABELS = 4096;

	unsigned int* labels = nullptr;
	unsigned int* counts = nullptr;
	unsigned int* rayHitCounts = nullptr;
	unsigned char* labelStates = nullptr; // 0: Invalid, 1: Normal, 2: PatchConnected, 3: ReservedToDelete
	unsigned int* occupiedLabels = nullptr;
	unsigned char* occupiedLabelStates = nullptr;
	unsigned int* occupiedLabelCounts = nullptr;
	unsigned int* occupiedLabelRayHitCounts = nullptr;
	unsigned int* numberOfOccupiedLabels = nullptr;
	unsigned int capacity = 0;

	void Initialize(unsigned int capacity);
	void Terminate();
	void Resize(unsigned int newCapacity);
	void Clear();

	__device__ __forceinline__ unsigned int Hash_uint(unsigned int key);

	__device__ bool Insert(unsigned int key, unsigned int value);

	__device__ bool Find(unsigned int key, unsigned int& outValue);

	__device__ void IncreaseCount(unsigned int label);

	__device__ void IncreaseRayHitCount(unsigned int label);

	__device__ unsigned int GetCount(unsigned int label);

	__device__ unsigned int GetRayHitCount(unsigned int label);

	__device__ unsigned char GetLabelState(unsigned int label);

	__device__ bool IsPatchConnected(unsigned int label);
	__device__ bool SetPatchConnected(unsigned int label);

	__device__ bool IsReservedToDelete(unsigned int label);
	__device__ bool SetReservedToDelete(unsigned int label);
};
