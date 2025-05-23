#include <CUDA/LabelCounter.cuh>

void LabelCounter::Initialize(unsigned int capacity)
{
	this->capacity = capacity;
	cudaMalloc(&labels, sizeof(unsigned int) * capacity);
	cudaMalloc(&counts, sizeof(unsigned int) * capacity);
	cudaMalloc(&rayHitCounts, sizeof(unsigned int) * capacity);
	cudaMalloc(&labelStates, sizeof(unsigned char) * capacity);

	cudaMalloc(&occupiedLabels, sizeof(unsigned int) * MAX_OCCUPIED_LABELS);
	cudaMalloc(&occupiedLabelStates, sizeof(unsigned char) * MAX_OCCUPIED_LABELS);
	cudaMalloc(&occupiedLabelCounts, sizeof(unsigned int) * MAX_OCCUPIED_LABELS);
	cudaMalloc(&occupiedLabelRayHitCounts, sizeof(unsigned int) * MAX_OCCUPIED_LABELS);
	cudaMalloc(&numberOfOccupiedLabels, sizeof(unsigned int));
}

void LabelCounter::Terminate()
{
	if (labels) cudaFree(labels);
	if (counts) cudaFree(counts);
	if (rayHitCounts) cudaFree(rayHitCounts);
	if (labelStates) cudaFree(labelStates);

	if (occupiedLabels) cudaFree(occupiedLabels);
	if (occupiedLabelStates) cudaFree(occupiedLabelStates);
	if (occupiedLabelCounts) cudaFree(occupiedLabelCounts);
	if (occupiedLabelRayHitCounts) cudaFree(occupiedLabelRayHitCounts);
	if (numberOfOccupiedLabels) cudaFree(numberOfOccupiedLabels);
}

void LabelCounter::Resize(unsigned int newCapacity)
{
	if (capacity < newCapacity)
	{
		printf("LabelCounter::Resize %d to %d\n", capacity, newCapacity * 4);

		capacity = newCapacity * 4;

		//if (nullptr == stream)
		//{
			if (labels) cudaFree(labels);
			if (counts) cudaFree(counts);
			if (rayHitCounts) cudaFree(rayHitCounts);
			if (labelStates) cudaFree(labelStates);

			cudaMalloc(&labels, sizeof(unsigned int) * capacity);
			cudaMalloc(&counts, sizeof(unsigned int) * capacity);
			cudaMalloc(&rayHitCounts, sizeof(unsigned int) * capacity);
			cudaMalloc(&labelStates, sizeof(unsigned char) * capacity);
		//}
		//else
		//{
		//	if (labels) cudaFreeAsync(labels, stream);
		//	if (counts) cudaFreeAsync(counts, stream);
		//	if (rayHitCounts) cudaFreeAsync(rayHitCounts, stream);
		//	if (labelStates) cudaFreeAsync(labelStates, stream);

		//	//checkCudaErrors(cudaStreamSynchronize(stream));

		//	cudaMallocAsync(&labels, sizeof(unsigned int) * capacity, stream);
		//	cudaMallocAsync(&counts, sizeof(unsigned int) * capacity, stream);
		//	cudaMallocAsync(&rayHitCounts, sizeof(unsigned int) * capacity, stream);
		//	cudaMallocAsync(&labelStates, sizeof(unsigned char) * capacity, stream);
		//}

		//checkCudaErrors(cudaStreamSynchronize(stream));

		Clear();
	}
}

void LabelCounter::Clear()
{
	//if (nullptr == stream)
	//{
		if (labels)
			cudaMemset(labels, 0xFF, sizeof(unsigned int) * capacity);
		if (counts)
			cudaMemset(counts, 0, sizeof(unsigned int) * capacity);
		if (labelStates)
			cudaMemset(labelStates, 0, sizeof(unsigned char) * capacity);

		if (occupiedLabels)
			cudaMemset(occupiedLabels, 0, sizeof(unsigned int) * MAX_OCCUPIED_LABELS);
		if (occupiedLabelCounts)
			cudaMemset(occupiedLabelCounts, 0, sizeof(unsigned int) * MAX_OCCUPIED_LABELS);
		if (occupiedLabelRayHitCounts)
			cudaMemset(occupiedLabelRayHitCounts, 0, sizeof(unsigned int) * MAX_OCCUPIED_LABELS);
		if (occupiedLabelStates)
			cudaMemset(occupiedLabelStates, 0, sizeof(unsigned char) * MAX_OCCUPIED_LABELS);
		if (numberOfOccupiedLabels)
			cudaMemset(numberOfOccupiedLabels, 0, sizeof(unsigned int));
	//}
	//else
	//{
	//	if (labels)
	//		cudaMemsetAsync(labels, 0xFF, sizeof(unsigned int) * capacity, stream);
	//	if (counts)
	//		cudaMemsetAsync(counts, 0, sizeof(unsigned int) * capacity, stream);
	//	if (labelStates)
	//		cudaMemsetAsync(labelStates, 0, sizeof(unsigned char) * capacity, stream);

	//	if (occupiedLabels)
	//		cudaMemsetAsync(occupiedLabels, 0, sizeof(unsigned int) * MAX_OCCUPIED_LABELS, stream);
	//	if (occupiedLabelCounts)
	//		cudaMemsetAsync(occupiedLabelCounts, 0, sizeof(unsigned int) * MAX_OCCUPIED_LABELS, stream);
	//	if (occupiedLabelRayHitCounts)
	//		cudaMemsetAsync(occupiedLabelRayHitCounts, 0, sizeof(unsigned int) * MAX_OCCUPIED_LABELS, stream);
	//	if (occupiedLabelStates)
	//		cudaMemsetAsync(occupiedLabelStates, 0, sizeof(unsigned char) * MAX_OCCUPIED_LABELS, stream);
	//	if (numberOfOccupiedLabels)
	//		cudaMemsetAsync(numberOfOccupiedLabels, 0, sizeof(unsigned int), stream);
	//}

	//checkCudaErrors(cudaStreamSynchronize(stream));
}

__device__ __forceinline__ unsigned int LabelCounter::Hash_uint(unsigned int key)
{
	key = (key ^ 61) ^ (key >> 16);
	key *= 9;
	key = key ^ (key >> 4);
	key *= 0x27d4eb2d;
	key = key ^ (key >> 15);
	return key;
}

__device__ bool LabelCounter::Insert(unsigned int label, unsigned int count)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		unsigned int old = atomicCAS(&labels[index], EMPTY_LABEL, label);
		if (old == EMPTY_LABEL) {
			atomicAdd(&counts[index], 1);
			auto oldOccupiedIndex = atomicAdd(numberOfOccupiedLabels, 1);
			occupiedLabels[oldOccupiedIndex] = label;
			return true;
		}
		else if (old == label) {
			atomicAdd(&counts[index], 1);
			return true;
		}
	}
	return false;
}

__device__ bool LabelCounter::Find(unsigned int label, unsigned int& outCount) {
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			outCount = counts[index];
			return true;
		}
		if (labels[index] == EMPTY_LABEL) return false;
	}
	return false;
}

__device__ void LabelCounter::IncreaseCount(unsigned int label)
{
	if (capacity == 0 || labels == nullptr || counts == nullptr)
	{
		printf("LabelCounter::IncreaseCount called with invalid state\n");
		return;
	}

	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i)
	{
		unsigned int index = (slot + i) % capacity;

		unsigned int old = atomicCAS(&labels[index], EMPTY_LABEL, label);
		if (old == EMPTY_LABEL)
		{
			atomicAdd(&counts[index], 1);
			auto oldOccupiedIndex = atomicAdd(numberOfOccupiedLabels, 1);
			occupiedLabels[oldOccupiedIndex] = label;
			return;
		}
		else if (old == label)
		{
			atomicAdd(&counts[index], 1);
			return;
		}
	}

	printf("LabelCounter::IncreaseCount failed for label %u at slot %u\n", label, slot);
}

__device__ void LabelCounter::IncreaseRayHitCount(unsigned int label)
{
	if (capacity == 0 || labels == nullptr || counts == nullptr || rayHitCounts == nullptr)
	{
		printf("LabelCounter::IncreaseCount called with invalid state\n");
		return;
	}

	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i)
	{
		unsigned int index = (slot + i) % capacity;

		unsigned int old = atomicCAS(&labels[index], EMPTY_LABEL, label);
		if (old == EMPTY_LABEL)
		{
			atomicAdd(&rayHitCounts[index], 1);
			return;
		}
		else if (old == label)
		{
			atomicAdd(&rayHitCounts[index], 1);
			return;
		}
	}

	printf("LabelCounter::IncreaseRayHitCount failed for label %u at slot %u\n", label, slot);
}

__device__ unsigned int LabelCounter::GetCount(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			return counts[index];
		}
		if (labels[index] == EMPTY_LABEL) return 0;
	}
	return 0;
}

__device__ unsigned int LabelCounter::GetRayHitCount(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			return rayHitCounts[index];
		}
		if (labels[index] == EMPTY_LABEL) return 0;
	}
	return 0;
}

__device__ unsigned char LabelCounter::GetLabelState(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			return labelStates[index];
		}
		if (labels[index] == EMPTY_LABEL) return 0;
	}
	return 0;
}

__device__ bool LabelCounter::IsPatchConnected(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			return labelStates[index] == 2;
		}
		if (labels[index] == EMPTY_LABEL) return false;
	}
	return false;
}

__device__ bool LabelCounter::SetPatchConnected(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			labelStates[index] = 2;
			return true;
		}
		if (labels[index] == EMPTY_LABEL) return false;
	}
	return false;
}

__device__ bool LabelCounter::IsReservedToDelete(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			return labelStates[index] == 3;
		}
		if (labels[index] == EMPTY_LABEL) return false;
	}
	return false;
}

__device__ bool LabelCounter::SetReservedToDelete(unsigned int label)
{
	unsigned int slot = Hash_uint(label) % capacity;
	for (unsigned int i = 0; i < MAX_PROBE; ++i) {
		unsigned int index = (slot + i) % capacity;
		if (labels[index] == label) {
			labelStates[index] = 3;
			return true;
		}
		if (labels[index] == EMPTY_LABEL) return false;
	}
	return false;
}
