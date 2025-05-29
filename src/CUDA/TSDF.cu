#include <CUDA/TSDF.cuh>

//void TSDF::Initialize(size_t capacity)
//{
//	info.capacity = capacity;
//
//	cudaMalloc(&info.d_hashTable, sizeof(TSDFVoxel) * info.capacity);
//	cudaMemset(info.d_hashTable, 0, sizeof(TSDFVoxel) * info.capacity);
//
//	cudaMalloc(&info.d_numberOfOccupiedVoxels, sizeof(unsigned int));
//	cudaMemset(info.d_numberOfOccupiedVoxels, 0, sizeof(unsigned int));
//
//	cudaMalloc(&info.d_occupiedVoxelIndices, sizeof(int3) * info.capacity);
//	cudaMemset(info.d_occupiedVoxelIndices, 0, sizeof(int3) * info.capacity);
//
//	info.labelCounter.Initialize(info.capacity);
//	info.subLabelCounter.Initialize(info.capacity);
//}
//
//void TSDF::Terminate()
//{
//	cudaFree(info.d_hashTable);
//	cudaFree(info.d_numberOfOccupiedVoxels);
//	cudaFree(info.d_occupiedVoxelIndices);
//
//	info.labelCounter.Terminate();
//	info.subLabelCounter.Terminate();
//}
//
//__global__ void Kernel_InsertPoints(TSDFInfo info, PointCloudBuffers buffers)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//	if (idx >= buffers.numberOfPoints) return;
//
//	auto p = buffers.positions[idx];
//	auto n = buffers.normals[idx].normalized();
//	auto c = buffers.colors[idx];
//
//	int3 coord = make_int3(floorf(p.x() / info.voxelSize), floorf(p.y() / info.voxelSize), floorf(p.z() / info.voxelSize));
//
//	// 중심 voxel 슬롯을 확보 (안정성 확보용)
//	auto centerVoxelSlot = GetTSDFVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == centerVoxelSlot)
//	{
//		InsertTSDFVoxel(info, coord, n, c);
//	}
//
//	// 중심 위치
//	Eigen::Vector3f voxelCenterBase(
//		(coord.x + 0.5f) * info.voxelSize,
//		(coord.y + 0.5f) * info.voxelSize,
//		(coord.z + 0.5f) * info.voxelSize);
//
//	// truncate 거리
//	const float truncation = info.truncation;
//
//	int offset = 1;
//
//	for (int zi = -offset; zi <= offset; zi++)
//	{
//		for (int yi = -offset; yi <= offset; yi++)
//		{
//			for (int xi = -offset; xi <= offset; xi++)
//			{
//				int3 neighborCoord = make_int3(coord.x + xi, coord.y + yi, coord.z + zi);
//
//				size_t neighborSlot = GetTSDFVoxelSlot(info, neighborCoord);
//				if (INVALID_VOXEL_SLOT == neighborSlot)
//				{
//					InsertTSDFVoxel(info, neighborCoord, n, c);
//				}
//
//				auto neighborVoxel = GetTSDFVoxel(info, neighborSlot);
//				if (neighborVoxel == nullptr) continue;
//
//				Eigen::Vector3f neighborVoxelCenter(
//					(neighborCoord.x + 0.5f) * info.voxelSize,
//					(neighborCoord.y + 0.5f) * info.voxelSize,
//					(neighborCoord.z + 0.5f) * info.voxelSize);
//
//				// signed distance = dot(p - voxel, n)
//				float signed_distance = (p - neighborVoxelCenter).dot(n);
//
//				// truncate
//				if (signed_distance < -truncation) continue;
//				float tsdf = fminf(1.0f, signed_distance / truncation);
//				tsdf = fmaxf(-1.0f, tsdf);  // Clamp between -1 and 1
//
//				// 가중치 업데이트
//				float w_old = neighborVoxel->weight;
//				float w_new = 1.0f;
//				if (FLT_MAX != neighborVoxel->value)
//				{
//					neighborVoxel->value = (w_old * neighborVoxel->value + w_new * tsdf) / (w_old + w_new);
//				}
//				else
//				{
//					neighborVoxel->value = tsdf;
//				}
//				neighborVoxel->weight += w_new;
//			}
//		}
//	}
//}
//
////void TSDF::InsertPoints(PointCloudBuffers buffers)
////{
////	unsigned int blockSize = 256;
////	unsigned int gridOccupied = (buffers.numberOfPoints + blockSize - 1) / blockSize;
////
////	Kernel_InsertPoints << <gridOccupied, blockSize >> > (info, buffers);
////
////	cudaDeviceSynchronize();
////
////	cudaMemcpy(&info.h_numberOfOccupiedVoxels, info.d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);
////}
//
//__global__ void Kernel_CountLabels(TSDFInfo info)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//	if (idx >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[idx];
//	auto voxelSlot = GetTSDFVoxelSlot(info, coord);
//	if (UINT64_MAX == voxelSlot) return;
//
//	auto voxel = GetTSDFVoxel(info, voxelSlot);
//	if (nullptr == voxel) return;
//
//	info.labelCounter.IncreaseCount(voxel->label);
//	info.subLabelCounter.IncreaseCount(voxel->subLabel);
//}
//
//void TSDF::CountLabels()
//{
//	info.labelCounter.Clear();
//	info.subLabelCounter.Clear();
//
//	//info.labelCounter.Resize(info.capacity * 2);
//	//info.subLabelCounter.Resize(info.capacity * 2);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (info.h_numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_CountLabels << <gridOccupied, blockSize >> > (info);
//
//	cudaDeviceSynchronize();
//}
//
//__global__ void Kernel_Serialize_TSDF(TSDFInfo info, PointCloudBuffers buffers)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//	if (idx >= *info.d_numberOfOccupiedVoxels) return;
//
//	int3 coord = info.d_occupiedVoxelIndices[idx];
//
//	auto voxelSlot = GetTSDFVoxelSlot(info, coord);
//	if (INVALID_VOXEL_SLOT == voxelSlot) return;
//
//	auto voxel = GetTSDFVoxel(info, voxelSlot);
//	if (nullptr == voxel) return;
//
//	if (-info.voxelSize <= voxel->value && voxel->value <= info.voxelSize)
//	{
//		buffers.positions[idx] = Eigen::Vector3f(
//			(float)voxel->coord.x * info.voxelSize,
//			(float)voxel->coord.y * info.voxelSize,
//			(float)voxel->coord.z * info.voxelSize);
//		buffers.normals[idx] = (voxel->normal / (float)voxel->pointCount).normalized();
//		buffers.colors[idx] = voxel->color;
//	}
//}
//
//void TSDF::SerializeToPLY(const std::string& filename)
//{
//	PLYFormat ply;
//
//	unsigned int numberOfOccupiedVoxels = info.h_numberOfOccupiedVoxels;
//
//	PointCloudBuffers d_buffers;
//	d_buffers.Initialize(numberOfOccupiedVoxels, false);
//
//	unsigned int blockSize = 256;
//	unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	Kernel_Serialize_TSDF << <gridOccupied, blockSize >> > (info, d_buffers);
//
//	cudaDeviceSynchronize();
//
//	PointCloudBuffers h_buffers;
//	h_buffers.Initialize(numberOfOccupiedVoxels, true);
//
//	d_buffers.CopyTo(h_buffers);
//
//	for (size_t i = 0; i < numberOfOccupiedVoxels; i++)
//	{
//		auto& p = h_buffers.positions[i];
//		auto& n = h_buffers.normals[i];
//		auto& c = h_buffers.colors[i];
//
//		//printf("%3d, %3d, %3d\n", c.x(), c.y(), c.z());
//
//		ply.AddPoint(p.x(), p.y(), p.z());
//		ply.AddNormal(n.x(), n.y(), n.z());
//		ply.AddColor(
//			fminf(1.0f, fmaxf(0.0f, c.x() / 255.0f)),
//			fminf(1.0f, fmaxf(0.0f, c.y() / 255.0f)),
//			fminf(1.0f, fmaxf(0.0f, c.z() / 255.0f))
//		);
//	}
//
//	ply.Serialize(filename);
//
//	d_buffers.Terminate();
//	h_buffers.Terminate();
//}
//
//__device__ size_t GetTSDFVoxelSlot(TSDFInfo& info, int3 coord)
//{
//	size_t h = voxel_hash(coord, info.capacity);
//	for (int i = 0; i < info.maxProbe; ++i) {
//		size_t slot = (h + i) % info.capacity;
//
//		if (info.d_hashTable[slot].coord.x == coord.x &&
//			info.d_hashTable[slot].coord.y == coord.y &&
//			info.d_hashTable[slot].coord.z == coord.z)
//		{
//			return slot;
//		}
//	}
//
//	return INVALID_VOXEL_SLOT;
//}
//
//__device__ TSDFVoxel* GetTSDFVoxel(TSDFInfo& info, size_t slot)
//{
//	if (INVALID_VOXEL_SLOT == slot) return nullptr;
//	return &(info.d_hashTable[slot]);
//}
//
//__device__ void InsertTSDFVoxel(TSDFInfo& info, const int3& coord, const Eigen::Vector3f& normal, const Eigen::Vector4b& color)
//{
//	size_t h = voxel_hash(coord, info.capacity);
//
//	for (int i = 0; i < info.maxProbe; ++i)
//	{
//		size_t slot = (h + i) % info.capacity;
//		TSDFVoxel* voxel = &info.d_hashTable[slot];
//
//		int oldLabel = atomicCAS(&(voxel->label), 0, slot); // atomic하게 점유 시도
//
//		if (oldLabel == 0)
//		{
//			// 빈 슬롯에 삽입 성공
//			voxel->coord = coord;
//			voxel->normal = normal;
//			voxel->color = color;
//			voxel->pointCount = 1;
//			voxel->weight = 0.0f;
//			voxel->value = FLT_MAX;
//			voxel->label = slot; // atomicCAS에서 설정되었지만 명시적으로 유지
//			voxel->subLabel = 0;
//
//			// occupied index 기록
//			unsigned int index = atomicAdd(info.d_numberOfOccupiedVoxels, 1);
//			info.d_occupiedVoxelIndices[index] = coord;
//			return;
//		}
//		else
//		{
//			// 이미 존재하는 voxel이면 누적
//			if (voxel->coord.x == coord.x &&
//				voxel->coord.y == coord.y &&
//				voxel->coord.z == coord.z)
//			{
//				// atomic이 필요할 수 있음 (성능/정확도 균형 선택)
//				voxel->normal += normal;
//				voxel->color = color; // 색상은 마지막 포인트 기준 (가중 평균 필요 시 수정)
//				voxel->pointCount++;
//				return;
//			}
//		}
//	}
//
//	// 슬롯을 찾지 못함 (해시 테이블 가득 찼거나 maxProbe 초과)
//	// -> 무시 또는 디버그 출력 가능
//}
