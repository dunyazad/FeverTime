#include <CUDA/PCD.cuh>

void DevicePointCloud::Clear(size_t numberOfPoints)
{
	if (0 != numberOfPoints && numberOfElements < numberOfPoints)
	{
		numberOfElements = numberOfPoints;
		positions.resize(numberOfPoints);
		normals.resize(numberOfPoints);
		colors.resize(numberOfPoints);
	}

	thrust::fill(positions.begin(), positions.begin() + numberOfPoints, make_float3(0.0f, 0.0f, 0.0f));
	thrust::fill(normals.begin(), normals.begin() + numberOfPoints, make_float3(0.0f, 0.0f, 0.0f));
	thrust::fill(colors.begin(), colors.begin() + numberOfPoints, make_uchar4(0, 0, 0, 255));

	aabb = Eigen::AlignedBox3f(
		Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
		Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

	hashmap.Clear(numberOfElements * 20);
}

void DevicePointCloud::CopyFrom(DevicePointCloud* pointCloud)
{
	if (numberOfElements < pointCloud->numberOfElements)
	{
		numberOfElements = pointCloud->numberOfElements;
		positions.resize(numberOfElements);
		normals.resize(numberOfElements);
		colors.resize(numberOfElements);
	}

	thrust::copy(pointCloud->positions.begin(), pointCloud->positions.begin() + numberOfElements, positions.begin());
	thrust::copy(pointCloud->normals.begin(), pointCloud->normals.begin() + numberOfElements, normals.begin());
	thrust::copy(pointCloud->colors.begin(), pointCloud->colors.begin() + numberOfElements, colors.begin());

	aabb = pointCloud->aabb;

	hashmap.Clear(numberOfElements * 20);

	auto d_positions = thrust::raw_pointer_cast(positions.data());
	auto d_normals = thrust::raw_pointer_cast(normals.data());
	auto d_colors = thrust::raw_pointer_cast(colors.data());

	hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void DevicePointCloud::CopyFrom(HostPointCloud* pointCloud)
{
	if (numberOfElements < pointCloud->numberOfElements)
	{
		numberOfElements = pointCloud->numberOfElements;
		positions.resize(numberOfElements);
		normals.resize(numberOfElements);
		colors.resize(numberOfElements);
	}

	thrust::copy(pointCloud->positions.begin(), pointCloud->positions.begin() + numberOfElements, positions.begin());
	thrust::copy(pointCloud->normals.begin(), pointCloud->normals.begin() + numberOfElements, normals.begin());
	thrust::copy(pointCloud->colors.begin(), pointCloud->colors.begin() + numberOfElements, colors.begin());

	aabb = pointCloud->aabb;

	hashmap.Clear(numberOfElements * 20);

	auto d_positions = thrust::raw_pointer_cast(positions.data());
	auto d_normals = thrust::raw_pointer_cast(normals.data());
	auto d_colors = thrust::raw_pointer_cast(colors.data());

	hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void DevicePointCloud::CopyTo(DevicePointCloud* pointCloud)
{
	if (pointCloud->numberOfElements < numberOfElements)
	{
		pointCloud->numberOfElements = numberOfElements;
		pointCloud->positions.resize(numberOfElements);
		pointCloud->normals.resize(numberOfElements);
		pointCloud->colors.resize(numberOfElements);
	}

	thrust::copy(positions.begin(), positions.begin() + numberOfElements, pointCloud->positions.begin());
	thrust::copy(normals.begin(), normals.begin() + numberOfElements, pointCloud->normals.begin());
	thrust::copy(colors.begin(), colors.begin() + numberOfElements, pointCloud->colors.begin());

	pointCloud->aabb = aabb;

	pointCloud->hashmap.Clear(numberOfElements * 20);

	thrust::device_vector<float3> dv_positions(pointCloud->positions);
	thrust::device_vector<float3> dv_normals(pointCloud->normals);
	thrust::device_vector<uchar4> dv_colors(pointCloud->colors);

	auto d_positions = thrust::raw_pointer_cast(dv_positions.data());
	auto d_normals = thrust::raw_pointer_cast(dv_normals.data());
	auto d_colors = thrust::raw_pointer_cast(dv_colors.data());

	pointCloud->hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void DevicePointCloud::CopyTo(HostPointCloud* pointCloud)
{
	if (pointCloud->numberOfElements < numberOfElements)
	{
		pointCloud->numberOfElements = numberOfElements;
		pointCloud->positions.resize(numberOfElements);
		pointCloud->normals.resize(numberOfElements);
		pointCloud->colors.resize(numberOfElements);
	}

	thrust::copy(positions.begin(), positions.begin() + numberOfElements, pointCloud->positions.begin());
	thrust::copy(normals.begin(), normals.begin() + numberOfElements, pointCloud->normals.begin());
	thrust::copy(colors.begin(), colors.begin() + numberOfElements, pointCloud->colors.begin());

	pointCloud->aabb = aabb;

	pointCloud->hashmap.Clear(numberOfElements * 20);

	thrust::device_vector<float3> dv_positions(pointCloud->positions);
	thrust::device_vector<float3> dv_normals(pointCloud->normals);
	thrust::device_vector<uchar4> dv_colors(pointCloud->colors);

	auto d_positions = thrust::raw_pointer_cast(dv_positions.data());
	auto d_normals = thrust::raw_pointer_cast(dv_normals.data());
	auto d_colors = thrust::raw_pointer_cast(dv_colors.data());

	pointCloud->hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void DevicePointCloud::Compact()
{
	auto is_invalid = [] __device__(const TupleType & t)
	{
		const float3& p = thrust::get<0>(t);
		return (FLT_MAX == p.x && FLT_MAX == p.y && FLT_MAX == p.z);
	};

	DeviceZipIter zip_begin = thrust::make_zip_iterator(thrust::make_tuple(
		positions.begin(), normals.begin(), colors.begin()));
	DeviceZipIter zip_end = thrust::make_zip_iterator(thrust::make_tuple(
		positions.end(), normals.end(), colors.end()));

	DeviceZipIter zip_new_end = thrust::remove_if(thrust::device, zip_begin, zip_end, is_invalid);

	numberOfElements = thrust::get<0>(zip_new_end.get_iterator_tuple()) - positions.begin();

	positions.resize(numberOfElements);
	normals.resize(numberOfElements);
	colors.resize(numberOfElements);

	aabb = Eigen::AlignedBox3f(
		Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
		Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

	thrust::host_vector<float3> h_positions(positions);
	for (const auto& p : h_positions)
	{
		aabb.extend(Eigen::Vector3f(p.x, p.y, p.z));
	}

	hashmap.Clear(numberOfElements * 20);

	auto d_positions = thrust::raw_pointer_cast(positions.data());
	auto d_normals = thrust::raw_pointer_cast(normals.data());
	auto d_colors = thrust::raw_pointer_cast(colors.data());

	hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

bool DevicePointCloud::LoadFromPLY(const string& filename)
{
	PLYFormat ply;
	if (false == ply.Deserialize(filename))
	{
		return false;
	}

	auto nop = ply.GetPoints().size() / 3;
	auto non = ply.GetNormals().size() / 3;
	auto noc = ply.GetColors().size() / 3;
	if (ply.GetPoints().size() != ply.GetColors().size() && nop == ply.GetColors().size() / 4)
	{
		noc = ply.GetColors().size() / 4;
	}

	thrust::host_vector<float3> h_positions(nop);
	thrust::host_vector<float3> h_normals(non);
	thrust::host_vector<uchar4> h_colors(noc);
	
	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];
	
		auto nx = ply.GetNormals()[i * 3];
		auto ny = ply.GetNormals()[i * 3 + 1];
		auto nz = ply.GetNormals()[i * 3 + 2];
	
		if (ply.GetPoints().size() == ply.GetColors().size())
		{
			auto r = ply.GetColors()[i * 3];
			auto g = ply.GetColors()[i * 3 + 1];
			auto b = ply.GetColors()[i * 3 + 2];
	
			h_positions[i] = make_float3(x, y, z);
			h_normals[i] = make_float3(nx, ny, nz);
			h_colors[i] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, 255);
		}
		else if (ply.GetPoints().size() / 3 == ply.GetColors().size() / 4)
		{
			auto r = ply.GetColors()[i * 4];
			auto g = ply.GetColors()[i * 4 + 1];
			auto b = ply.GetColors()[i * 4 + 2];
			auto a = ply.GetColors()[i * 4 + 3];
	
			h_positions[i] = make_float3(x, y, z);
			h_normals[i] = make_float3(nx, ny, nz);
			h_colors[i] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, a * 255.0f);
		}
	
		aabb.extend(Eigen::Vector3f(x, y, z));
	}
	
	positions.resize(h_positions.size());
	thrust::copy(h_positions.begin(), h_positions.end(), positions.begin());

	normals.resize(h_normals.size());
	thrust::copy(h_normals.begin(), h_normals.end(), normals.begin());

	colors.resize(h_colors.size());
	thrust::copy(h_colors.begin(), h_colors.end(), colors.begin());

	numberOfElements = h_positions.size();

	hashmap.Clear(numberOfElements * 20);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(positions.data()),
		thrust::raw_pointer_cast(normals.data()),
		thrust::raw_pointer_cast(colors.data()),
		numberOfElements);

	return true;
}

bool DevicePointCloud::LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi)
{
	PLYFormat ply;
	if (false == ply.Deserialize(filename))
	{
		return false;
	}

	auto nop = ply.GetPoints().size() / 3;
	auto non = ply.GetNormals().size() / 3;
	auto noc = ply.GetColors().size() / 3;
	if (ply.GetPoints().size() != ply.GetColors().size() && nop == ply.GetColors().size() / 4)
	{
		noc = ply.GetColors().size() / 4;
	}

	thrust::host_vector<float3> h_positions(nop);
	thrust::host_vector<float3> h_normals(non);
	thrust::host_vector<uchar4> h_colors(noc);

	size_t skipCount = 0;
	for (size_t i = 0; i < nop; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];

		if (false == roi.contains(Eigen::Vector3f(x, y, z)))
		{
			skipCount++;
			continue;
		}

		auto nx = ply.GetNormals()[i * 3];
		auto ny = ply.GetNormals()[i * 3 + 1];
		auto nz = ply.GetNormals()[i * 3 + 2];

		if (ply.GetPoints().size() == ply.GetColors().size())
		{
			auto r = ply.GetColors()[i * 3];
			auto g = ply.GetColors()[i * 3 + 1];
			auto b = ply.GetColors()[i * 3 + 2];

			h_positions[i - skipCount] = make_float3(x, y, z);
			h_normals[i - skipCount] = make_float3(nx, ny, nz);
			h_colors[i - skipCount] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, 255);
		}
		else if (nop == noc)
		{
			auto r = ply.GetColors()[i * 4];
			auto g = ply.GetColors()[i * 4 + 1];
			auto b = ply.GetColors()[i * 4 + 2];
			auto a = ply.GetColors()[i * 4 + 3];

			h_positions[i - skipCount] = make_float3(x, y, z);
			h_normals[i - skipCount] = make_float3(nx, ny, nz);
			h_colors[i - skipCount] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, a * 255.0f);
		}

		aabb.extend(Eigen::Vector3f(x, y, z));
	}

	positions.resize(nop - skipCount);
	thrust::copy(h_positions.begin(), h_positions.begin() + nop - skipCount, positions.begin());

	normals.resize(non - skipCount);
	thrust::copy(h_normals.begin(), h_normals.begin() + nop - skipCount, normals.begin());

	colors.resize(noc - skipCount);
	thrust::copy(h_colors.begin(), h_colors.begin() + nop - skipCount, colors.begin());

	numberOfElements = nop - skipCount;

	hashmap.Clear(numberOfElements * 20);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(positions.data()),
		thrust::raw_pointer_cast(normals.data()),
		thrust::raw_pointer_cast(colors.data()),
		numberOfElements);

	return true;
}

bool DevicePointCloud::SaveToPLY(const string& filename)
{
	thrust::host_vector<float3> h_positions(positions);
	thrust::host_vector<float3> h_normals(normals);
	thrust::host_vector<uchar4> h_colors(colors);

	PLYFormat ply;
	
	for (size_t i = 0; i < numberOfElements; i++)
	{
		auto& p = h_positions[i];
		auto& n = h_normals[i];
		auto& c = h_colors[i];
	
		ply.AddPoint(p.x, p.y, p.z);
		ply.AddNormal(n.x, n.y, n.z);
		ply.AddColor((float)c.x / 255.0f, (float)c.y / 255.0f, (float)c.z / 255.0f, (float)c.w / 255.0f);
	}
	
	ply.Serialize(filename);

	return true;
}

bool DevicePointCloud::LoadFromALP(const string& filename)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}
	
	thrust::host_vector<float3> h_positions(alp.GetPoints().size());
	thrust::host_vector<float3> h_normals(alp.GetPoints().size());
	thrust::host_vector<uchar4> h_colors(alp.GetPoints().size());
	
	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];
	
		h_positions[i] = p.position;
		h_normals[i] = p.normal;
		h_colors[i].x = p.color.x * 255.0f;
		h_colors[i].y = p.color.y * 255.0f;
		h_colors[i].z = p.color.z * 255.0f;
		h_colors[i].w = 255;
	
		aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
	}
	
	positions.resize(h_positions.size());
	thrust::copy(h_positions.begin(), h_positions.end(), positions.begin());

	normals.resize(h_normals.size());
	thrust::copy(h_normals.begin(), h_normals.end(), normals.begin());

	colors.resize(h_colors.size());
	thrust::copy(h_colors.begin(), h_colors.end(), colors.begin());

	numberOfElements = h_positions.size();

	hashmap.Clear(numberOfElements * 20);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(positions.data()),
		thrust::raw_pointer_cast(normals.data()),
		thrust::raw_pointer_cast(colors.data()),
		numberOfElements);

	return true;
}

bool DevicePointCloud::LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}

	thrust::host_vector<float3> h_positions(alp.GetPoints().size());
	thrust::host_vector<float3> h_normals(alp.GetPoints().size());
	thrust::host_vector<uchar4> h_colors(alp.GetPoints().size());

	size_t skipCount = 0;
	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];

		if (false == roi.contains(Eigen::Vector3f(p.position.x, p.position.y, p.position.z)))
		{
			skipCount++;
			continue;
		}

		h_positions[i] = p.position;
		h_normals[i] = p.normal;
		h_colors[i].x = p.color.x * 255.0f;
		h_colors[i].y = p.color.y * 255.0f;
		h_colors[i].z = p.color.z * 255.0f;
		h_colors[i].w = 255;

		aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
	}

	positions.resize(h_positions.size() - skipCount);
	thrust::copy(h_positions.begin(), h_positions.begin() + h_positions.size() - skipCount, positions.begin());

	normals.resize(h_normals.size() - skipCount);
	thrust::copy(h_normals.begin(), h_normals.begin() + h_positions.size() - skipCount, normals.begin());

	colors.resize(h_colors.size() - skipCount);
	thrust::copy(h_colors.begin(), h_colors.begin() + h_positions.size() - skipCount, colors.begin());

	numberOfElements = h_positions.size() - skipCount;

	hashmap.Clear(numberOfElements * 20);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(positions.data()),
		thrust::raw_pointer_cast(normals.data()),
		thrust::raw_pointer_cast(colors.data()),
		numberOfElements);

	return true;
}

bool DevicePointCloud::SaveToALP(const string& filename)
{
	thrust::host_vector<float3> h_positions(positions);
	thrust::host_vector<float3> h_normals(normals);
	thrust::host_vector<uchar4> h_colors(colors);

	ALPFormat<PointPNC> alp;
	
	for (size_t i = 0; i < numberOfElements; i++)
	{
		PointPNC point;
		point.position = h_positions[i];
		point.normal = h_normals[i];
		point.color.x = (float)h_colors[i].x / 255.0f;
		point.color.y = (float)h_colors[i].y / 255.0f;
		point.color.z = (float)h_colors[i].z / 255.0f;

		alp.AddPoint(point);
	}
	
	alp.Serialize(filename);

	return true;
}

__global__ void Kernel_PickPointWeightedByDepth(
	float3* positions,
	float3* normals,
	uchar4* colors,
	int numPoints,
	float3 rayOrigin,
	float3 rayDir,           // 정규화된 ray 방향
	float radiusThreshold,
	float alpha,  // 수직 거리 가중치
	float beta,   // depth 가중치
	int* bestIndex,
	float* minScore
) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numPoints) return;

	float3 p = positions[idx];
	float3 v = p - rayOrigin;

	float t = dot(v, rayDir);

	float3 proj = rayOrigin + t * rayDir;
	float3 offset = p - proj;
	float d_perp = sqrt(dot(offset, offset));
	if (d_perp > radiusThreshold) return;

	//colors[idx].x = 255;
	//colors[idx].y = 0;
	//colors[idx].z = 0;
	//colors[idx].w = 255;

	float score = alpha * d_perp + beta * t;

	float prev = atomicMinFloat(minScore, score);
	if (prev > score) {
		*bestIndex = idx;
	}
}

__global__ void Kernel_ChangePickedColor(
	float3* positions,
	float3* normals,
	uchar4* colors,
	int numPoints,
	int* bestIndex)
{
	colors[*bestIndex].x = 255;
	colors[*bestIndex].y = 0;
	colors[*bestIndex].z = 0;
	colors[*bestIndex].w = 255;
}

size_t DevicePointCloud::Pick(float3 rayOrigin, float3 rayDirection)
{
	int* d_closestIndex;
	float* d_minDistSq;
	cudaMalloc(&d_closestIndex, sizeof(int));
	cudaMalloc(&d_minDistSq, sizeof(float));

	int initIdx = -1;
	float initDist = FLT_MAX;
	cudaMemcpy(d_closestIndex, &initIdx, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_minDistSq, &initDist, sizeof(float), cudaMemcpyHostToDevice);

	float alpha = 1.0f;  // ray 중심성
	float beta = 0.25f;  // depth 우선순위

	int threads = 256;
	int blocks = (numberOfElements + threads - 1) / threads;
	Kernel_PickPointWeightedByDepth << <blocks, threads >> > (
		thrust::raw_pointer_cast(positions.data()),
		thrust::raw_pointer_cast(normals.data()),
		thrust::raw_pointer_cast(colors.data()),
		numberOfElements,
		rayOrigin,
		rayDirection,
		5.0f,
		alpha,
		beta,
		d_closestIndex,
		d_minDistSq);

	Kernel_ChangePickedColor << < 1, 1 >> > (
		thrust::raw_pointer_cast(positions.data()),
		thrust::raw_pointer_cast(normals.data()),
		thrust::raw_pointer_cast(colors.data()),
		numberOfElements,
		d_closestIndex);

	int h_closestIndex = -1;
	cudaMemcpy(&h_closestIndex, d_closestIndex, sizeof(int), cudaMemcpyDeviceToHost);

	printf("h_closestIndex = %d\n", h_closestIndex);

	cudaFree(d_closestIndex);
	cudaFree(d_minDistSq);

	return h_closestIndex >= 0 ? static_cast<size_t>(h_closestIndex) : size_t(-1);
}







void HostPointCloud::Clear(size_t numberOfPoints)
{
	if (0 != numberOfPoints && numberOfPoints != positions.size())
	{
		positions.resize(numberOfPoints);
		normals.resize(numberOfPoints);
		colors.resize(numberOfPoints);
	}

	thrust::fill(positions.begin(), positions.end(), make_float3(0.0f, 0.0f, 0.0f));
	thrust::fill(normals.begin(), normals.end(), make_float3(0.0f, 0.0f, 0.0f));
	thrust::fill(colors.begin(), colors.end(), make_uchar4(0, 0, 0, 255));

	aabb = Eigen::AlignedBox3f(
		Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
		Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
}

void HostPointCloud::CopyFrom(DevicePointCloud* pointCloud)
{
	if (numberOfElements < pointCloud->numberOfElements)
	{
		numberOfElements = pointCloud->numberOfElements;
		positions.resize(numberOfElements);
		normals.resize(numberOfElements);
		colors.resize(numberOfElements);
	}

	thrust::copy(pointCloud->positions.begin(), pointCloud->positions.begin() + numberOfElements, positions.begin());
	thrust::copy(pointCloud->normals.begin(), pointCloud->normals.begin() + numberOfElements, normals.begin());
	thrust::copy(pointCloud->colors.begin(), pointCloud->colors.begin() + numberOfElements, colors.begin());

	aabb = pointCloud->aabb;

	hashmap.Clear(numberOfElements * 20);

	auto d_positions = thrust::raw_pointer_cast(pointCloud->positions.data());
	auto d_normals = thrust::raw_pointer_cast(pointCloud->normals.data());
	auto d_colors = thrust::raw_pointer_cast(pointCloud->colors.data());

	hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void HostPointCloud::CopyFrom(HostPointCloud* pointCloud)
{
	if (numberOfElements < pointCloud->numberOfElements)
	{
		numberOfElements = pointCloud->numberOfElements;
		positions.resize(numberOfElements);
		normals.resize(numberOfElements);
		colors.resize(numberOfElements);
	}

	thrust::copy(pointCloud->positions.begin(), pointCloud->positions.begin() + numberOfElements, positions.begin());
	thrust::copy(pointCloud->normals.begin(), pointCloud->normals.begin() + numberOfElements, normals.begin());
	thrust::copy(pointCloud->colors.begin(), pointCloud->colors.begin() + numberOfElements, colors.begin());

	aabb = pointCloud->aabb;

	hashmap.Clear(numberOfElements * 20);

	auto d_positions = thrust::raw_pointer_cast(positions.data());
	auto d_normals = thrust::raw_pointer_cast(normals.data());
	auto d_colors = thrust::raw_pointer_cast(colors.data());

	hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void HostPointCloud::CopyTo(DevicePointCloud* pointCloud)
{
	if (pointCloud->numberOfElements < numberOfElements)
	{
		pointCloud->numberOfElements = numberOfElements;
		pointCloud->positions.resize(numberOfElements);
		pointCloud->normals.resize(numberOfElements);
		pointCloud->colors.resize(numberOfElements);
	}

	thrust::copy(positions.begin(), positions.begin() + numberOfElements, pointCloud->positions.begin());
	thrust::copy(normals.begin(), normals.begin() + numberOfElements, pointCloud->normals.begin());
	thrust::copy(colors.begin(), colors.begin() + numberOfElements, pointCloud->colors.begin());

	pointCloud->aabb = aabb;

	pointCloud->hashmap.Clear(numberOfElements * 20);

	thrust::device_vector<float3> dv_positions(pointCloud->positions);
	thrust::device_vector<float3> dv_normals(pointCloud->normals);
	thrust::device_vector<uchar4> dv_colors(pointCloud->colors);

	auto d_positions = thrust::raw_pointer_cast(dv_positions.data());
	auto d_normals = thrust::raw_pointer_cast(dv_normals.data());
	auto d_colors = thrust::raw_pointer_cast(dv_colors.data());

	pointCloud->hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void HostPointCloud::CopyTo(HostPointCloud* pointCloud)
{
	if (pointCloud->numberOfElements < numberOfElements)
	{
		pointCloud->numberOfElements = numberOfElements;
		pointCloud->positions.resize(numberOfElements);
		pointCloud->normals.resize(numberOfElements);
		pointCloud->colors.resize(numberOfElements);
	}

	thrust::copy(positions.begin(), positions.begin() + numberOfElements, pointCloud->positions.begin());
	thrust::copy(normals.begin(), normals.begin() + numberOfElements, pointCloud->normals.begin());
	thrust::copy(colors.begin(), colors.begin() + numberOfElements, pointCloud->colors.begin());

	pointCloud->aabb = aabb;

	pointCloud->hashmap.Clear(numberOfElements * 20);

	thrust::device_vector<float3> dv_positions(pointCloud->positions);
	thrust::device_vector<float3> dv_normals(pointCloud->normals);
	thrust::device_vector<uchar4> dv_colors(pointCloud->colors);

	auto d_positions = thrust::raw_pointer_cast(dv_positions.data());
	auto d_normals = thrust::raw_pointer_cast(dv_normals.data());
	auto d_colors = thrust::raw_pointer_cast(dv_colors.data());

	pointCloud->hashmap.InsertPoints(d_positions, d_normals, d_colors, numberOfElements);
}

void HostPointCloud::Compact()
{
	auto is_invalid = [] __device__(const TupleType & t)
	{
		const float3& p = thrust::get<0>(t);
		return (FLT_MAX == p.x && FLT_MAX == p.y && FLT_MAX == p.z);
	};

	HostZipIter zip_begin = thrust::make_zip_iterator(thrust::make_tuple(positions.begin(), normals.begin(), colors.begin()));
	HostZipIter zip_end = thrust::make_zip_iterator(thrust::make_tuple(positions.end(), normals.end(), colors.end()));

	HostZipIter zip_new_end = thrust::remove_if(thrust::host, zip_begin, zip_end, is_invalid);

	numberOfElements = thrust::get<0>(zip_new_end.get_iterator_tuple()) - positions.begin();

	positions.resize(numberOfElements);
	normals.resize(numberOfElements);
	colors.resize(numberOfElements);

	aabb = Eigen::AlignedBox3f(
		Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
		Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

	for (const auto& p : positions)
	{
		aabb.extend(Eigen::Vector3f(p.x, p.y, p.z));
	}

	hashmap.Clear(numberOfElements * 20);

	thrust::device_vector<float3> d_positions(positions);
	thrust::device_vector<float3> d_normals(normals);
	thrust::device_vector<uchar4> d_colors(colors);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(d_positions.data()),
		thrust::raw_pointer_cast(d_normals.data()),
		thrust::raw_pointer_cast(d_colors.data()),
		numberOfElements);
}

bool HostPointCloud::LoadFromPLY(const string& filename)
{
	PLYFormat ply;
	if (false == ply.Deserialize(filename))
	{
		return false;
	}

	auto nop = ply.GetPoints().size() / 3;
	auto non = ply.GetNormals().size() / 3;
	auto noc = ply.GetColors().size() / 3;
	if (ply.GetPoints().size() != ply.GetColors().size() && nop == ply.GetColors().size() / 4)
	{
		noc = ply.GetColors().size() / 4;
	}

	positions.resize(nop);
	normals.resize(non);
	colors.resize(noc);

	for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];

		auto nx = ply.GetNormals()[i * 3];
		auto ny = ply.GetNormals()[i * 3 + 1];
		auto nz = ply.GetNormals()[i * 3 + 2];

		if (ply.GetPoints().size() == ply.GetColors().size())
		{
			auto r = ply.GetColors()[i * 3];
			auto g = ply.GetColors()[i * 3 + 1];
			auto b = ply.GetColors()[i * 3 + 2];

			positions[i] = make_float3(x, y, z);
			normals[i] = make_float3(nx, ny, nz);
			colors[i] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, 255);
		}
		else if (ply.GetPoints().size() / 3 == ply.GetColors().size() / 4)
		{
			auto r = ply.GetColors()[i * 4];
			auto g = ply.GetColors()[i * 4 + 1];
			auto b = ply.GetColors()[i * 4 + 2];
			auto a = ply.GetColors()[i * 4 + 3];

			positions[i] = make_float3(x, y, z);
			normals[i] = make_float3(nx, ny, nz);
			colors[i] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, a * 255.0f);
		}

		aabb.extend(Eigen::Vector3f(x, y, z));
	}

	numberOfElements = ply.GetPoints().size() / 3;

	thrust::device_vector<float3> d_positions(positions);
	thrust::device_vector<float3> d_normals(normals);
	thrust::device_vector<uchar4> d_colors(colors);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(d_positions.data()),
		thrust::raw_pointer_cast(d_normals.data()),
		thrust::raw_pointer_cast(d_colors.data()),
		numberOfElements);

	return true;
}

bool HostPointCloud::LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi)
{
	PLYFormat ply;
	if (false == ply.Deserialize(filename))
	{
		return false;
	}

	auto nop = ply.GetPoints().size() / 3;
	auto non = ply.GetNormals().size() / 3;
	auto noc = ply.GetColors().size() / 3;
	if (ply.GetPoints().size() != ply.GetColors().size() && nop == ply.GetColors().size() / 4)
	{
		noc = ply.GetColors().size() / 4;
	}

	positions.resize(nop);
	normals.resize(non);
	colors.resize(noc);

	size_t skipCount = 0;
	for (size_t i = 0; i < nop; i++)
	{
		auto x = ply.GetPoints()[i * 3];
		auto y = ply.GetPoints()[i * 3 + 1];
		auto z = ply.GetPoints()[i * 3 + 2];

		if (false == roi.contains(Eigen::Vector3f(x, y, z)))
		{
			skipCount++;
			continue;
		}

		auto nx = ply.GetNormals()[i * 3];
		auto ny = ply.GetNormals()[i * 3 + 1];
		auto nz = ply.GetNormals()[i * 3 + 2];

		if (ply.GetPoints().size() == ply.GetColors().size())
		{
			auto r = ply.GetColors()[i * 3];
			auto g = ply.GetColors()[i * 3 + 1];
			auto b = ply.GetColors()[i * 3 + 2];

			positions[i - skipCount] = make_float3(x, y, z);
			normals[i - skipCount] = make_float3(nx, ny, nz);
			colors[i - skipCount] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, 255);
		}
		else if (nop == noc)
		{
			auto r = ply.GetColors()[i * 4];
			auto g = ply.GetColors()[i * 4 + 1];
			auto b = ply.GetColors()[i * 4 + 2];
			auto a = ply.GetColors()[i * 4 + 3];

			positions[i - skipCount] = make_float3(x, y, z);
			normals[i - skipCount] = make_float3(nx, ny, nz);
			colors[i - skipCount] = make_uchar4(r * 255.0f, g * 255.0f, b * 255.0f, a * 255.0f);
		}

		aabb.extend(Eigen::Vector3f(x, y, z));
	}

	positions.resize(nop - skipCount);
	normals.resize(non - skipCount);
	colors.resize(noc - skipCount);

	numberOfElements = nop - skipCount;

	thrust::device_vector<float3> d_positions(positions);
	thrust::device_vector<float3> d_normals(normals);
	thrust::device_vector<uchar4> d_colors(colors);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(d_positions.data()),
		thrust::raw_pointer_cast(d_normals.data()),
		thrust::raw_pointer_cast(d_colors.data()),
		numberOfElements);

	return true;
}

bool HostPointCloud::SaveToPLY(const string& filename)
{
	PLYFormat ply;

	for (size_t i = 0; i < numberOfElements; i++)
	{
		auto& p = positions[i];
		auto& n = normals[i];
		auto& c = colors[i];

		ply.AddPoint(p.x, p.y, p.z);
		ply.AddNormal(n.x, n.y, n.z);
		ply.AddColor((float)c.x / 255.0f, (float)c.y / 255.0f, (float)c.z / 255.0f, (float)c.w / 255.0f);
	}

	ply.Serialize(filename);

	return true;
}

bool HostPointCloud::LoadFromALP(const string& filename)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}

	positions.resize(alp.GetPoints().size());
	normals.resize(alp.GetPoints().size());
	colors.resize(alp.GetPoints().size());

	numberOfElements = alp.GetPoints().size();

	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];

		positions[i] = p.position;
		normals[i] = p.normal;
		colors[i].x = p.color.x * 255.0f;
		colors[i].y = p.color.y * 255.0f;
		colors[i].z = p.color.z * 255.0f;
		colors[i].w = 255;

		aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
	}

	thrust::device_vector<float3> d_positions(positions);
	thrust::device_vector<float3> d_normals(normals);
	thrust::device_vector<uchar4> d_colors(colors);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(d_positions.data()),
		thrust::raw_pointer_cast(d_normals.data()),
		thrust::raw_pointer_cast(d_colors.data()),
		numberOfElements);

	return true;
}

bool HostPointCloud::LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi)
{
	ALPFormat<PointPNC> alp;
	if (false == alp.Deserialize(filename))
	{
		return false;
	}

	positions.resize(alp.GetPoints().size());
	normals.resize(alp.GetPoints().size());
	colors.resize(alp.GetPoints().size());

	size_t skipCount = 0;
	for (size_t i = 0; i < alp.GetPoints().size(); i++)
	{
		auto& p = alp.GetPoints()[i];

		if (false == roi.contains(Eigen::Vector3f(p.position.x, p.position.y, p.position.z)))
		{
			skipCount++;
			continue;
		}

		positions[i - skipCount] = p.position;
		normals[i - skipCount] = p.normal;
		colors[i - skipCount].x = p.color.x * 255.0f;
		colors[i - skipCount].y = p.color.y * 255.0f;
		colors[i - skipCount].z = p.color.z * 255.0f;
		colors[i - skipCount].w = 255;

		aabb.extend(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
	}

	numberOfElements = alp.GetPoints().size() - skipCount;

	thrust::device_vector<float3> d_positions(positions);
	thrust::device_vector<float3> d_normals(normals);
	thrust::device_vector<uchar4> d_colors(colors);

	hashmap.InsertPoints(
		thrust::raw_pointer_cast(d_positions.data()),
		thrust::raw_pointer_cast(d_normals.data()),
		thrust::raw_pointer_cast(d_colors.data()),
		numberOfElements);

	return true;
}

bool HostPointCloud::SaveToALP(const string& filename)
{
	ALPFormat<PointPNC> alp;

	for (size_t i = 0; i < numberOfElements; i++)
	{
		PointPNC point;
		point.position = positions[i];
		point.normal = normals[i];
		point.color.x = (float)colors[i].x / 255.0f;
		point.color.y = (float)colors[i].y / 255.0f;
		point.color.z = (float)colors[i].z / 255.0f;

		alp.AddPoint(point);
	}

	alp.Serialize(filename);

	return true;
}

size_t HostPointCloud::Pick(float3 rayOrigin, float3 rayDirection)
{
	float minDistSq = FLT_MAX;
	int bestIndex = -1;

	for (size_t i = 0; i < numberOfElements; ++i)
	{
		float3 p = positions[i];
		float3 v = p - rayOrigin;
		float t = dot(v, rayDirection);
		if (t < 0) continue;

		float3 closest = rayOrigin + t * rayDirection;
		float3 diff = p - closest;
		float distSq = dot(diff, diff);

		if (distSq < 0.01f * 0.01f && distSq < minDistSq)
		{
			minDistSq = distSq;
			bestIndex = static_cast<int>(i);
		}
	}

	return bestIndex >= 0 ? static_cast<size_t>(bestIndex) : size_t(-1);
}
