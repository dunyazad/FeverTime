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

	pointCloud->aabb = aabb;

	hashmap.Clear(numberOfElements * 20);
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
	thrust::copy(colors.begin(),  colors.begin() + numberOfElements, pointCloud->colors.begin());

	aabb = pointCloud->aabb;

	hashmap.Clear(numberOfElements * 20);
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
	
	for (size_t i = 0; i < h_positions.size(); i++)
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
	
	for (size_t i = 0; i < h_positions.size(); i++)
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
	positions.resize(pointCloud->positions.size());
	thrust::copy(pointCloud->positions.begin(), pointCloud->positions.end(), positions.begin());

	normals.resize(pointCloud->normals.size());
	thrust::copy(pointCloud->normals.begin(), pointCloud->normals.end(), normals.begin());

	colors.resize(pointCloud->colors.size());
	thrust::copy(pointCloud->colors.begin(), pointCloud->colors.end(), colors.begin());

	pointCloud->aabb = aabb;
}

void HostPointCloud::CopyTo(DevicePointCloud* pointCloud)
{
	pointCloud->positions.resize(positions.size());
	thrust::copy(positions.begin(), positions.end(), pointCloud->positions.begin());

	pointCloud->normals.resize(normals.size());
	thrust::copy(normals.begin(), normals.end(), pointCloud->normals.begin());

	pointCloud->colors.resize(colors.size());
	thrust::copy(colors.begin(), colors.end(), pointCloud->colors.begin());

	aabb = pointCloud->aabb;
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

	for (size_t i = 0; i < positions.size(); i++)
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

	for (size_t i = 0; i < positions.size(); i++)
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
