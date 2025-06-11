#pragma once

#include <CUDA/cudaCommon.cuh>
#include <CUDA/HashMap.cuh>

using TupleType = thrust::tuple<float3, float3, uchar4>;

using DeviceZipIter = thrust::zip_iterator<
	thrust::tuple<
	thrust::device_vector<float3>::iterator,
	thrust::device_vector<float3>::iterator,
	thrust::device_vector<uchar4>::iterator>>;

using HostZipIter = thrust::zip_iterator<
	thrust::tuple<
	thrust::host_vector<float3>::iterator,
	thrust::host_vector<float3>::iterator,
	thrust::host_vector<uchar4>::iterator>>;

class DevicePointCloud;
class HostPointCloud;

class IPointCloud
{
public:
	IPointCloud() = default;
	virtual ~IPointCloud() = default;

	IPointCloud(const IPointCloud&) = delete;
	IPointCloud& operator=(const IPointCloud&) = delete;

	virtual void Compact() = 0;

	virtual bool LoadFromPLY(const string& filename) = 0;
	virtual bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi) = 0;
	virtual bool SaveToPLY(const string& filename) = 0;
	virtual bool LoadFromALP(const string& filename) = 0;
	virtual bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi) = 0;
	virtual bool SaveToALP(const string& filename) = 0;

	virtual size_t Pick(float3 rayOrigin, float3 rayDirection) = 0;

	inline Eigen::AlignedBox3f& GetAABB() { return aabb; }
	inline const Eigen::AlignedBox3f& GetAABB() const { return aabb; }

	inline size_t GetNumberOfElements() { return numberOfElements; }

	inline HashMap& GetHashMap() { return hashmap; }
	inline const HashMap& GetHashMap() const { return hashmap; }

protected:
	size_t numberOfElements = 0;

	Eigen::AlignedBox3f aabb =
		Eigen::AlignedBox3f(
			Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
			Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

	HashMap hashmap;
};

class DevicePointCloud : public IPointCloud
{
public:
	void Clear(size_t numberOfPoints = 0);

	void CopyFrom(DevicePointCloud* pointCloud);
	void CopyFrom(HostPointCloud* pointCloud);

	void CopyTo(DevicePointCloud* pointCloud);
	void CopyTo(HostPointCloud* pointCloud);

	virtual void Compact();

	virtual bool LoadFromPLY(const string& filename);

	virtual bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToPLY(const string& filename);

	virtual bool LoadFromALP(const string& filename);

	virtual bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToALP(const string& filename);

	virtual size_t Pick(float3 rayOrigin, float3 rayDirection);

	inline thrust::device_vector<float3>& GetPositions() { return positions; }
	inline const thrust::device_vector<float3>& GetPositions() const { return positions; }

	inline thrust::device_vector<float3>& GetNormals() { return normals; }
	inline const thrust::device_vector<float3>& GetNormals() const { return normals; }

	inline thrust::device_vector<uchar4>& GetColors() { return colors; }
	inline const thrust::device_vector<uchar4>& GetColors() const { return colors; }

	inline DeviceZipIter begin()
	{
		return thrust::make_zip_iterator(thrust::make_tuple(
			positions.begin(), normals.begin(), colors.begin()));
	}

	inline DeviceZipIter end()
	{
		return thrust::make_zip_iterator(thrust::make_tuple(
			positions.end(), normals.end(), colors.end()));
	}

protected:
	thrust::device_vector<float3> positions;
	thrust::device_vector<float3> normals;
	thrust::device_vector<uchar4> colors;

public:
	friend class HostPointCloud;
};

class HostPointCloud : public IPointCloud
{
public:
	void Clear(size_t numberOfPoints = 0);

	void CopyFrom(DevicePointCloud* pointCloud);
	void CopyFrom(HostPointCloud* pointCloud);

	void CopyTo(DevicePointCloud* pointCloud);
	void CopyTo(HostPointCloud* pointCloud);

	virtual void Compact();

	virtual bool LoadFromPLY(const string& filename);

	virtual bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToPLY(const string& filename);

	virtual bool LoadFromALP(const string& filename);

	virtual bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToALP(const string& filename);

	virtual size_t Pick(float3 rayOrigin, float3 rayDirection);

	inline thrust::host_vector<float3>& GetPositions() { return positions; }
	inline const thrust::host_vector<float3>& GetPositions() const { return positions; }

	inline thrust::host_vector<float3>& GetNormals() { return normals; }
	inline const thrust::host_vector<float3>& GetNormals() const { return normals; }

	inline thrust::host_vector<uchar4>& GetColors() { return colors; }
	inline const thrust::host_vector<uchar4>& GetColors() const { return colors; }

	inline HostZipIter begin()
	{
		return thrust::make_zip_iterator(thrust::make_tuple(
			positions.begin(), normals.begin(), colors.begin()));
	}

	inline HostZipIter end()
	{
		return thrust::make_zip_iterator(thrust::make_tuple(
			positions.end(), normals.end(), colors.end()));
	}

protected:
	thrust::host_vector<float3> positions;
	thrust::host_vector<float3> normals;
	thrust::host_vector<uchar4> colors;

public:
	friend class DevicePointCloud;
};
