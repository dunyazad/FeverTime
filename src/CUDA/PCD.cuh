#pragma once

#include <CUDA/cudaCommon.cuh>

class DevicePointCloud;
class HostPointCloud;

class IPointCloud
{
public:
	virtual bool LoadFromPLY(const string& filename) = 0;
	virtual bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi) = 0;
	virtual bool SaveToPLY(const string& filename) = 0;
	virtual bool LoadFromALP(const string& filename) = 0;
	virtual bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi) = 0;
	virtual bool SaveToALP(const string& filename) = 0;

	inline Eigen::AlignedBox3f& GetAABB() { return aabb; }
	inline const Eigen::AlignedBox3f& GetAABB() const { return aabb; }

protected:
	Eigen::AlignedBox3f aabb =
		Eigen::AlignedBox3f(
			Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX),
			Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
};

class DevicePointCloud : public IPointCloud
{
public:
	void Clear(size_t numberOfPoints = 0);

	void CopyFrom(HostPointCloud* pointCloud);

	void CopyTo(HostPointCloud* pointCloud);

	virtual bool LoadFromPLY(const string& filename);

	virtual bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToPLY(const string& filename);

	virtual bool LoadFromALP(const string& filename);

	virtual bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToALP(const string& filename);

	inline thrust::device_vector<float3>& GetPoints() { return points; }
	inline const thrust::device_vector<float3>& GetPoints() const { return points; }

	inline thrust::device_vector<float3>& GetNormals() { return normals; }
	inline const thrust::device_vector<float3>& GetNormals() const { return normals; }

	inline thrust::device_vector<uchar4>& GetColors() { return colors; }
	inline const thrust::device_vector<uchar4>& GetColors() const { return colors; }

protected:
	thrust::device_vector<float3> points;
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

	void CopyTo(DevicePointCloud* pointCloud);

	virtual bool LoadFromPLY(const string& filename);

	virtual bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToPLY(const string& filename);

	virtual bool LoadFromALP(const string& filename);

	virtual bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi);

	virtual bool SaveToALP(const string& filename);

	inline thrust::host_vector<float3>& GetPoints() { return points; }
	inline const thrust::host_vector<float3>& GetPoints() const { return points; }

	inline thrust::host_vector<float3>& GetNormals() { return normals; }
	inline const thrust::host_vector<float3>& GetNormals() const { return normals; }

	inline thrust::host_vector<uchar4>& GetColors() { return colors; }
	inline const thrust::host_vector<uchar4>& GetColors() const { return colors; }

protected:
	thrust::host_vector<float3> points;
	thrust::host_vector<float3> normals;
	thrust::host_vector<uchar4> colors;

public:
	friend class DevicePointCloud;
};
