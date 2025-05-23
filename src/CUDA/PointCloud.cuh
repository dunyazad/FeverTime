#pragma once

#include <CUDA/cudaCommon.cuh>
#include <CUDA/HashMap.cuh>
#include <CUDA/TSDF.cuh>

class PointCloud
{
public:
	PointCloud();
	~PointCloud();

	void Initialize(unsigned int numberOfPoints);
	void Terminate();

	void HtoD();
	void DtoH();

	bool LoadFromPLY(const string& filename);
	bool LoadFromPLY(const string& filename, const Eigen::AlignedBox3f& roi);

	bool SaveToPLY(const string& filename);
	
	bool LoadFromALP(const string& filename);
	bool LoadFromALP(const string& filename, const Eigen::AlignedBox3f& roi);

	bool SaveToALP(const string& filename);

	void SerializeVoxels(PointCloudBuffers& d_tempBuffers);
	void SerializeVoxelsColoringByLabel(PointCloudBuffers& d_tempBuffers);

	void ComputeVoxelNormalPCA();
	void ComputeVoxelNormalAverage();

	void ComputeNeighborCount();
	void SerializeColoringByNeighborCount(unsigned int countThreshold, PointCloudBuffers& d_tempBuffers);

	void ComputeEmptyNeighborCount();
	void SerializeColoringByEmptyNeighborCount(unsigned int countThreshold, PointCloudBuffers& d_tempBuffers);

	void ComputeNormalDiscontinuity(float normalDiscontinuityThreshold = 10.0f);
	void SerializeColoringByNormalDiscontinuity(PointCloudBuffers& d_tempBuffers);

	void ComputeNormalGradient();
	void SerializeColoringByNormalGradient(float threshold, PointCloudBuffers& d_tempBuffers);

	void ComputeNormalDivergence();
	void SerializeColoringByNormalDivergence(float threshold, PointCloudBuffers& d_tempBuffers);

	void ComputeColorMultiplication();
	void SerializeColoringByColorMultiplication(float threshold, PointCloudBuffers& d_tempBuffers);

	vector<uint3> Clustering(float normalDegreeThreshold = 10.0f);

	void SerializeColoringByLabel(PointCloudBuffers& d_tempBuffers);
	void SerializeColoringBySubLabel(PointCloudBuffers& d_tempBuffers);

	inline unsigned int GetNumberOfPoints() const { return h_buffers.numberOfPoints; }
	inline PointCloudBuffers& GetHostBuffers() { return h_buffers; };
	inline const PointCloudBuffers& GetHostBuffers() const { return h_buffers; };
	inline PointCloudBuffers& GetDeviceBuffers() { return d_buffers; };
	inline const PointCloudBuffers& GetDeviceBuffers() const { return d_buffers; };

private:
	PointCloudBuffers h_buffers;
	PointCloudBuffers d_buffers;

	HashMap hashmap;
};
