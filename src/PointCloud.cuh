#pragma once

#include <stdHeaderFiles.h>
#include <cudaHeaderFiles.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace Eigen
{
    using Vector3b = Eigen::Vector<unsigned char, 3>;
    using Vector3ui = Eigen::Vector<unsigned int, 3>;
}

#include <HashMap.cuh>

class PointCloud
{
public:
	PointCloud();
	~PointCloud();

	void Initialize(unsigned int numberOfPoints);
	void Terminate();

	void HtoD();
	void DtoH();

	bool LoadFromPLY(const std::string& filename);
	bool LoadFromALP(const std::string& filename);

	void ComputeNeighborCount();

	void SerializeColoringByNeighborCount(PointCloudBuffers& d_tempBuffers);

	void ComputeNormalGradient();

	void SerializeColoringByNormalGradient(float threshold, PointCloudBuffers& d_tempBuffers);

	std::vector<unsigned int> Clustering();

	void SerializeColoringByLabel(PointCloudBuffers& d_tempBuffers);

	void SplitByNormal(PointCloudBuffers& d_tempBuffers);

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
