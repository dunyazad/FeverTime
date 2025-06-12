#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>

class PointCloudAlgorithm_FindSurfaceNeighbor : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_FindSurfaceNeighbor();
	virtual ~PointCloudAlgorithm_FindSurfaceNeighbor();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

private:
};
