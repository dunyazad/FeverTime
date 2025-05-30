#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>
#include <CUDA/TSDF.cuh>

class PointCloudAlgorithm_CheckOverlap : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_CheckOverlap();
	virtual ~PointCloudAlgorithm_CheckOverlap();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

private:
	float angleThreshold = 9.0f;
};
