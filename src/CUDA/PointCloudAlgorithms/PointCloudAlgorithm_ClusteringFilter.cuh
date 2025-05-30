#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>

class PointCloudAlgorithm_ClusteringFilter : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_ClusteringFilter();
	virtual ~PointCloudAlgorithm_ClusteringFilter();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

private:
	float angleThreshold = 25.0f;
};
