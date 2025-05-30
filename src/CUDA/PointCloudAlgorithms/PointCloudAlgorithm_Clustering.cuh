#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>

class PointCloudAlgorithm_Clustering : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_Clustering();
	virtual ~PointCloudAlgorithm_Clustering();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

private:
	float angleThreshold = 25.0f;
};
