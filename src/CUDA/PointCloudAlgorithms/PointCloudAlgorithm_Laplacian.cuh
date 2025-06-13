#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>

class PointCloudAlgorithm_Laplacian : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_Laplacian();
	virtual ~PointCloudAlgorithm_Laplacian();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

private:
};
