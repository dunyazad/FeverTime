#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>
#include <CUDA/TSDF.cuh>

class PointCloudAlgorithm_ComputeTSDF : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_ComputeTSDF();
	virtual ~PointCloudAlgorithm_ComputeTSDF();

	virtual void RunAlgorithm(PointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

private:
	TSDF tsdf;
};
