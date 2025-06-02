#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>
#include <CUDA/TSDF.cuh>

class PointCloudAlgorithm_Smoothing : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_Smoothing();
	virtual ~PointCloudAlgorithm_Smoothing();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

	inline bool GetApplyColor() const { return applyColor; }
	inline void SetApplyColor(bool apply) { applyColor = apply; }

private:
	bool applyColor = true;
};
