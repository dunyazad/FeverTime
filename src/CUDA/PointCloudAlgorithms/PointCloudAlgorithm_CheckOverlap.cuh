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

	inline bool GetApplyColor() const { return applyColor; }
	inline void SetApplyColor(bool apply) { applyColor = apply; }

	inline int GetStep() const { return step; }
	inline void SetStep(int step) { this->step = step; }

private:
	float angleThreshold = 9.0f;
	bool applyColor = true;
	int step = 3;
};
