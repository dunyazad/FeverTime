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

	inline int GetStep() const { return step; }
	inline void SetStep(int step) { this->step = step; }

	inline bool GetRemoveCheckedPoints() { return removeCheckedPoints; }
	inline void SetRemoveCheckedPoints(bool remove) { removeCheckedPoints = remove; }

private:
	bool removeCheckedPoints = false;
	float angleThreshold = 90.0f;
	int step = 3;
};
