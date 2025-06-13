#pragma once

#include <CUDA/PointCloudAlgorithms/PointCloud_Algorithm.cuh>

class PointCloudAlgorithm_NormalSimilarity : public PointCloudAlgorithm
{
public:
	PointCloudAlgorithm_NormalSimilarity();
	virtual ~PointCloudAlgorithm_NormalSimilarity();

	virtual void RunAlgorithm(DevicePointCloud* pointCloud);
	virtual void RunAlgorithm(HostPointCloud* pointCloud);

	virtual void IncreaseParameter() override;
	virtual void DecreaseParameter() override;

	inline bool GetRemoveCheckedPoints() { return removeCheckedPoints; }
	inline void SetRemoveCheckedPoints(bool remove) { removeCheckedPoints = remove; }

private:
	bool removeCheckedPoints = false;
};
