#pragma once

class DevicePointCloud;
class HostPointCloud;

class PointCloudAlgorithm
{
public:
	PointCloudAlgorithm() = default;
	virtual ~PointCloudAlgorithm() = default;

	virtual void RunAlgorithm(DevicePointCloud* pointCloud) = 0;
	virtual void RunAlgorithm(HostPointCloud* pointCloud) = 0;

	virtual void IncreaseParameter() = 0;
	virtual void DecreaseParameter() = 0;
private:
};
