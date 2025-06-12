#pragma once

#include <Common.h>

//#include <CUDA/PointCloud.cuh>
#include <CUDA/PCD.cuh>

class Entity
{
public:
	Entity(vtkSmartPointer<vtkRenderer> renderer, const string& name);
	~Entity();

	void Clear();

	//void FromPointCloudBuffers(PointCloudBuffers* buffers);
	void FromPointCloud(DevicePointCloud* pointCloud);
	void FromPointCloud(DevicePointCloud* pointCloud, const Eigen::AlignedBox3f& roi);
	void FromPointCloud(HostPointCloud* pointCloud);
	void FromPointCloud(HostPointCloud* pointCloud, const Eigen::AlignedBox3f& roi);
	//void FromPointCloud(DevicePointCloud* pointCloud);
	//void FromPointCloud(DevicePointCloud* pointCloud, const Eigen::AlignedBox3f& roi);
	//void FromPointCloud(HostPointCloud* pointCloud);
	//void FromPointCloud(HostPointCloud* pointCloud, const Eigen::AlignedBox3f& roi);
	void CopyFrom(Entity* other);
	void CopyFrom(Entity* other, const Eigen::AlignedBox3f& roi);
	void CopyTo(Entity* other);
	void CopyTo(Entity* other, const Eigen::AlignedBox3f& roi);

	//void UpdateColorFromBuffer(const PointCloudBuffers& buffer);
	void UpdateColorFromBuffer(DevicePointCloud* pointCloud);
	void UpdateColorFromBuffer(HostPointCloud* pointCloud);

	inline vtkSmartPointer<vtkPolyData> GetPolyData() const { return polyData; }
	inline vtkSmartPointer<vtkPolyDataMapper> GetMapper() const { return mapper; }
	inline vtkSmartPointer<vtkActor> GetActor() const { return actor; }

	void SetVisibility(bool visible);
	void ToggleVisibility();

	void SetNormalVisibility(bool visible);
	void ToggleNormalVisibility();

	void SetLighting(bool lighting);
	void ToggleLighting();

	void SetPointSize(unsigned int pointSize);
	void IncreasePointSize();
	void DecreasePointSize();

	inline const string& GetName() const { return name; }
	inline void SetName(const string& name) { this->name = name; }

private:
	string name;

	Eigen::AlignedBox3f roi = Eigen::AlignedBox3f(
		Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX),
		Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX));

	vtkSmartPointer<vtkRenderer> renderer = nullptr;

	vtkSmartPointer<vtkAssembly> assembly = vtkSmartPointer<vtkAssembly>::New();

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

	vtkSmartPointer<vtkPolyData> normalPolyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> normalActor = vtkSmartPointer<vtkActor>::New();
};
