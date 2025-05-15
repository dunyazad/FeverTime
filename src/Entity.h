#pragma once

#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>

#include <PointCloud.cuh>

class Entity
{
public:
	Entity(vtkSmartPointer<vtkRenderer> renderer);
	~Entity();

	void Clear();

	void FromPointCloudBuffers(PointCloudBuffers* buffers);
	void FromPointCloud(PointCloud* pointCloud);
	void FromPointCloud(PointCloud* pointCloud, const Eigen::AlignedBox3f& roi);
	void CopyFrom(Entity* other);
	void CopyFrom(Entity* other, const Eigen::AlignedBox3f& roi);
	void CopyTo(Entity* other);
	void CopyTo(Entity* other, const Eigen::AlignedBox3f& roi);

	void UpdateColorFromBuffer(const PointCloudBuffers& buffer);

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

private:
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
