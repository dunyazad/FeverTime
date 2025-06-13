#pragma once

#include <Common.h>

Eigen::Vector3f Transform(const Eigen::Matrix4f& tm, const Eigen::Vector3f& p)
{
	return (tm * Eigen::Vector4f(p.x(), p.y(), p.z(), 1.0f)).head<3>();
}

Eigen::Matrix4f vtkToEigen(const vtkMatrix4x4* vtkMat)
{
	Eigen::Matrix4f eigenMat;

	// VTK is row-major, Eigen is column-major by default
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			eigenMat(i, j) = vtkMat->GetElement(i, j);
		}
	}

	return eigenMat;
}

vtkSmartPointer<vtkMatrix4x4> eigenToVtk(const Eigen::Matrix4f& eigenMat)
{
	vtkSmartPointer<vtkMatrix4x4> vtkMat = vtkSmartPointer<vtkMatrix4x4>::New();

	// Eigen is column-major, VTK is row-major
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			vtkMat->SetElement(i, j, eigenMat(i, j));
		}
	}

	return vtkMat;
}
