#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_ComputeTSDF.cuh>
#include <CUDA/PointCloud.cuh>

PointCloudAlgorithm_ComputeTSDF::PointCloudAlgorithm_ComputeTSDF()
{

}

PointCloudAlgorithm_ComputeTSDF::~PointCloudAlgorithm_ComputeTSDF()
{

}

void PointCloudAlgorithm_ComputeTSDF::RunAlgorithm(PointCloud* pointCloud)
{
	tsdf.Initialize(pointCloud->GetNumberOfPoints());

	tsdf.InsertPoints(pointCloud->GetDeviceBuffers());

	tsdf.SerializeToPLY("C:\\Debug\\PLY\\TSDF.ply");

	tsdf.Terminate();
}

void PointCloudAlgorithm_ComputeTSDF::IncreaseParameter()
{

}

void PointCloudAlgorithm_ComputeTSDF::DecreaseParameter()
{

}
