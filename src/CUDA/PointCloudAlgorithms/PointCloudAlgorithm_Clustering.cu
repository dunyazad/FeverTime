#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Clustering.cuh>

//#include <CUDA/PointCloud.cuh>
//
//PointCloudAlgorithm_Clustering::PointCloudAlgorithm_Clustering()
//{
//
//}
//
//PointCloudAlgorithm_Clustering::~PointCloudAlgorithm_Clustering()
//{
//
//}
//
//void PointCloudAlgorithm_Clustering::RunAlgorithm(PointCloud* pointCloud)
//{
//	//unsigned int numberOfOccupiedVoxels = pointCloud->GetHashMap().info.h_numberOfOccupiedVoxels;
//
//	//unsigned int blockSize = 256;
//	//unsigned int gridOccupied = (numberOfOccupiedVoxels + blockSize - 1) / blockSize;
//
//	//Kernel_Prepare_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info);
//
//	//Kernel_CheckOverlap << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, angleThreshold);
//
//	//gridOccupied = (pointCloud->GetNumberOfPoints() + blockSize - 1) / blockSize;
//
//	//Kernel_Serialize_CheckOverlapResult << <gridOccupied, blockSize >> > (pointCloud->GetHashMap().info, pointCloud->GetDeviceBuffers());
//
//	//cudaDeviceSynchronize();
//}
//
//void PointCloudAlgorithm_Clustering::IncreaseParameter()
//{
//	angleThreshold += 1.0f;
//}
//
//void PointCloudAlgorithm_Clustering::DecreaseParameter()
//{
//	angleThreshold -= 1.0f;
//}
