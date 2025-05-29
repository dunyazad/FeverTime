#include <CUDA/PointCloudBuffers.cuh>

//void PointCloudBuffers::Initialize(unsigned int numberOfPoints, bool isHostBuffer)
//{
//	this->isHostBuffer = isHostBuffer;
//	this->numberOfPoints = numberOfPoints;
//
//	if (isHostBuffer)
//	{
//		positions = new Eigen::Vector3f[numberOfPoints];
//		normals = new Eigen::Vector3f[numberOfPoints];
//		colors = new Eigen::Vector4b[numberOfPoints];
//	}
//	else
//	{
//		cudaMalloc(&positions, sizeof(Eigen::Vector3f) * numberOfPoints);
//		cudaMalloc(&normals, sizeof(Eigen::Vector3f) * numberOfPoints);
//		cudaMalloc(&colors, sizeof(Eigen::Vector4b) * numberOfPoints);
//	}
//}
//
//void PointCloudBuffers::Terminate()
//{
//	if (isHostBuffer)
//	{
//		if (0 < numberOfPoints)
//		{
//			delete[] positions;
//			delete[] normals;
//			delete[] colors;
//		}
//	}
//	else
//	{
//		if (0 < numberOfPoints)
//		{
//			cudaFree(positions);
//			cudaFree(normals);
//			cudaFree(colors);
//		}
//	}
//}
//
//void PointCloudBuffers::CopyTo(PointCloudBuffers& other)
//{
//	if (isHostBuffer && other.isHostBuffer)
//	{
//		other.numberOfPoints = numberOfPoints;
//		other.aabb = aabb;
//		memcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints);
//		memcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints);
//		memcpy(other.colors, colors, sizeof(Eigen::Vector4b) * numberOfPoints);
//	}
//	else if (false == isHostBuffer && other.isHostBuffer)
//	{
//		other.numberOfPoints = numberOfPoints;
//		other.aabb = aabb;
//		cudaMemcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToHost);
//		cudaMemcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToHost);
//		cudaMemcpy(other.colors, colors, sizeof(Eigen::Vector4b) * numberOfPoints, cudaMemcpyDeviceToHost);
//	}
//	else if (isHostBuffer && false == other.isHostBuffer)
//	{
//		other.numberOfPoints = numberOfPoints;
//		other.aabb = aabb;
//		cudaMemcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);
//		cudaMemcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);
//		cudaMemcpy(other.colors, colors, sizeof(Eigen::Vector4b) * numberOfPoints, cudaMemcpyHostToDevice);
//	}
//	else if (false == isHostBuffer && false == other.isHostBuffer)
//	{
//		other.numberOfPoints = numberOfPoints;
//		other.aabb = aabb;
//		cudaMemcpy(other.positions, positions, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToDevice);
//		cudaMemcpy(other.normals, normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyDeviceToDevice);
//		cudaMemcpy(other.colors, colors, sizeof(Eigen::Vector4b) * numberOfPoints, cudaMemcpyDeviceToDevice);
//	}
//}
