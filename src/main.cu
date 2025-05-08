#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
namespace Eigen {
    using Vector3b = Eigen::Vector<unsigned char, 3>;
    using Vector3ui = Eigen::Vector<unsigned int, 3>;
}

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPoints.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3DMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkCellArray.h>
#include <vtkPolyVertex.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkProperty.h>



#include <nvtx3/nvToolsExt.h>

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <device_launch_parameters.h>
#include <nvtx3/nvToolsExt.h>
#include "nvapi.h"
#include "NvApiDriverSettings.h"

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include <Serialization.hpp>

#pragma comment(lib, "nvapi64.lib")


struct HashMapVoxel
{
    unsigned int label = 0;
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3b color = Eigen::Vector3b(255, 255, 255);
};

__device__ __host__ inline size_t voxel_hash(int3 coord, size_t tableSize)
{
    return ((size_t)(coord.x * 73856093) ^ (coord.y * 19349663) ^ (coord.z * 83492791)) % tableSize;
}

__global__ void Kernel_InsertPoints(Eigen::Vector3f* points, Eigen::Vector3f* normals, Eigen::Vector3b* colors, int numberOfPoints, float voxelSize, HashMapVoxel* table, size_t tableSize, unsigned int maxProbe)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numberOfPoints) return;

    auto p = points[idx];
    auto n = normals[idx];
    auto c = colors[idx];

    int3 coord = make_int3(floorf(p.x() / voxelSize), floorf(p.y() / voxelSize), floorf(p.z() / voxelSize));

    size_t h = voxel_hash(coord, tableSize);
    for (int i = 0; i < maxProbe; ++i) {
        size_t slot = (h + i) % tableSize;
        if (atomicCAS(&table[slot].label, 0, slot) == 0)
        {
            //alog("%d, %d, %d\n", coord.x, coord.y, coord.z);

            table[slot].label = slot;
            table[slot].position = Eigen::Vector3f((float)coord.x * voxelSize, (float)coord.y * voxelSize, (float)coord.z * voxelSize);
            table[slot].normal = Eigen::Vector3f(n.x(), n.y(), n.z());
            table[slot].color = Eigen::Vector3b(c.x(), c.y(), c.z());
            return;
        }
    }
}

__global__ void Kernel_Serialize(HashMapVoxel* d_table, size_t tableSize,
    Eigen::Vector3f* d_points, Eigen::Vector3f* d_normals, Eigen::Vector3b* d_colors,
    unsigned int* numberOfOccupiedVoxels)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= tableSize) return;

    auto& voxel = d_table[idx];

    if (0 != voxel.label)
    {
        //alog("%f, %f, %f\n", voxel.position.x(), voxel.position.y(), voxel.position.z());

        auto oldIndex = atomicAdd(numberOfOccupiedVoxels, 1);
        d_points[oldIndex] = voxel.position;
        d_normals[oldIndex] = voxel.normal;
        d_colors[oldIndex] = voxel.color;
    }
}

struct HashMap
{
    size_t tableSize = 10485760;
    unsigned int maxProbe = 32;
    unsigned int blockSize = 256;

    HashMapVoxel* d_table = nullptr;

    void Initialize()
    {
        cudaMalloc(&d_table, sizeof(HashMapVoxel) * tableSize);
        cudaMemset(d_table, 0, sizeof(HashMapVoxel) * tableSize);
    }

    void Terminate()
    {
        cudaFree(d_table);
    }

    void InsertHPoints(Eigen::Vector3f* h_points, Eigen::Vector3f* h_normals, Eigen::Vector3b* h_colors, unsigned numberOfPoints)
    {
        Eigen::Vector3f* d_points = nullptr;
        Eigen::Vector3f* d_normals = nullptr;
        Eigen::Vector3b* d_colors = nullptr;

        cudaMalloc(&d_points, sizeof(Eigen::Vector3f) * numberOfPoints);
        cudaMemcpy(d_points, h_points, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);

        cudaMalloc(&d_normals, sizeof(Eigen::Vector3f) * numberOfPoints);
        cudaMemcpy(d_normals, h_normals, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);

        cudaMalloc(&d_colors, sizeof(Eigen::Vector3b) * numberOfPoints);
        cudaMemcpy(d_colors, d_colors, sizeof(Eigen::Vector3b) * numberOfPoints, cudaMemcpyHostToDevice);

        InsertDPoints(d_points, d_normals, d_colors, numberOfPoints);

        cudaFree(d_points);
        cudaFree(d_normals);
        cudaFree(d_colors);
    }

    void InsertDPoints(Eigen::Vector3f* d_points, Eigen::Vector3f* d_normals, Eigen::Vector3b* d_colors, unsigned numberOfPoints)
    {
        unsigned int blockSize = 256;
        unsigned int gridOccupied = (numberOfPoints + blockSize - 1) / blockSize;

        Kernel_InsertPoints << <gridOccupied, blockSize >> > (
            d_points,
            d_normals,
            d_colors,
            numberOfPoints,
            0.1f, d_table, tableSize, maxProbe);

        cudaDeviceSynchronize();
    }

    void Serialize(const std::string& filename)
    {
        PLYFormat ply;

        unsigned int* d_numberOfOccupiedVoxels = nullptr;
        cudaMalloc(&d_numberOfOccupiedVoxels, sizeof(unsigned int));

        Eigen::Vector3f* d_points = nullptr;
        Eigen::Vector3f* d_normals = nullptr;
        Eigen::Vector3b* d_colors = nullptr;

        cudaMalloc(&d_points, sizeof(Eigen::Vector3f) * tableSize);
        cudaMalloc(&d_normals, sizeof(Eigen::Vector3f) * tableSize);
        cudaMalloc(&d_colors, sizeof(Eigen::Vector3b) * tableSize);

        unsigned int blockSize = 256;
        unsigned int gridOccupied = (tableSize + blockSize - 1) / blockSize;

        Kernel_Serialize << <gridOccupied, blockSize >> > (
            d_table,
            tableSize,
            d_points,
            d_normals,
            d_colors,
            d_numberOfOccupiedVoxels);

        cudaDeviceSynchronize();

        unsigned int h_numberOfOccupiedVoxels = 0;
        cudaMemcpy(&h_numberOfOccupiedVoxels, d_numberOfOccupiedVoxels, sizeof(unsigned int), cudaMemcpyDeviceToHost);

        Eigen::Vector3f* h_points = new Eigen::Vector3f[h_numberOfOccupiedVoxels];
        Eigen::Vector3f* h_normals = new Eigen::Vector3f[h_numberOfOccupiedVoxels];
        Eigen::Vector3b* h_colors = new Eigen::Vector3b[h_numberOfOccupiedVoxels];

        cudaMemcpy(h_points, d_points, sizeof(Eigen::Vector3f) * h_numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
        cudaMemcpy(h_normals, d_normals, sizeof(Eigen::Vector3f) * h_numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);
        cudaMemcpy(h_colors, d_colors, sizeof(Eigen::Vector3b) * h_numberOfOccupiedVoxels, cudaMemcpyDeviceToHost);

        for (size_t i = 0; i < h_numberOfOccupiedVoxels; i++)
        {
            auto& p = h_points[i];
            auto& n = h_normals[i];
            auto& c = h_colors[i];

            ply.AddPoint(p.x(), p.y(), p.z());
            ply.AddNormal(n.x(), n.y(), n.z());
            ply.AddColor(c.x() / 255.0f, c.y() / 255.0f, c.z() / 255.0f);
        }

        ply.Serialize(filename);

        cudaFree(d_numberOfOccupiedVoxels);
        cudaFree(d_points);
        cudaFree(d_normals);
        cudaFree(d_colors);

        delete[] h_points;
        delete[] h_normals;
        delete[] h_colors;
    }
};

struct PointCloud
{
    Eigen::Vector3f* d_points = nullptr;
    Eigen::Vector3f* d_normals = nullptr;
    Eigen::Vector3b* d_colors = nullptr;
    unsigned int numberOfPoints = 0;
};

PointCloud pointCloud;


struct PointPNC
{
    float3 position;
    float3 normal;
    float3 color;
};

ALPFormat<PointPNC> alp;

const string resource_file_name = "Compound";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

int main(int argc, char** argv)
{
#pragma region Load ALP File
    std::vector<float3> host_points;
    std::vector<float3> host_normals;
    std::vector<uchar3> host_colors;

    if (false == alp.Deserialize(resource_file_name_alp))
    {
        bool foundZero = false;

        PLYFormat ply;
        ply.Deserialize(resource_file_name_ply);
        //ply.SwapAxisYZ();

        std::vector<PointPNC> points;
        for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
        {
            auto px = ply.GetPoints()[i * 3];
            auto py = ply.GetPoints()[i * 3 + 1];
            auto pz = ply.GetPoints()[i * 3 + 2];

            if (0 == px && 0 == py && 0 == pz)
            {
                if (false == foundZero)
                {
                    foundZero = true;
                }
                else
                {
                    continue;
                }
            }

            auto nx = ply.GetNormals()[i * 3];
            auto ny = ply.GetNormals()[i * 3 + 1];
            auto nz = ply.GetNormals()[i * 3 + 2];

            if (false == ply.GetColors().empty())
            {
                if (ply.UseAlpha())
                {
                    auto cx = ply.GetColors()[i * 4];
                    auto cy = ply.GetColors()[i * 4 + 1];
                    auto cz = ply.GetColors()[i * 4 + 2];
                    auto ca = ply.GetColors()[i * 4 + 3];

                    points.push_back({ {px, py, pz}, {nx, ny, nz}, {cx, cy, cz} });
                }
                else
                {
                    auto cx = ply.GetColors()[i * 3];
                    auto cy = ply.GetColors()[i * 3 + 1];
                    auto cz = ply.GetColors()[i * 3 + 2];

                    points.push_back({ {px, py, pz}, {nx, ny, nz}, {cx, cy, cz} });
                }
            }
            else
            {
                points.push_back({ {px, py, pz}, {nx, ny, nz}, {1.0f, 1.0f, 1.0f} });
            }
        }
        alog("PLY %llu points loaded\n", points.size());

        alp.AddPoints(points);
        alp.Serialize(resource_file_name_alp);
    }

    for (auto& p : alp.GetPoints())
    {
        auto r = p.color.x;
        auto g = p.color.y;
        auto b = p.color.z;
        auto a = 1.f;

        host_points.push_back(make_float3(p.position.x, p.position.y, p.position.z));
        host_normals.push_back(make_float3(p.normal.x, p.normal.y, p.normal.z));
        host_colors.push_back(make_uchar3(r * 255, g * 255, b * 255));
    }

    alog("ALP %llu points loaded\n", alp.GetPoints().size());
#pragma endregion



    pointCloud.numberOfPoints = host_points.size();

    cudaMalloc(&pointCloud.d_points, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints);
    cudaMalloc(&pointCloud.d_normals, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints);
    cudaMalloc(&pointCloud.d_colors, sizeof(Eigen::Vector3b) * pointCloud.numberOfPoints);

    cudaMemcpy(pointCloud.d_points, host_points.data(), sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);
    cudaMemcpy(pointCloud.d_normals, host_normals.data(), sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);
    cudaMemcpy(pointCloud.d_colors, host_colors.data(), sizeof(Eigen::Vector3b) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);

#pragma region Hashmap
    /*
    HashMap hm;
    hm.Initialize();

    hm.InsertDPoints(pointCloud.d_points, pointCloud.d_normals, pointCloud.d_colors, pointCloud.numberOfPoints);

    hm.Serialize("../../res/3D/Voxels.ply");

    hm.Terminate();
    */
#pragma endregion




    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();

    colors->SetNumberOfComponents(4); // RGBA
    colors->SetName("Colors");

    normals->SetNumberOfComponents(3);
    normals->SetName("Normals");

    for (size_t i = 0; i < host_points.size(); i++)
    {
        auto& p = host_points[i];
        auto& n = host_normals[i];
        auto& c = host_colors[i];

        points->InsertNextPoint(p.x, p.y, p.z);
        normals->InsertNextTuple3(n.x, n.y, n.z);
        unsigned char color[4] = { c.x, c.y, c.z, 255 };
        colors->InsertNextTypedTuple(color);
    }

    auto vertices = vtkSmartPointer<vtkCellArray>::New();
    for (vtkIdType i = 0; i < host_points.size(); ++i)
    {
        vtkIdType pid = i;
        vertices->InsertNextCell(1, &pid);  // 개별 점 처리
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);
    polyData->GetPointData()->SetScalars(colors);
    polyData->GetPointData()->SetNormals(normals);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->SetScalarVisibility(true);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2); // 점 크기 지정
    actor->GetProperty()->SetRepresentationToPoints(); // 꼭 필요!

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor->SetInteractorStyle(style);


    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.1, 0.2);

    renderWindow->SetSize(800, 600);
    renderWindow->Render();
    interactor->Start();

    return EXIT_SUCCESS;
}
