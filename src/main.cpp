#include "App.h"

#include <main.cuh>

PointCloud pointCloud;

ALPFormat<PointPNC> alp;

const string resource_file_name = "Compound";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

int main(int argc, char** argv)
{
    App app;
    app.SetInitializeCallback([](App& app) {
        app.GetRenderer()->SetBackground(0.3, 0.5, 0.7);

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

#pragma region Hashmap
        /*
        pointCloud.numberOfPoints = host_points.size();

        cudaMalloc(&pointCloud.d_points, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints);
        cudaMalloc(&pointCloud.d_normals, sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints);
        cudaMalloc(&pointCloud.d_colors, sizeof(Eigen::Vector3b) * pointCloud.numberOfPoints);

        cudaMemcpy(pointCloud.d_points, host_points.data(), sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);
        cudaMemcpy(pointCloud.d_normals, host_normals.data(), sizeof(Eigen::Vector3f) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);
        cudaMemcpy(pointCloud.d_colors, host_colors.data(), sizeof(Eigen::Vector3b) * pointCloud.numberOfPoints, cudaMemcpyHostToDevice);

        HashMap hm;
        hm.Initialize();

        hm.InsertDPoints(pointCloud.d_points, pointCloud.d_normals, pointCloud.d_colors, pointCloud.numberOfPoints);

        hm.Serialize("../../res/3D/Voxels.ply");

        hm.Terminate();

        cudaFree(pointCloud.d_points);
        cudaFree(pointCloud.d_normals);
        cudaFree(pointCloud.d_colors);
        */
#pragma endregion

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        normals->SetName("Normals");

        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(4); // RGBA
        colors->SetName("Colors");

        vtkSmartPointer<vtkPoints> normalLinesPoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> normalLines = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkUnsignedCharArray> normalColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        normalColors->SetNumberOfComponents(4); // RGBA
        normalColors->SetName("Colors");


        for (size_t i = 0; i < host_points.size(); i++)
        {
            auto& p = host_points[i];
            auto& n = host_normals[i];
            auto& c = host_colors[i];

            points->InsertNextPoint(p.x, p.y, p.z);
            normals->InsertNextTuple3(n.x, n.y, n.z);
            unsigned char color[4] = { c.x, c.y, c.z, 255 };
            colors->InsertNextTypedTuple(color);

            double startPoint[3] = { p.x, p.y, p.z };
            double endPoint[3] = {
                p.x + n.x * 0.1,  // 길이 조절 (0.02)
                p.y + n.y * 0.1,
                p.z + n.z * 0.1
            };

            vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
            vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, idStart);
            line->GetPointIds()->SetId(1, idEnd);

            normalLines->InsertNextCell(line);

            unsigned char normalColor[4] = {
                static_cast<unsigned char>((n.x * 0.5f + 0.5f) * 255),
                static_cast<unsigned char>((n.y * 0.5f + 0.5f) * 255),
                static_cast<unsigned char>((n.z * 0.5f + 0.5f) * 255),
                255
            };
            normalColors->InsertNextTypedTuple(normalColor);
            normalColors->InsertNextTypedTuple(normalColor); // start, end 같은 색상
        }

        {
            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < host_points.size(); ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);
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
            actor->GetProperty()->SetPointSize(2);
            actor->GetProperty()->SetRepresentationToPoints();

            app.GetRenderer()->AddActor(actor);
        }

        {
            auto normalPolyData = vtkSmartPointer<vtkPolyData>::New();
            normalPolyData->SetPoints(normalLinesPoints);
            normalPolyData->SetLines(normalLines);
            normalPolyData->GetPointData()->SetScalars(normalColors);

            auto normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            normalMapper->SetInputData(normalPolyData);

            auto normalActor = vtkSmartPointer<vtkActor>::New();
            normalActor->SetMapper(normalMapper);
            normalMapper->SetScalarModeToUsePointData();
            normalMapper->SetColorModeToDirectScalars();
            normalMapper->SetScalarVisibility(true);
            normalActor->GetProperty()->SetLineWidth(1.5);       // 선 두께

            app.GetRenderer()->AddActor(normalActor);
        }

        {
            auto picker = vtkSmartPointer<vtkPointPicker>::New();
            app.GetInteractor()->SetPicker(picker);

            auto doubleClickPickerCallback = vtkSmartPointer<DoubleClickPickerCallback>::New();
            doubleClickPickerCallback->SetApp(&app);

            app.GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, doubleClickPickerCallback);
        }

        {
            auto keyCallback = vtkSmartPointer<KeyPressCallback>::New();
            keyCallback->SetApp(&app);
            app.GetInteractor()->AddObserver(vtkCommand::KeyPressEvent, keyCallback);
        }
    });

    app.Initialize();
    app.Run();

    return EXIT_SUCCESS;
}
