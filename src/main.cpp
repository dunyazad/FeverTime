#include "App.h"

#include <main.cuh>

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

        Host_PointCloud pointCloud;
        pointCloud.Initialize(alp.GetPoints().size());

        for (size_t i = 0; i < alp.GetPoints().size(); i++)
        {
            auto& p = alp.GetPoints()[i];

            auto r = p.color.x;
            auto g = p.color.y;
            auto b = p.color.z;

            pointCloud.points[i] = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
            pointCloud.normals[i] = Eigen::Vector3f(p.normal.x, p.normal.y, p.normal.z);
            pointCloud.colors[i] = Eigen::Vector3b(r * 255, g * 255, b * 255);
        }

        alog("ALP %llu points loaded\n", alp.GetPoints().size());
#pragma endregion

#pragma region Hashmap
        HashMap hm;
        hm.Initialize();

        hm.InsertHPoints(pointCloud);

        hm.ComputeNormalDivergence();

        auto labels = hm.Clustering(pointCloud);

        //hm.SerializeToPLY("../../res/3D/Voxels.ply");
        //hm.SerializeColoringByLabel("../../res/3D/VoxelsColoringByLabel.ply");
        //hm.SerializeColoringByDivergence("../../res/3D/VoxelsColoringByDivergence.ply");

        hm.Terminate();
#pragma endregion

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        normals->SetName("Normals");

        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(4);
        colors->SetName("Colors");

        vtkSmartPointer<vtkPoints> normalLinesPoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> normalLines = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkUnsignedCharArray> normalColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        normalColors->SetNumberOfComponents(4);
        normalColors->SetName("Colors");

        for (unsigned int i = 0; i < pointCloud.numberOfPoints; i++)
        {
            auto& p = pointCloud.points[i];
            auto& n = pointCloud.normals[i];
            auto& c = pointCloud.colors[i];

            points->InsertNextPoint(p.x(), p.y(), p.z());
            normals->InsertNextTuple3(n.x(), n.y(), n.z());

            auto hashToFloat = [](uint32_t seed) -> float {
                seed ^= seed >> 13;
                seed *= 0x5bd1e995;
                seed ^= seed >> 15;
                return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
            };

            unsigned char color[4];

            //color[0] = c.x();
            //color[1] = c.y();
            //color[2] = c.z();
            //color[3] = 255;
            //colors->InsertNextTypedTuple(color);

            auto label = labels[i];
            if (0 != label)
            {
                float r = hashToFloat(label * 3 + 0);
                float g = hashToFloat(label * 3 + 1);
                float b = hashToFloat(label * 3 + 2);

                //alog("label : %d\n", label);

                //unsigned char color[4] = { c.x(), c.y(), c.z(), 255 };
                color[0] = (unsigned char)(r * 255.0f);
                color[1] = (unsigned char)(g * 255.0f);
                color[2] = (unsigned char)(b * 255.0f);
                color[3] = 255;
                colors->InsertNextTypedTuple(color);
            }
            else
            {
                color[0] = c.x();
                color[1] = c.y();
                color[2] = c.z();
                color[3] = 255;
                colors->InsertNextTypedTuple(color);
            }

            double startPoint[3] = { p.x(), p.y(), p.z() };
            double endPoint[3] = {
                p.x() + n.x() * 0.1,
                p.y() + n.y() * 0.1,
                p.z() + n.z() * 0.1
            };

            vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
            vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, idStart);
            line->GetPointIds()->SetId(1, idEnd);

            normalLines->InsertNextCell(line);

            unsigned char endColor[4] = {
                static_cast<unsigned char>((n.x() * 0.5f + 0.5f) * 255),
                static_cast<unsigned char>((n.y() * 0.5f + 0.5f) * 255),
                static_cast<unsigned char>((n.z() * 0.5f + 0.5f) * 255),
                255
            };
            normalColors->InsertNextTypedTuple(color);
            normalColors->InsertNextTypedTuple(endColor);
        }

        {
            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < pointCloud.numberOfPoints; ++i)
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
            normalActor->GetProperty()->SetLineWidth(1.5);

            //app.GetRenderer()->AddActor(normalActor);
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

        pointCloud.Terminate();
    });

    app.Initialize();
    app.Run();

    return EXIT_SUCCESS;
}
