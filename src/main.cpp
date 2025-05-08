#include "App.h"

#include <main.cuh>

PointCloud pointCloud;

ALPFormat<PointPNC> alp;

const string resource_file_name = "Compound";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

class DoubleClickPickerCallback : public vtkCommand
{
public:
    static DoubleClickPickerCallback* New() {
        return new DoubleClickPickerCallback;
    }

    void SetRenderer(vtkRenderer* ren) { this->Renderer = ren; }

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override
    {
        auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
        auto picker = static_cast<vtkPointPicker*>(interactor->GetPicker());

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastClickTime).count();
        lastClickTime = now;

        if (elapsed > 50 && elapsed < 300) // 감지 기준: 50ms 이상, 300ms 이하
        {
            int x, y;
            interactor->GetEventPosition(x, y);
            if (picker->Pick(x, y, 0, this->Renderer))
            {
                /*
                vtkIdType pid = picker->GetPointId();
                double* pos = picker->GetPickPosition();

                if (pid >= 0)
                {
                    std::cout << "[DOUBLE CLICK] Picked Point ID: " << pid << std::endl;
                    std::cout << "Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
                }
                */

                vtkIdType pid = picker->GetPointId();
                double* pos = picker->GetPickPosition();

                if (pid >= 0)
                {
                    vtkCamera* camera = this->Renderer->GetActiveCamera();

                    double focal[3] = { pos[0], pos[1], pos[2] };
                    double position[3];

                    // 현재 카메라의 시선 방향을 유지한 채 focal point만 이동
                    double direction[3];
                    camera->GetDirectionOfProjection(direction);
                    double dist = std::sqrt(
                        vtkMath::Distance2BetweenPoints(camera->GetPosition(), camera->GetFocalPoint()));

                    // 새로운 position = focal - direction * 거리
                    for (int i = 0; i < 3; ++i)
                        position[i] = focal[i] - direction[i] * dist;

                    camera->SetFocalPoint(focal);
                    camera->SetPosition(position);

                    // 필요 시 up-vector 재설정
                    // camera->SetViewUp(0, 1, 0);

                    this->Renderer->ResetCameraClippingRange(); // 클리핑 거리 보정
                    this->Renderer->GetRenderWindow()->Render(); // 즉시 렌더링
                }
            }
        }
    }

private:
    std::chrono::steady_clock::time_point lastClickTime = std::chrono::steady_clock::now();
    vtkRenderer* Renderer = nullptr;
};

class KeyPressCallback : public vtkCommand
{
public:
    static KeyPressCallback* New() {
        return new KeyPressCallback;
    }

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override
    {
        auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
        std::string key = interactor->GetKeySym(); // 눌린 키의 이름 (예: "r", "Escape", "Up", "Down")

        std::cout << "[KEY] Pressed: " << key << std::endl;

        // 예: 특정 키에 대해 동작 수행
        if (key == "r") {
            std::cout << "R 키가 눌렸습니다. 카메라 리셋" << std::endl;
            if (this->Renderer) {
                this->Renderer->ResetCamera();
                this->Renderer->GetRenderWindow()->Render();
            }
        }
        else if (key == "Escape") {
            std::cout << "종료" << std::endl;
            interactor->GetRenderWindow()->Finalize();
            interactor->TerminateApp();
        }
    }

    void SetRenderer(vtkRenderer* ren) { this->Renderer = ren; }

private:
    vtkRenderer* Renderer = nullptr;
};

int main(int argc, char** argv)
{
    App app;
    app.SetInitializeCallback([](App& app) {

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

        {
            auto picker = vtkSmartPointer<vtkPointPicker>::New();
            app.GetInteractor()->SetPicker(picker);

            auto doubleClickPickerCallback = vtkSmartPointer<DoubleClickPickerCallback>::New();
            doubleClickPickerCallback->SetRenderer(app.GetRenderer());

            app.GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, doubleClickPickerCallback);
        }

        {
            auto keyCallback = vtkSmartPointer<KeyPressCallback>::New();
            keyCallback->SetRenderer(app.GetRenderer());
            app.GetInteractor()->AddObserver(vtkCommand::KeyPressEvent, keyCallback);
        }

        app.GetRenderer()->AddActor(actor);
        app.GetRenderer()->SetBackground(0.3, 0.5, 0.7);

        app.GetRenderWindow()->SetSize(800, 600);

        });

    app.Initialize();
    app.Run();

    return EXIT_SUCCESS;
}
