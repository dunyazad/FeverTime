#pragma warning(disable : 4819)

#include "App.h"

#include <main.cuh>

bool operator == (uint3 a, uint3 b)
{
    if (a.x != b.x) return false;
    if (a.y != b.y) return false;
    if (a.z != b.z) return false;
    return true;
}

bool operator<(const uint3& a, const uint3& b) {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
}

ostream& operator<<(ostream& os, const uint3& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

pair<uint3, unsigned int> max1, max2;

void findTopTwo(const map<uint3, unsigned int>& colorHistogram) {
    max1.second = max2.second = 0;

    for (const auto& kv : colorHistogram) {
        if (kv.second > max1.second) {
            max2 = max1;
            max1 = kv;
        }
        else if (kv.second > max2.second) {
            max2 = kv;
        }
    }

    cout << "1st Max: " << max1.first << " -> " << max1.second << "\n";
    cout << "2nd Max: " << max2.first << " -> " << max2.second << "\n";
}


const string resource_file_name = "Compound";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

int main(int argc, char** argv)
{
    App app;

    PointCloud pointCloud;

    app.SetInitializeCallback([&](App& app) {
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
            keyCallback->pointCloud = &pointCloud;
            app.GetInteractor()->AddObserver(vtkCommand::KeyPressEvent, keyCallback);
        }

        app.GetRenderer()->SetBackground(0.3, 0.5, 0.7);

        if (false == pointCloud.LoadFromALP(resource_file_name_alp))
        {
            if (false == pointCloud.LoadFromPLY(resource_file_name_ply))
            {
                return;
            }
        }

        auto defaultEntity = app.CreateEntity("Default");
        defaultEntity->FromPointCloud(&pointCloud);

        {
            auto entity = app.CreateEntity("Normal Gradient");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeNormalGradient();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNormalGradient(0.05f, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }

        {
            auto entity = app.CreateEntity("Neighbor Count");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeNeighborCount();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNeighborCount(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        {
            auto entity = app.CreateEntity("Color Mutiplication");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeColorMultiplication();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByColorMultiplication(400, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }

        {
            auto entity = app.CreateEntity("Clustering");
            entity->CopyFrom(defaultEntity);

            pointCloud.Clustering();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByLabel(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }

        //{
        //    auto colorBinarization = app.CreateEntity("Color Binarization");
        //    entity->CopyTo(colorBinarization);

        //    colorBinarization->SetVisibility(false);
        //}
  
        //    auto hashToFloat = [](uint32_t seed) -> float {
        //        seed ^= seed >> 13;
        //        seed *= 0x5bd1e995;
        //        seed ^= seed >> 15;
        //        return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
        //    };
    });

    app.Initialize();

    app.Run();

    pointCloud.Terminate();

    app.Terminate();

    return EXIT_SUCCESS;
}
