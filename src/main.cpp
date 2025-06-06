#pragma warning(disable : 4819)

#include "App.h"
#include <CUDA/main.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Clustering.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_ClusteringFilter.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_CheckOverlap.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Smoothing.cuh>

#include <Debugging/VisualDebugging.h>

int main(int argc, char** argv)
{
    App app;

    DevicePointCloud pointCloud;

    app.SetInitializeCallback([&](App& app) {
        VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 100.0f * 0.5f, 0.0f, 0.0f }, Color4::Red);
        VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 100.0f * 0.5f, 0.0f }, Color4::Green);
        VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 0.0f, 100.0f * 0.5f }, Color4::Blue);

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

        TestHalfEdge();

        app.GetRenderer()->SetBackground(0.3, 0.5, 0.7);

        app.GetRenderer()->ResetCamera();
        app.GetRenderWindow()->Render();

    });

    app.Initialize();

    app.Run();

    app.Terminate();

    return EXIT_SUCCESS;
}
