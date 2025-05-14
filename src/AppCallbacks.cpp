#include <AppCallbacks.h>
#include <App.h>

#include <PointCloud.cuh>

DoubleClickPickerCallback* DoubleClickPickerCallback::New()
{
    return new DoubleClickPickerCallback;
}

void DoubleClickPickerCallback::Execute(vtkObject* caller, unsigned long eventId, void* callData)
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
        if (picker->Pick(x, y, 0, app->GetRenderer()))
        {
            vtkIdType pid = picker->GetPointId();
            double* pos = picker->GetPickPosition();

            if (pid >= 0)
            {
                vtkCamera* camera = app->GetRenderer()->GetActiveCamera();

                double focal[3] = { pos[0], pos[1], pos[2] };
                double position[3];
                double direction[3];
                camera->GetDirectionOfProjection(direction);
                double dist = std::sqrt(vtkMath::Distance2BetweenPoints(camera->GetPosition(), camera->GetFocalPoint()));

                for (int i = 0; i < 3; ++i)
                    position[i] = focal[i] - direction[i] * dist;

                camera->SetFocalPoint(focal);
                camera->SetPosition(position);

                app->GetRenderer()->ResetCameraClippingRange();
                app->GetRenderer()->GetRenderWindow()->Render();
            }
        }
    }
}

KeyPressCallback* KeyPressCallback::New()
{
    return new KeyPressCallback;
}

void KeyPressCallback::Execute(vtkObject* caller, unsigned long eventId, void* callData)
{
    auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    std::string key = interactor->GetKeySym();

    std::cout << "[KEY] Pressed: " << key << std::endl;

    if (key == "Escape")
    {
        std::cout << "종료" << std::endl;
        interactor->GetRenderWindow()->Finalize();
        interactor->TerminateApp();
    }
    else if (key == "r")
    {
        std::cout << "R 키가 눌렸습니다. 카메라 리셋" << std::endl;
        if (app->GetRenderer())
        {
            app->GetRenderer()->ResetCamera();
            app->GetRenderer()->GetRenderWindow()->Render();
        }
    }
    else if (key == "grave")
    {
        app->GetActiveEntity()->ToggleLighting();
        app->GetRenderWindow()->Render();
    }
    else if (key == "equal")
    {
        app->GetActiveEntity()->IncreasePointSize();
        app->GetRenderWindow()->Render();
    }
    else if (key == "minus")
    {
        app->GetActiveEntity()->DecreasePointSize();
        app->GetRenderWindow()->Render();
    }
    else if (key == "Tab")
    {
        app->GetActiveEntity()->ToggleNormalVisibility();
        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "space")
    {
        std::cout << "Space 키가 눌렸습니다." << std::endl;
        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "Prior")
    {
        normalGradientThreshold += 0.005f;
        printf("ormalGradientThreshold : %f\n", normalGradientThreshold);

        //{ // Normal Gradient
        //    pointCloud->ComputeNormalGradient();

        //    PointCloudBuffers d_tempBuffers;
        //    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

        //    pointCloud->SerializeColoringByNormalGradient(normalGradientThreshold, d_tempBuffers);

        //    PointCloudBuffers h_tempBuffers;
        //    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);

        //    d_tempBuffers.CopyTo(h_tempBuffers);

        //    vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        //    clusteringColors->SetNumberOfComponents(4);
        //    clusteringColors->SetName("Colors");

        //    auto vertices = vtkSmartPointer<vtkCellArray>::New();
        //    for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
        //    {
        //        vtkIdType pid = i;
        //        vertices->InsertNextCell(1, &pid);

        //        unsigned char color[4] = {
        //            h_tempBuffers.colors[i].x(),
        //            h_tempBuffers.colors[i].y(),
        //            h_tempBuffers.colors[i].z(),
        //            255 };
        //        clusteringColors->InsertNextTypedTuple(color);
        //    }

        //    vtkSmartPointer<vtkPolyData> polyData =
        //        vtkPolyData::SafeDownCast(
        //            vtkPolyDataMapper::SafeDownCast(pointCloudClusteringActor->GetMapper())->GetInput()
        //        );

        //    polyData->GetPointData()->SetScalars(clusteringColors);

        //    d_tempBuffers.Terminate();
        //    h_tempBuffers.Terminate();
        //}

        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "Next")
    {
        normalGradientThreshold -= 0.005f;
        printf("ormalGradientThreshold : %f\n", normalGradientThreshold);

        //{ // Normal Gradient
        //    pointCloud->ComputeNormalGradient();

        //    PointCloudBuffers d_tempBuffers;
        //    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

        //    pointCloud->SerializeColoringByNormalGradient(normalGradientThreshold, d_tempBuffers);

        //    PointCloudBuffers h_tempBuffers;
        //    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);

        //    d_tempBuffers.CopyTo(h_tempBuffers);

        //    vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        //    clusteringColors->SetNumberOfComponents(4);
        //    clusteringColors->SetName("Colors");

        //    auto vertices = vtkSmartPointer<vtkCellArray>::New();
        //    for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
        //    {
        //        vtkIdType pid = i;
        //        vertices->InsertNextCell(1, &pid);

        //        unsigned char color[4] = {
        //            h_tempBuffers.colors[i].x(),
        //            h_tempBuffers.colors[i].y(),
        //            h_tempBuffers.colors[i].z(),
        //            255 };
        //        clusteringColors->InsertNextTypedTuple(color);
        //    }

        //    vtkSmartPointer<vtkPolyData> polyData =
        //        vtkPolyData::SafeDownCast(
        //            vtkPolyDataMapper::SafeDownCast(pointCloudClusteringActor->GetMapper())->GetInput()
        //        );

        //    polyData->GetPointData()->SetScalars(clusteringColors);

        //    d_tempBuffers.Terminate();
        //    h_tempBuffers.Terminate();
        //}

        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "Left")
    {
        app->DecreaseActiveEntityIndex();
        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "Right")
    {
        app->IncreaseActiveEntityIndex();
        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "3")
    {
        //this->SetAbortFlag(1);
    }
}
