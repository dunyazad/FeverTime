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
        auto entity = app->GetActiveEntity();
        if(nullptr != entity)
        {
            if ("Clustering" == entity->GetName())
            {
                auto degree = app->GetClusteringDegree();
                degree += 1.0f;

                printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                app->SetClusteringDegree(degree);

                pointCloud->Clustering(degree);

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByLabel(d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
            else if ("Clustering Sub" == entity->GetName())
            {
                auto degree = app->GetClusteringDegree();
                degree += 1.0f;

                printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                app->SetClusteringDegree(degree);

                pointCloud->Clustering(degree);

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByLabel(d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
            else if ("Normal Discontinuity" == entity->GetName())
            {
                auto threshold = app->GetNormalDiscontinuityThreshold();
                threshold += 1.0f;

                printf("Normal Discontinuity Threshold : %f = %f\n", threshold, threshold * M_PI / 180);

                app->SetNormalDiscontinuityThreshold(threshold);

                pointCloud->ComputeNormalDiscontinuity(threshold);

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByNormalDiscontinuity(d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
            else if ("Normal Divergence" == entity->GetName())
            {
                auto threshold = app->GetNormalDivergenceThreshold();
                threshold += 0.01f;

                printf("Normal Divergence Threshold : %f\n", threshold);

                app->SetNormalDivergenceThreshold(threshold);

                pointCloud->ComputeNormalDivergence();

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByNormalDivergence(threshold, d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
        }

        app->GetRenderer()->GetRenderWindow()->Render();
    }
    else if (key == "Next")
    {
        auto entity = app->GetActiveEntity();
        if (nullptr != entity)
        {
            if ("Clustering" == entity->GetName())
            {
                auto degree = app->GetClusteringDegree();
                degree -= 1.0f;

                printf("Clustering Degree : %f = %f\n", degree, degree* M_PI / 180);

                app->SetClusteringDegree(degree);

                pointCloud->Clustering(degree);

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByLabel(d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
            else if ("Clustering Sub" == entity->GetName())
            {
                auto degree = app->GetClusteringDegree();
                degree -= 1.0f;

                printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                app->SetClusteringDegree(degree);

                pointCloud->Clustering(degree);

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByLabel(d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
            else if("Normal Discontinuity" == entity->GetName())
            {
                auto threshold = app->GetNormalDiscontinuityThreshold();
                threshold -= 1.0f;

                printf("Normal Discontinuity Threshold : %f = %f\n", threshold, threshold* M_PI / 180);

                app->SetNormalDiscontinuityThreshold(threshold);

                pointCloud->ComputeNormalDiscontinuity(threshold);

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByNormalDiscontinuity(d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
            else if ("Normal Divergence" == entity->GetName())
            {
                auto threshold = app->GetNormalDivergenceThreshold();
                threshold -= 0.01f;

                printf("Normal Divergence Threshold : %f\n", threshold);

                app->SetNormalDivergenceThreshold(threshold);

                pointCloud->ComputeNormalDivergence();

                PointCloudBuffers d_tempBuffers;
                d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                pointCloud->SerializeColoringByNormalDivergence(threshold, d_tempBuffers);

                PointCloudBuffers h_tempBuffers;
                h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                d_tempBuffers.CopyTo(h_tempBuffers);

                entity->UpdateColorFromBuffer(h_tempBuffers);

                d_tempBuffers.Terminate();
                h_tempBuffers.Terminate();
            }
        }

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
