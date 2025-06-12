#include <AppCallbacks.h>
#include <App.h>

#include <CUDA/PointCloud.cuh>

#include <Debugging/VisualDebugging.h>

SingleClickPickerCallback* SingleClickPickerCallback::New()
{
    return new SingleClickPickerCallback;
}

void SingleClickPickerCallback::Execute(vtkObject* caller, unsigned long eventId, void* callData)
{
    //auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    //auto renderer = app->GetRenderer();
    //vtkCamera* camera = app->GetRenderer()->GetActiveCamera();

    //int* clickPos = interactor->GetEventPosition();

    //vtkSmartPointer<vtkCoordinate> coordinate = vtkSmartPointer<vtkCoordinate>::New();
    //coordinate->SetCoordinateSystemToDisplay();
    //coordinate->SetValue(clickPos[0], clickPos[1], 0.0);
    //double* world = coordinate->GetComputedWorldValue(renderer);

    //double cameraPos[3];
    //renderer->GetActiveCamera()->GetPosition(cameraPos);

    //Eigen::Vector3d origin(cameraPos);
    //Eigen::Vector3d pick(world);
    //Eigen::Vector3d dir = (pick - origin).normalized();

    //std::cout << "Ray: origin = " << origin.transpose()
    //    << ", direction = " << dir.transpose() << std::endl;

    //auto index = pointCloud->Pick(
    //    make_float3(cameraPos[0], cameraPos[1], cameraPos[2]),
    //    make_float3(dir[0], dir[1], dir[2]));

    //if (-1 != index)
    //{
    //    thrust::host_vector<float3> positions(pointCloud->GetPositions());
    //    auto pickedPosition = positions[index];

    //    VisualDebugging::Clear("Picked");
    //    VisualDebugging::AddLine("Picked",
    //        { (float)cameraPos[0], (float)cameraPos[1], (float)cameraPos[2] },
    //    {
    //        (float)cameraPos[0] + (float)dir[0] * 250.0f,
    //        (float)cameraPos[1] + (float)dir[1] * 250.0f,
    //        (float)cameraPos[2] + (float)dir[2] * 250.0f
    //    }, Color4::White);

    //    VisualDebugging::AddSphere("Picked",
    //        { pickedPosition.x, pickedPosition.y, pickedPosition.z },
    //        { 0.05f, 0.05f, 0.05f },
    //        Eigen::Vector3f::UnitZ(),
    //        Color4::Red);

    //    camera->SetFocalPoint(pickedPosition.x, pickedPosition.y, pickedPosition.z);

    //    app->GetActiveEntity()->UpdateColorFromBuffer(pointCloud);

    //    app->GetRenderer()->ResetCameraClippingRange();
    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
}

DoubleClickPickerCallback* DoubleClickPickerCallback::New()
{
    return new DoubleClickPickerCallback;
}

void DoubleClickPickerCallback::Execute(vtkObject* caller, unsigned long eventId, void* callData)
{
    //auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    //auto picker = static_cast<vtkPointPicker*>(interactor->GetPicker());

    //auto now = std::chrono::steady_clock::now();
    //auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastClickTime).count();
    //lastClickTime = now;

    //if (elapsed > 50 && elapsed < 300) // 감지 기준: 50ms 이상, 300ms 이하
    //{
    //    int x, y;
    //    interactor->GetEventPosition(x, y);
    //    if (picker->Pick(x, y, 0, app->GetRenderer()))
    //    {
    //        vtkIdType pid = picker->GetPointId();
    //        double* pos = picker->GetPickPosition();

    //        if (pid >= 0)
    //        {
    //            vtkCamera* camera = app->GetRenderer()->GetActiveCamera();

    //            double focal[3] = { pos[0], pos[1], pos[2] };
    //            double position[3];
    //            double direction[3];
    //            camera->GetDirectionOfProjection(direction);
    //            double dist = std::sqrt(vtkMath::Distance2BetweenPoints(camera->GetPosition(), camera->GetFocalPoint()));

    //            for (int i = 0; i < 3; ++i)
    //                position[i] = focal[i] - direction[i] * dist;

    //            camera->SetFocalPoint(focal);
    //            camera->SetPosition(position);

    //            auto p = Eigen::Vector3f((float)focal[0], (float)focal[1], (float)focal[2]);
    //            auto dx = Eigen::Vector3f(10.0f * 0.5f, 0.0f, 0.0f);
    //            auto dy = Eigen::Vector3f(0.0f, 10.0f * 0.5f, 0.0f);
    //            auto dz = Eigen::Vector3f(0.0f, 0.0f, 10.0f * 0.5f);

    //            VisualDebugging::Clear("Clicked");
    //            VisualDebugging::AddLine("Clicked", p, p + dx, Color4::Red);
    //            VisualDebugging::AddLine("Clicked", p, p + dy, Color4::Green);
    //            VisualDebugging::AddLine("Clicked", p, p + dz, Color4::Blue);

    //            app->GetRenderer()->ResetCameraClippingRange();
    //            app->GetRenderer()->GetRenderWindow()->Render();
    //        }
    //    }
    //}
}

KeyPressCallback* KeyPressCallback::New()
{
    return new KeyPressCallback;
}

void KeyPressCallback::Execute(vtkObject* caller, unsigned long eventId, void* callData)
{
    //auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    //std::string key = interactor->GetKeySym();

    //std::cout << "[KEY] Pressed: " << key << std::endl;

    //if (key == "Escape")
    //{
    //    std::cout << "종료" << std::endl;
    //    interactor->GetRenderWindow()->Finalize();
    //    interactor->TerminateApp();
    //}
    //else if (key == "r")
    //{
    //    std::cout << "R 키가 눌렸습니다. 카메라 리셋" << std::endl;
    //    if (app->GetRenderer())
    //    {
    //        app->GetRenderer()->ResetCamera();
    //        app->GetRenderer()->GetRenderWindow()->Render();
    //    }
    //}
    //else if (key == "grave")
    //{
    //    app->GetActiveEntity()->ToggleLighting();
    //    app->GetRenderWindow()->Render();
    //}
    //else if (key == "equal")
    //{
    //    app->GetActiveEntity()->IncreasePointSize();
    //    app->GetRenderWindow()->Render();
    //}
    //else if (key == "minus")
    //{
    //    app->GetActiveEntity()->DecreasePointSize();
    //    app->GetRenderWindow()->Render();
    //}
    //else if (key == "Tab")
    //{
    //    app->GetActiveEntity()->ToggleNormalVisibility();
    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
    //else if (key == "space")
    //{
    //    std::cout << "Space 키가 눌렸습니다." << std::endl;
    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
    //else if (key == "Prior")
    //{
    //    auto entity = app->GetActiveEntity();
    //    if(nullptr != entity)
    //    {
    //        if ("Clustering" == entity->GetName())
    //        {
    //            auto degree = app->GetClusteringDegree();
    //            degree += 1.0f;

    //            printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

    //            app->SetClusteringDegree(degree);

    //            //pointCloud->Clustering(degree);

    //            //PointCloudBuffers d_tempBuffers;
    //            //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            //pointCloud->SerializeColoringByLabel(d_tempBuffers);

    //            //PointCloudBuffers h_tempBuffers;
    //            //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            //d_tempBuffers.CopyTo(h_tempBuffers);

    //            //entity->UpdateColorFromBuffer(h_tempBuffers);

    //            //d_tempBuffers.Terminate();
    //            //h_tempBuffers.Terminate();
    //        }
    //        else if ("Clustering Sub" == entity->GetName())
    //        {
    //            auto degree = app->GetClusteringDegree();
    //            degree += 1.0f;

    //            printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

    //            app->SetClusteringDegree(degree);

    //            //pointCloud->Clustering(degree);

    //            //PointCloudBuffers d_tempBuffers;
    //            //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            //pointCloud->SerializeColoringByLabel(d_tempBuffers);

    //            //PointCloudBuffers h_tempBuffers;
    //            //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            //d_tempBuffers.CopyTo(h_tempBuffers);

    //            //entity->UpdateColorFromBuffer(h_tempBuffers);

    //            //d_tempBuffers.Terminate();
    //            //h_tempBuffers.Terminate();
    //        }
    //        /*
    //        else if ("Empty Neighbor Count" == entity->GetName())
    //        {
    //            auto threshold = app->GetEmptyNeighborCountThreshold();
    //            threshold += 1;

    //            printf("Empty NeighborCountThreshold : %d\n", threshold);

    //            app->SetEmptyNeighborCountThreshold(threshold);

    //            PointCloudBuffers d_tempBuffers;
    //            d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            pointCloud->SerializeColoringByEmptyNeighborCount(threshold, d_tempBuffers);

    //            PointCloudBuffers h_tempBuffers;
    //            h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            d_tempBuffers.CopyTo(h_tempBuffers);

    //            entity->UpdateColorFromBuffer(h_tempBuffers);

    //            d_tempBuffers.Terminate();
    //            h_tempBuffers.Terminate();
    //        }
    //        else if ("Normal Discontinuity" == entity->GetName())
    //        {
    //            auto threshold = app->GetNormalDiscontinuityThreshold();
    //            threshold += 1.0f;

    //            printf("Normal Discontinuity Threshold : %f = %f\n", threshold, threshold * M_PI / 180);

    //            app->SetNormalDiscontinuityThreshold(threshold);

    //            pointCloud->ComputeNormalDiscontinuity(threshold);

    //            PointCloudBuffers d_tempBuffers;
    //            d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            pointCloud->SerializeColoringByNormalDiscontinuity(d_tempBuffers);

    //            PointCloudBuffers h_tempBuffers;
    //            h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            d_tempBuffers.CopyTo(h_tempBuffers);

    //            entity->UpdateColorFromBuffer(h_tempBuffers);

    //            d_tempBuffers.Terminate();
    //            h_tempBuffers.Terminate();
    //        }
    //        else if ("Normal Divergence" == entity->GetName())
    //        {
    //            auto threshold = app->GetNormalDivergenceThreshold();
    //            threshold += 0.01f;

    //            printf("Normal Divergence Threshold : %f\n", threshold);

    //            app->SetNormalDivergenceThreshold(threshold);

    //            pointCloud->ComputeNormalDivergence();

    //            PointCloudBuffers d_tempBuffers;
    //            d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            pointCloud->SerializeColoringByNormalDivergence(threshold, d_tempBuffers);

    //            PointCloudBuffers h_tempBuffers;
    //            h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            d_tempBuffers.CopyTo(h_tempBuffers);

    //            entity->UpdateColorFromBuffer(h_tempBuffers);

    //            d_tempBuffers.Terminate();
    //            h_tempBuffers.Terminate();
    //        }
    //        */
    //    }

    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
    //else if (key == "Next")
    //{
    //    auto entity = app->GetActiveEntity();
    //    if (nullptr != entity)
    //    {
    //        if ("Clustering" == entity->GetName())
    //        {
    //            auto degree = app->GetClusteringDegree();
    //            degree -= 1.0f;

    //            printf("Clustering Degree : %f = %f\n", degree, degree* M_PI / 180);

    //            app->SetClusteringDegree(degree);

    //            //pointCloud->Clustering(degree);

    //            //PointCloudBuffers d_tempBuffers;
    //            //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            //pointCloud->SerializeColoringByLabel(d_tempBuffers);

    //            //PointCloudBuffers h_tempBuffers;
    //            //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            //d_tempBuffers.CopyTo(h_tempBuffers);

    //            //entity->UpdateColorFromBuffer(h_tempBuffers);

    //            //d_tempBuffers.Terminate();
    //            //h_tempBuffers.Terminate();
    //        }
    //        else if ("Clustering Sub" == entity->GetName())
    //        {
    //            auto degree = app->GetClusteringDegree();
    //            degree -= 1.0f;

    //            printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

    //            app->SetClusteringDegree(degree);

    //            //pointCloud->Clustering(degree);

    //            //PointCloudBuffers d_tempBuffers;
    //            //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            //pointCloud->SerializeColoringByLabel(d_tempBuffers);

    //            //PointCloudBuffers h_tempBuffers;
    //            //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            //d_tempBuffers.CopyTo(h_tempBuffers);

    //            //entity->UpdateColorFromBuffer(h_tempBuffers);

    //            //d_tempBuffers.Terminate();
    //            //h_tempBuffers.Terminate();
    //        }
    //        /*
    //        else if ("Empty Neighbor Count" == entity->GetName())
    //        {
    //            auto threshold = app->GetEmptyNeighborCountThreshold();
    //            threshold -= 1;

    //            printf("Empty NeighborCountThreshold : %d\n", threshold);

    //            app->SetEmptyNeighborCountThreshold(threshold);

    //            PointCloudBuffers d_tempBuffers;
    //            d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            pointCloud->SerializeColoringByEmptyNeighborCount(threshold, d_tempBuffers);

    //            PointCloudBuffers h_tempBuffers;
    //            h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            d_tempBuffers.CopyTo(h_tempBuffers);

    //            entity->UpdateColorFromBuffer(h_tempBuffers);

    //            d_tempBuffers.Terminate();
    //            h_tempBuffers.Terminate();
    //        }
    //        else if("Normal Discontinuity" == entity->GetName())
    //        {
    //            auto threshold = app->GetNormalDiscontinuityThreshold();
    //            threshold -= 1.0f;

    //            printf("Normal Discontinuity Threshold : %f = %f\n", threshold, threshold* M_PI / 180);

    //            app->SetNormalDiscontinuityThreshold(threshold);

    //            pointCloud->ComputeNormalDiscontinuity(threshold);

    //            PointCloudBuffers d_tempBuffers;
    //            d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            pointCloud->SerializeColoringByNormalDiscontinuity(d_tempBuffers);

    //            PointCloudBuffers h_tempBuffers;
    //            h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            d_tempBuffers.CopyTo(h_tempBuffers);

    //            entity->UpdateColorFromBuffer(h_tempBuffers);

    //            d_tempBuffers.Terminate();
    //            h_tempBuffers.Terminate();
    //        }
    //        else if ("Normal Divergence" == entity->GetName())
    //        {
    //            auto threshold = app->GetNormalDivergenceThreshold();
    //            threshold -= 0.01f;

    //            printf("Normal Divergence Threshold : %f\n", threshold);

    //            app->SetNormalDivergenceThreshold(threshold);

    //            pointCloud->ComputeNormalDivergence();

    //            PointCloudBuffers d_tempBuffers;
    //            d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

    //            pointCloud->SerializeColoringByNormalDivergence(threshold, d_tempBuffers);

    //            PointCloudBuffers h_tempBuffers;
    //            h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
    //            d_tempBuffers.CopyTo(h_tempBuffers);

    //            entity->UpdateColorFromBuffer(h_tempBuffers);

    //            d_tempBuffers.Terminate();
    //            h_tempBuffers.Terminate();
    //        }
    //        */
    //    }

    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
    //else if (key == "Left")
    //{
    //    app->DecreaseActiveEntityIndex();
    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
    //else if (key == "Right")
    //{
    //    app->IncreaseActiveEntityIndex();
    //    app->GetRenderer()->GetRenderWindow()->Render();
    //}
    //else if (key == "3")
    //{
    //    //this->SetAbortFlag(1);
    //}
}
