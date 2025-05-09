#include <AppCallbacks.h>
#include <App.h>

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

    //std::cout << "[KEY] Pressed: " << key << std::endl;

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
    else if (key == "equal")
    {
        vtkActorCollection* actors = app->GetRenderer()->GetActors();
        vtkCollectionSimpleIterator it;
        actors->InitTraversal(it);
        while (vtkActor* actor = actors->GetNextActor(it))
        {
            auto pointSize = actor->GetProperty()->GetPointSize() + 1;
            actor->GetProperty()->SetPointSize(pointSize);
        }

        app->GetRenderWindow()->Render();
    }
    else if (key == "minus")
    {
        vtkActorCollection* actors = app->GetRenderer()->GetActors();
        vtkCollectionSimpleIterator it;
        actors->InitTraversal(it);
        while (vtkActor* actor = actors->GetNextActor(it))
        {
            auto pointSize = actor->GetProperty()->GetPointSize() - 1;
            if (pointSize == 0) pointSize = 1;

            actor->GetProperty()->SetPointSize(pointSize);
        }

        app->GetRenderWindow()->Render();
    }
}
