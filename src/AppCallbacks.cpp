#include <AppCallbacks.h>

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
        if (picker->Pick(x, y, 0, renderer))
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
                vtkCamera* camera = renderer->GetActiveCamera();

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

                renderer->ResetCameraClippingRange(); // 클리핑 거리 보정
                renderer->GetRenderWindow()->Render(); // 즉시 렌더링
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
    std::string key = interactor->GetKeySym(); // 눌린 키의 이름 (예: "r", "Escape", "Up", "Down")

    std::cout << "[KEY] Pressed: " << key << std::endl;

    // 예: 특정 키에 대해 동작 수행
    if (key == "r") {
        std::cout << "R 키가 눌렸습니다. 카메라 리셋" << std::endl;
        if (renderer) {
            renderer->ResetCamera();
            renderer->GetRenderWindow()->Render();
        }
    }
    else if (key == "Escape") {
        std::cout << "종료" << std::endl;
        interactor->GetRenderWindow()->Finalize();
        interactor->TerminateApp();
    }
}
