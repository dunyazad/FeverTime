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

    if (elapsed > 50 && elapsed < 300) // ���� ����: 50ms �̻�, 300ms ����
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

                // ���� ī�޶��� �ü� ������ ������ ä focal point�� �̵�
                double direction[3];
                camera->GetDirectionOfProjection(direction);
                double dist = std::sqrt(
                    vtkMath::Distance2BetweenPoints(camera->GetPosition(), camera->GetFocalPoint()));

                // ���ο� position = focal - direction * �Ÿ�
                for (int i = 0; i < 3; ++i)
                    position[i] = focal[i] - direction[i] * dist;

                camera->SetFocalPoint(focal);
                camera->SetPosition(position);

                // �ʿ� �� up-vector �缳��
                // camera->SetViewUp(0, 1, 0);

                renderer->ResetCameraClippingRange(); // Ŭ���� �Ÿ� ����
                renderer->GetRenderWindow()->Render(); // ��� ������
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
    std::string key = interactor->GetKeySym(); // ���� Ű�� �̸� (��: "r", "Escape", "Up", "Down")

    std::cout << "[KEY] Pressed: " << key << std::endl;

    // ��: Ư�� Ű�� ���� ���� ����
    if (key == "r") {
        std::cout << "R Ű�� ���Ƚ��ϴ�. ī�޶� ����" << std::endl;
        if (renderer) {
            renderer->ResetCamera();
            renderer->GetRenderWindow()->Render();
        }
    }
    else if (key == "Escape") {
        std::cout << "����" << std::endl;
        interactor->GetRenderWindow()->Finalize();
        interactor->TerminateApp();
    }
}
