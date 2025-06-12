#include <App/CustomTrackballStyle.h>
#include <App/App.h>

// Implement the New method
vtkStandardNewMacro(CustomTrackballStyle);

CustomTrackballStyle::CustomTrackballStyle() {
    LeftButtonPressed = false;
    RightButtonPressed = false;
}

void CustomTrackballStyle::OnLeftButtonDown()
{
    LeftButtonPressed = true;
    std::cout << "Left Button Pressed" << std::endl;

    if (LeftButtonPressed && RightButtonPressed) {
        std::cout << "Both Left and Right Buttons Pressed" << std::endl;

        HandleBothButtons();
    }

    app->OnMouseButtonPress(0);

    //vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void CustomTrackballStyle::OnMiddleButtonDown()
{
    MiddleButtonPressed = true;
    std::cout << "Middle Button Pressed" << std::endl;

    app->OnMouseButtonPress(1);
}

void CustomTrackballStyle::OnRightButtonDown()
{
    RightButtonPressed = true;
    std::cout << "Right Button Pressed" << std::endl;

    if (LeftButtonPressed && RightButtonPressed) {
        std::cout << "Both Left and Right Buttons Pressed" << std::endl;

        HandleBothButtons();
    }

    app->OnMouseButtonPress(2);

    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void CustomTrackballStyle::OnLeftButtonUp()
{
    LeftButtonPressed = false;
    std::cout << "Left Button Released" << std::endl;

    app->OnMouseButtonRelease(0);

    //vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void CustomTrackballStyle::OnMiddleButtonUp()
{
    MiddleButtonPressed = false;
    std::cout << "Middle Button Released" << std::endl;

    app->OnMouseButtonRelease(1);
}

void CustomTrackballStyle::OnRightButtonUp()
{
    RightButtonPressed = false;
    std::cout << "Right Button Released" << std::endl;

    app->OnMouseButtonRelease(2);

    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void CustomTrackballStyle::HandleBothButtons()
{
    std::cout << "Custom behavior for both buttons being pressed" << std::endl;
}

void CustomTrackballStyle::OnKeyPress()
{
    std::string key = this->GetInteractor()->GetKeySym();
    std::cout << "Key pressed: " << key << std::endl;

    app->OnKeyPress();

    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CustomTrackballStyle::OnMouseMove()
{
    int* pos = this->GetInteractor()->GetEventPosition();
    int* lastPos = this->GetInteractor()->GetLastEventPosition();

    if (RightButtonPressed)
    {
        this->Rotate();

        app->OnMouseMove(pos[0], pos[1], lastPos[0], lastPos[1], LeftButtonPressed, MiddleButtonPressed, RightButtonPressed);
    }
}

void CustomTrackballStyle::OnTimer()
{
}

void CustomTrackballStyle::Rotate()
{
    if (!CurrentRenderer)
        return;

    vtkRenderWindowInteractor* rwi = this->Interactor;

    int* pos = rwi->GetEventPosition();
    int* lastPos = rwi->GetLastEventPosition();

    int dx = pos[0] - lastPos[0];
    int dy = pos[1] - lastPos[1];

    int* size = CurrentRenderer->GetRenderWindow()->GetSize();
    double delta_elevation = -20.0 / size[1];
    double delta_azimuth = -20.0 / size[0];

    double rxf = dx * delta_azimuth * RotationSpeed;
    double ryf = dy * delta_elevation * RotationSpeed;

    vtkCamera* camera = CurrentRenderer->GetActiveCamera();
    camera->Azimuth(rxf);
    camera->Elevation(ryf);
    camera->OrthogonalizeViewUp();

    rwi->Render();
}
