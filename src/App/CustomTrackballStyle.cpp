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
    else if (MiddleButtonPressed)
    {
        this->Pan();

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

void CustomTrackballStyle::Pan()
{
    if (!CurrentRenderer)
        return;

    vtkRenderWindowInteractor* rwi = this->Interactor;
    vtkCamera* camera = CurrentRenderer->GetActiveCamera();
    if (!camera)
        return;

    int* pos = rwi->GetEventPosition();
    int* lastPos = rwi->GetLastEventPosition();

    int dx = pos[0] - lastPos[0];
    int dy = pos[1] - lastPos[1];

    int* size = CurrentRenderer->GetRenderWindow()->GetSize();
    double delta_x = static_cast<double>(dx) / size[0];
    double delta_y = static_cast<double>(dy) / size[1];

    double bounds[6];
    CurrentRenderer->ComputeVisiblePropBounds(bounds);

    double center[3] = {
        (bounds[0] + bounds[1]) / 2.0,
        (bounds[2] + bounds[3]) / 2.0,
        (bounds[4] + bounds[5]) / 2.0
    };

    double viewFocus[4], viewPoint[4];
    camera->GetFocalPoint(viewFocus);
    camera->GetPosition(viewPoint);

    CurrentRenderer->SetWorldPoint(viewFocus[0], viewFocus[1], viewFocus[2], 1.0);
    CurrentRenderer->WorldToDisplay();
    double* displayFocus = CurrentRenderer->GetDisplayPoint();

    double focalDepth = displayFocus[2];

    double newDisplayPoint[3] = {
        displayFocus[0] - dx,
        displayFocus[1] - dy,
        focalDepth
    };

    CurrentRenderer->SetDisplayPoint(newDisplayPoint);
    CurrentRenderer->DisplayToWorld();
    double* newWorldPoint = CurrentRenderer->GetWorldPoint();
    if (newWorldPoint[3] == 0.0)
        return;

    double worldFocus[3] = {
        newWorldPoint[0] / newWorldPoint[3],
        newWorldPoint[1] / newWorldPoint[3],
        newWorldPoint[2] / newWorldPoint[3]
    };

    double motionVector[3] = {
        worldFocus[0] - viewFocus[0],
        worldFocus[1] - viewFocus[1],
        worldFocus[2] - viewFocus[2]
    };

    camera->SetFocalPoint(
        viewFocus[0] + motionVector[0],
        viewFocus[1] + motionVector[1],
        viewFocus[2] + motionVector[2]);

    camera->SetPosition(
        viewPoint[0] + motionVector[0],
        viewPoint[1] + motionVector[1],
        viewPoint[2] + motionVector[2]);

    rwi->Render();
}
