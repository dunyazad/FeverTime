#pragma once

#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

class App;

class CustomTrackballStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static CustomTrackballStyle* New();
    vtkTypeMacro(CustomTrackballStyle, vtkInteractorStyleTrackballCamera);

    CustomTrackballStyle();

    virtual void OnLeftButtonDown() override;
    virtual void OnMiddleButtonUp() override;
    virtual void OnRightButtonDown() override;
    virtual void OnLeftButtonUp() override;
    virtual void OnMiddleButtonDown() override;
    virtual void OnRightButtonUp() override;
    void HandleBothButtons();

    void OnKeyPress() override;
    void OnMouseMove() override;
    void OnTimer() override;

    inline void SetApp(App* app) { this->app = app; }
    inline void SetRotationSpeed(double speed) { RotationSpeed = speed; }

protected:
    void Rotate() override;

private:
    App* app = nullptr;
    bool LeftButtonPressed = false;
    bool MiddleButtonPressed = false;
    bool RightButtonPressed = false;

    double RotationSpeed = 50.0; // default value
};
