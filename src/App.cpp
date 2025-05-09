#include "App.h"

#include <vtkRenderWindow.h>
#include <windows.h> // For HWND

App::App()
{
}

App::~App()
{
}

void App::Initialize()
{
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor->SetInteractorStyle(interactorStyle);

	this->onInitializeCallback(*this);
}

void App::Terminate()
{
	this->onTerminateCallback();
}

void App::Run()
{
	renderWindow->Render();

#ifdef _WINDOWS
    MaximizeConsoleWindowOnMonitor(1);

    HWND hwnd = reinterpret_cast<HWND>(renderWindow->GetGenericWindowId());
    MaximizeWindowOnMonitor(hwnd, 2);
#endif

	interactor->Start();
}
