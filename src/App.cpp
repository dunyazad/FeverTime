#include "App.h"

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
	interactor->Start();
}
