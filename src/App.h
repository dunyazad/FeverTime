#pragma once

#include <Common.h>

#include <Serialization.hpp>
#include <AppCallbacks.h>

class App
{
public:
	App();
	~App();

	void Initialize();
	void Terminate();

	void Run();

	inline void SetInitializeCallback(function<void(App&)> callback) { onInitializeCallback = callback; }
	inline void SetTerminateCallback(function<void()> callback) { onTerminateCallback = callback; }

	inline vtkSmartPointer<vtkRenderer> GetRenderer() { return renderer; }
	inline vtkSmartPointer<vtkRenderWindow> GetRenderWindow() { return renderWindow; }
	inline vtkSmartPointer<vtkRenderWindowInteractor> GetInteractor() { return interactor; }
	inline vtkSmartPointer<vtkInteractorStyleTrackballCamera> GetInteractorStyle() { return interactorStyle; }

private:
	function<void(App&)> onInitializeCallback;
	function<void()> onTerminateCallback;

	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> interactor;
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle;
};
