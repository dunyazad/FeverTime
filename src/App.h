#pragma once

#include <Common.h>

#include <Monitor.h>
#include <Serialization.hpp>
#include <CustomInteractorStyle.h>
#include <AppCallbacks.h>
#include <Entity.h>

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

	inline unsigned int GetActiveEntityIndex() const { return activeEntityIndex; }
	inline void SetActiveEntityIndex(unsigned int index)
	{
		activeEntityIndex = index;
		
		printf("current activeEntityIndex : %d\n", activeEntityIndex);

		for (size_t i = 0; i < entities.size(); i++)
		{
			if (activeEntityIndex != i)
			{
				printf("%s is Invisible\n", entityNameMapping[i].c_str());
				entities[i]->SetVisibility(false);
			}
			else
			{
				printf("%s is Visible\n", entityNameMapping[i].c_str());
				entities[i]->SetVisibility(true);
			}
		}
	}
	inline void IncreaseActiveEntityIndex()
	{
		if (activeEntityIndex >= entities.size() - 1) return;
		else SetActiveEntityIndex(activeEntityIndex + 1);
	}
	inline void DecreaseActiveEntityIndex()
	{
		if (activeEntityIndex <= 0) return;
		else SetActiveEntityIndex(activeEntityIndex - 1);
	}

	Entity* CreateEntity(const string& name = "");
	Entity* GetEntity(const string& name);
	Entity* GetEntity(unsigned int index);

	Entity* GetActiveEntity();

	vector<Entity*>& GetEntities() { return entities; }
	const vector<Entity*>& GetEntities() const { return entities; }

private:
	function<void(App&)> onInitializeCallback;
	function<void()> onTerminateCallback;

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle = vtkSmartPointer<CustomInteractorStyle>::New();

	map<string, Entity*> nameEntityMapping;
	map<unsigned int, string> entityNameMapping;
	vector<Entity*> entities;
	unsigned int activeEntityIndex = 0;
};
