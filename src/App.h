#pragma once

#include <Common.h>

#include <Monitor.h>
#include <Serialization.hpp>
#include <CustomInteractorStyle.h>
#include <AppCallbacks.h>
#include <Entity.h>

class TimerCallback : public vtkCommand
{
public:
	static TimerCallback* New();
	TimerCallback() = default;

	App* app;
	void SetApp(App* app);

	virtual void Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData)) override;

private:
	void OnTimer();
};

class PostRenderCallback : public vtkCommand
{
public:
	static PostRenderCallback* New();
	PostRenderCallback() = default;

	App* app;
	void SetApp(App* app);

	virtual void Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData)) override;

private:
	void OnPostRender();
};

class App
{
public:
	App();
	~App();

	void Initialize();
	void Terminate();

	void Run();

	void OnUpdate();
	void OnPostRender();

	void CaptureColorAndDepth(const string& saveDirectory);
	void CaptureAsPointCloud(const string& saveDirectory);

	Entity* CreateEntity(const string& name = "");
	Entity* GetEntity(const string& name);
	Entity* GetEntity(unsigned int index);

	Entity* GetActiveEntity();

	vector<Entity*>& GetEntities() { return entities; }
	const vector<Entity*>& GetEntities() const { return entities; }

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
		
		for (size_t i = 0; i < entities.size(); i++)
		{
			if (activeEntityIndex != i)
			{
				entities[i]->SetVisibility(false);
			}
			else
			{
				printf("Active Entity : %s\n", entities[i]->GetName().c_str());
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

	inline float GetClusteringDegree() { return clusteringDegree; }
	inline void SetClusteringDegree(float degree) { clusteringDegree = degree; }

	inline float GetNormalDiscontinuityThreshold() { return normalDiscontinuityThreshold; }
	inline void SetNormalDiscontinuityThreshold(float threshold) { normalDiscontinuityThreshold = threshold; }

	inline float GetNormalDivergenceThreshold() { return normalDivergenceThreshold; }
	inline void SetNormalDivergenceThreshold(float threshold) { normalDivergenceThreshold = threshold; }

private:
	function<void(App&)> onInitializeCallback;
	function<void()> onTerminateCallback;

	vtkSmartPointer<TimerCallback> timerCallback;
	vtkSmartPointer<PostRenderCallback> postRenderCallback;

	map<string, function<void(App*)>> appUpdateCallbacks;
	map<string, function<void(App*)>> appPostRenderCallbacks;

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle = vtkSmartPointer<CustomInteractorStyle>::New();
	
	map<string, unsigned int> nameEntityIndexMapping;
	vector<Entity*> entities;
	unsigned int activeEntityIndex = 0;

	float clusteringDegree = 10.0f;
	float normalDiscontinuityThreshold = 10.0f;
	float normalDivergenceThreshold = 0.05f;
};
