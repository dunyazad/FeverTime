#pragma once

#include <Common.h>

#include <Serialization.hpp>

#include <App/Monitor.h>
#include <App/CustomInteractorStyle.h>
#include <App/CustomTrackballStyle.h>
#include <App/AppCallbacks.h>
#include <App/Entity.h>
#include <App/USBHandler.h>

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

struct AppConfiguration
{
	int windowWidth = 1920;
	int windowHeight = 1080;
	bool maximizeRenderWindow = true;
	bool maximizeConsoleWindow = true;

	AppConfiguration()
	{
		windowWidth = 1920;
		windowHeight = 1080;
		maximizeRenderWindow = true;
		maximizeConsoleWindow = true;
	}

	AppConfiguration(
		int windowWidth,
		int windowHeight,
		bool maximizeRenderWindow,
		bool maximizeConsoleWindow)
		: windowWidth(windowWidth),
		windowHeight(windowHeight),
		maximizeRenderWindow(maximizeRenderWindow),
		maximizeConsoleWindow(maximizeConsoleWindow)
	{
	}
};

class App
{
public:
    App();
    ~App();

    void Run();
    void Run(AppConfiguration configuration);

    void AddAppStartCallback(function<void(App*)> f);
    void AddAppStartCallback(const string& name, function<void(App*)> f);
    void RemoveAppStartCallback();
    void RemoveAppStartCallback(const string& name);

    void AddAppUpdateCallback(function<void(App*)> f);
    void AddAppUpdateCallback(const string& name, function<void(App*)> f);
    void RemoveAppUpdateCallback();
    void RemoveAppUpdateCallback(const string& name);

    void AddAppPostRenderCallback(function<void(App*)> f);
    void AddAppPostRenderCallback(const string& name, function<void(App*)> f);
    void RemoveAppPostRenderCallback();
    void RemoveAppPostRenderCallback(const string& name);

    void AddKeyPressCallback(function<void(App*)> f);
    void AddKeyPressCallback(const string& name, function<void(App*)> f);
    void RemoveKeyPressCallback();
    void RemoveKeyPressCallback(const string& name);

    void AddMouseButtonPressCallback(function<void(App*, int)> f);
    void AddMouseButtonPressCallback(const string& name, function<void(App*, int)> f);
    void RemoveMouseButtonPressCallback();
    void RemoveMouseButtonPressCallback(const string& name);

    void AddMouseButtonReleaseCallback(function<void(App*, int)> f);
    void AddMouseButtonReleaseCallback(const string& name, function<void(App*, int)> f);
    void RemoveMouseButtonReleaseCallback();
    void RemoveMouseButtonReleaseCallback(const string& name);

    void AddMouseMoveCallback(function<void(App*, int, int, int, int, bool, bool, bool)> f);
    void AddMouseMoveCallback(const string& name, function<void(App*, int, int, int, int, bool, bool, bool)> f);
    void RemoveMouseMoveCallback();
    void RemoveMouseMoveCallback(const string& name);

#ifdef _WINDOWS
    void AddUSBEventCallback(function<void(App*, USBEvent)> f);
    void AddUSBEventCallback(const string& name, function<void(App*, USBEvent)> f);
    void RemoveUSBEventCallback();
    void RemoveUSBEventCallback(const string& name);
#endif

    void OnUpdate();
    void OnPostRender();

    void CaptureColorAndDepth(const string& saveDirectory);
    void CaptureAsPointCloud(const string& saveDirectory);

    static void OnKeyPress();
    static void OnMouseButtonPress(int button);
    static void OnMouseButtonRelease(int button);
    static void OnMouseMove(int posx, int posy, int lastx, int lasty, bool lButton, bool mButton, bool rButton);
#ifdef _WINDOWS
    static void OnUSBEvent(USBEvent usbEvent);
#endif

    inline AppConfiguration* Configuration() { return &configuration; }

    inline vtkSmartPointer<vtkRenderer> GetRenderer() const { return renderer; }
    inline vtkSmartPointer<vtkRenderWindow> GetRenderWindow() const { return renderWindow; }
    inline vtkSmartPointer<vtkRenderWindowInteractor> GetInteractor() const { return interactor; }

    Entity* CreateEntity(const string& name = "");
    Entity* GetEntity(const string& name);
    Entity* GetEntity(unsigned int index);

    Entity* GetActiveEntity();

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

    map<string, void*> registry;

private:
    static set<App*> s_instances;
    AppConfiguration configuration;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    vtkSmartPointer<CustomTrackballStyle> customTrackballStyle;

    vtkSmartPointer<vtkCallbackCommand> keyPressCallback;

    vtkSmartPointer<TimerCallback> timerCallback;
    vtkSmartPointer<PostRenderCallback> postRenderCallback;

    map<string, function<void(App*)>> appStartCallbacks;
    map<string, function<void(App*)>> appUpdateCallbacks;
    map<string, function<void(App*)>> appPostRenderCallbacks;
    map<string, function<void(App*)>> keyPressCallbacks;
    map<string, function<void(App*, int)>> mouseButtonPressCallbacks;
    map<string, function<void(App*, int)>> mouseButtonReleaseCallbacks;
    map<string, function<void(App*, int, int, int, int, bool, bool, bool)>> mouseMoveCallbacks;

#ifdef _WINDOWS
    map<string, function<void(App*, USBEvent)>> usbEventCallbacks;

    USBHandler usbHandler;
    mutex usbEventQueueLock;
    queue<USBEvent> usbEventQueue;
#endif

    map<string, unsigned int> nameEntityIndexMapping;
    vector<Entity*> entities;
    unsigned int activeEntityIndex = 0;

    bool captureEnabled = false;
};
