#pragma once

#include <Common.h>

class App;
class DevicePointCloud;

class EventCallback : public vtkCommand
{
public:
    inline void SetApp(App* app) { this->app = app; }

protected:
    App* app = nullptr;
};

class DoubleClickPickerCallback : public EventCallback
{
public:
    static DoubleClickPickerCallback* New();

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override;

private:
    chrono::steady_clock::time_point lastClickTime = chrono::steady_clock::now();
};

class KeyPressCallback : public EventCallback
{
public:
    static KeyPressCallback* New();

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override;

    DevicePointCloud* pointCloud;
};
