#include "App.h"
#include <windows.h>

#include <Entity.h>

App::App()
{
}

App::~App()
{
}

void App::Initialize()
{
    renderer->GetActiveCamera()->SetEyeAngle(0);

    renderWindow->StereoCapableWindowOff();
    renderWindow->SetStereoRender(false);
    renderWindow->SetStereoType(0);
    renderWindow->AddRenderer(renderer);

    interactor->SetRenderWindow(renderWindow);

    interactor->SetInteractorStyle(interactorStyle);

    if (nullptr != onInitializeCallback)
    {
        this->onInitializeCallback(*this);
    }
}

void App::Terminate()
{
    if (nullptr != onTerminateCallback)
    {
        this->onTerminateCallback();
    }
}

void App::Run()
{
    renderer->ResetCamera();
	renderWindow->Render();

#ifdef _WINDOWS
    MaximizeConsoleWindowOnMonitor(1);

    HWND hwnd = reinterpret_cast<HWND>(renderWindow->GetGenericWindowId());
    MaximizeWindowOnMonitor(hwnd, 2);
#endif

	interactor->Start();
}

Entity* App::CreateEntity(const string& name)
{
    Entity* entity = new Entity(renderer, name);

    if (!name.empty())
    {
        if (nameEntityIndexMapping.find(name) != nameEntityIndexMapping.end())
        {
            cerr << "[App] Entity with name \"" << name << "\" already exists!" << endl;
            delete entity;
            return nullptr;
        }
        nameEntityIndexMapping[name] = entities.size();
    }

    entities.push_back(entity);
    return entity;
}

Entity* App::GetEntity(const string& name)
{
    auto it = nameEntityIndexMapping.find(name);
    if (it != nameEntityIndexMapping.end())
    {
        return entities[it->second];
    }
    return nullptr;
}

Entity* App::GetEntity(unsigned int index)
{
    if (index < entities.size())
    {
        return entities[index];
    }
    return nullptr;
}

Entity* App::GetActiveEntity()
{
    if (entities.size() <= activeEntityIndex) return nullptr;
    else return entities[activeEntityIndex];
}