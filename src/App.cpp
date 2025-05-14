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
    Entity* entity = new Entity(renderer);

    if (!name.empty())
    {
        if (nameEntityMapping.find(name) != nameEntityMapping.end())
        {
            cerr << "[App] Entity with name \"" << name << "\" already exists!" << endl;
            delete entity;
            return nullptr;
        }
        nameEntityMapping[name] = entity;
        entityNameMapping[static_cast<unsigned int>(entities.size())] = name;
    }

    entities.push_back(entity);
    return entity;
}

Entity* App::GetEntity(const string& name)
{
    auto it = nameEntityMapping.find(name);
    if (it != nameEntityMapping.end())
    {
        return it->second;
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