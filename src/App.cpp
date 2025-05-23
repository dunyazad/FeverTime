#include "App.h"
#include <windows.h>

#include <Entity.h>

#include <Debugging/VisualDebugging.h>

TimerCallback* TimerCallback::New() { return new TimerCallback; }
void TimerCallback::SetApp(App* app) { this->app = app; }

void TimerCallback::Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
{
    if (eventId == vtkCommand::TimerEvent)
    {
        OnTimer();
    }
    else
    {
        std::cerr << "Unexpected event ID: " << eventId << std::endl;
    }
}

void TimerCallback::OnTimer()
{
    app->OnUpdate();

    VisualDebugging::Update();
}

PostRenderCallback* PostRenderCallback::New() { return new PostRenderCallback; }
void PostRenderCallback::SetApp(App* app) { this->app = app; }

void PostRenderCallback::Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
{
    if (eventId == vtkCommand::EndEvent)
    {
        OnPostRender();
    }
    else
    {
        std::cerr << "Unexpected event ID: " << eventId << std::endl;
    }
}

void PostRenderCallback::OnPostRender()
{
    app->OnPostRender();
}

App::App()
{
}

App::~App()
{
}

void App::Initialize()
{
    renderer->GetActiveCamera()->SetEyeAngle(0);
    renderer->UseDepthPeelingOn();
    renderer->SetMaximumNumberOfPeels(100);
    renderer->SetOcclusionRatio(0.1);

    renderWindow->StereoCapableWindowOff();
    renderWindow->SetStereoRender(false);
    renderWindow->SetStereoType(0);
    renderWindow->SetAlphaBitPlanes(1);
    renderWindow->SetMultiSamples(0);
    renderWindow->AddRenderer(renderer);

    interactor->SetRenderWindow(renderWindow);

    interactor->SetInteractorStyle(interactorStyle);
    interactor->Initialize();

	VisualDebugging::Initialize(renderer);

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

    timerCallback = vtkSmartPointer<TimerCallback>::New();
    timerCallback->SetApp(this);

    interactor->AddObserver(vtkCommand::TimerEvent, timerCallback);
    int timerId = interactor->CreateRepeatingTimer(16);
    if (timerId < 0) {
        std::cerr << "Error: Timer was not created!" << std::endl;
    }

    postRenderCallback = vtkSmartPointer<PostRenderCallback>::New();
    postRenderCallback->SetApp(this);

    renderWindow->AddObserver(vtkCommand::EndEvent, postRenderCallback);

    renderWindow->Render();
    interactor->Start();

    VisualDebugging::Terminate();
}

void App::OnUpdate()
{
    for (auto& kvp : appUpdateCallbacks)
    {
        kvp.second(this);
    }
}

void App::OnPostRender()
{
    for (auto& kvp : appPostRenderCallbacks)
    {
        kvp.second(this);
    }
}

void App::CaptureColorAndDepth(const string& saveDirectory)
{
	static int captureCount = 0;
	std::stringstream ss;
	ss << captureCount++;

	std::string depthDataFileName = saveDirectory + "\\depth_data_" + ss.str() + ".dpt";
	std::string depthmapFileName = saveDirectory + "\\depth_" + ss.str() + ".png";
	std::string colormapFileName = saveDirectory + "\\color_" + ss.str() + ".png";

	{
		vtkNew<vtkWindowToImageFilter> colorFilter;
		colorFilter->SetInput(renderWindow);
		colorFilter->SetInputBufferTypeToRGB();
		colorFilter->Update();

		vtkNew<vtkPNGWriter> colorWriter;
		colorWriter->SetFileName(colormapFileName.c_str());
		colorWriter->SetInputConnection(colorFilter->GetOutputPort());
		colorWriter->Write();
	}

	{
		vtkNew<vtkWindowToImageFilter> depthFilter;
		depthFilter->SetInput(renderWindow);
		depthFilter->SetInputBufferTypeToZBuffer();
		depthFilter->Update();

		vtkNew<vtkImageShiftScale> shiftScale;
		shiftScale->SetInputConnection(depthFilter->GetOutputPort());
		shiftScale->SetOutputScalarTypeToUnsignedChar();
		shiftScale->SetShift(0);
		shiftScale->SetScale(255);
		shiftScale->Update();

		vtkNew<vtkPNGWriter> depthWriter;
		depthWriter->SetFileName(depthmapFileName.c_str());
		depthWriter->SetInputConnection(shiftScale->GetOutputPort());
		depthWriter->Write();
	}

	{
		ofstream ofs;
		ofs.open(depthDataFileName, ios::out | ios::binary);

		int width = 256;
		int height = 480;
		ofs.write((char*)&width, sizeof(int));
		ofs.write((char*)&height, sizeof(int));

		Eigen::Matrix4f viewMatrix = vtkToEigen(renderer->GetActiveCamera()->GetViewTransformMatrix());
		Eigen::Matrix4f tm = viewMatrix.inverse();
		//auto& tm = cameraTransforms[0];

		vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
		windowToImageFilter->SetInput(GetRenderWindow());
		windowToImageFilter->SetInputBufferTypeToZBuffer(); // Get the depth buffer
		windowToImageFilter->Update();

		// Access the depth buffer as an image
		vtkSmartPointer<vtkImageData> depthImage = windowToImageFilter->GetOutput();

		// Get the depth values as a float array
		vtkSmartPointer<vtkFloatArray> depthArray = vtkFloatArray::SafeDownCast(depthImage->GetPointData()->GetScalars());

		double* clippingRange = renderer->GetActiveCamera()->GetClippingRange();
		float depthRatio = (float)(clippingRange[1] - clippingRange[0]);

		vtkIdType index = 0;
		for (float y = -24.0f; y < 24.0f; y += 0.1f)
		{
			for (float x = -12.8f; x < 12.8f; x += 0.1f)
			{
				float depth = depthArray->GetValue(index++);
				auto p = Transform(tm, { x, y, -depth * depthRatio });
				ofs.write((char*)p.data(), sizeof(Eigen::Vector3f));
			}
		}

		ofs.close();
	}
}

void App::CaptureAsPointCloud(const string& saveDirectory)
{
	static int captureCount = 0;
	std::stringstream ss;
	ss << captureCount++;
	std::string pointCloudFileName = saveDirectory + "\\point_" + ss.str() + ".ply";

	auto normals = vtkSmartPointer<vtkFloatArray>::New();
	normals->SetNumberOfComponents(3);
	normals->SetName("Normals");

	auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3); // RGB
	colors->SetName("Colors");

	auto windowToColorImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToColorImageFilter->SetInput(renderWindow);
	//windowToColorImageFilter->SetMagnification(1); // Optional, set to >1 for higher resolution
	windowToColorImageFilter->SetInputBufferTypeToRGB(); // Capture only RGB, not alpha
	windowToColorImageFilter->Update();

	auto imageData = windowToColorImageFilter->GetOutput();

	auto colorArray = vtkUnsignedCharArray::SafeDownCast(imageData->GetPointData()->GetScalars());

	vtkNew<vtkPoints> points;

	Eigen::Matrix4f viewMatrix = vtkToEigen(renderer->GetActiveCamera()->GetViewTransformMatrix());
	Eigen::Matrix4f tm = viewMatrix.inverse();

	vtkSmartPointer<vtkWindowToImageFilter> windowToDepthImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToDepthImageFilter->SetInput(GetRenderWindow());
	windowToDepthImageFilter->SetInputBufferTypeToZBuffer(); // Get the depth buffer
	windowToDepthImageFilter->Update();

	vtkSmartPointer<vtkImageData> depthImage = windowToDepthImageFilter->GetOutput();

	vtkSmartPointer<vtkFloatArray> depthArray = vtkFloatArray::SafeDownCast(depthImage->GetPointData()->GetScalars());

	double* clippingRange = renderer->GetActiveCamera()->GetClippingRange();
	float depthRatio = (float)(clippingRange[1] - clippingRange[0]);

	vector<Eigen::Vector3f> inputPoints;

	vtkIdType index = 0;
	for (float y = -24.0f; y < 24.0f; y += 0.1f)
	{
		for (float x = -12.8f; x < 12.8f; x += 0.1f)
		{
			float depth = depthArray->GetValue(index);
			if (depth < 1.0f)
			{
				auto p = Transform(tm, { x, y, -depth * depthRatio });
				//auto p = Eigen::Vector3f(x, y, -depth * depthRatio);

				inputPoints.push_back(p);

				points->InsertNextPoint(p.x(), p.y(), p.z());

				unsigned char rgb[3];
				colorArray->GetTypedTuple(index, rgb);

				colors->InsertNextTuple3(rgb[0], rgb[1], rgb[2]);
			}
			else
			{
				auto p = Transform(tm, { x, y, -depth * depthRatio });
				//auto p = Eigen::Vector3f(x, y, -depth * depthRatio);

				inputPoints.push_back(p);
			}
			index++;
		}
	}


	//{
	//	float3* d_points;
	//	float3* d_normals;
	//	cudaMallocManaged(&d_points, sizeof(float3) * 256 * 480);
	//	cudaMallocManaged(&d_normals, sizeof(float3) * 256 * 480);

	//	cudaDeviceSynchronize();

	//	cudaMemcpy(d_points, inputPoints.data(), sizeof(float3) * 256 * 480, cudaMemcpyHostToDevice);

	//	CUDA::GeneratePatchNormals(256, 480, d_points, 256 * 480, d_normals);

	//	for (size_t i = 0; i < inputPoints.size(); i++)
	//	{
	//		float depth = depthArray->GetValue(i);
	//		if (depth < 1.0f)
	//		{
	//			auto n = d_normals[i];
	//			normals->InsertNextTuple3(n.x, n.y, n.z);
	//		}
	//	}

	//	cudaFree(d_points);
	//	cudaFree(d_normals);
	//}

	vtkNew<vtkPolyData> polyData;
	polyData->SetPoints(points);

	vtkNew<vtkVertexGlyphFilter> vertexFilter;
	vertexFilter->SetInputData(polyData);
	vertexFilter->Update();
	polyData->ShallowCopy(vertexFilter->GetOutput());

	polyData->GetPointData()->SetNormals(normals);
	polyData->GetPointData()->SetScalars(colors);
	//polyData->GetPointData()->SetAttribute(colors, vtkDataSetAttributes::SCALARS);

	vtkNew<vtkPLYWriter> plyWriter;
	plyWriter->SetFileName(pointCloudFileName.c_str());
	plyWriter->SetInputData(polyData);
	plyWriter->SetFileTypeToASCII();
	plyWriter->SetColorModeToDefault();
	plyWriter->SetArrayName("Normals");
	plyWriter->SetArrayName("Colors");
	//plyWriter->Update();
	plyWriter->Write();
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