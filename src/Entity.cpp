#include <Entity.h>
#include <PointCloud.cuh>

Entity::Entity(vtkSmartPointer<vtkRenderer> renderer)
    : renderer(renderer)
{
}

Entity::~Entity()
{
}

void Entity::Clear()
{
    assembly->RemovePart(actor);
    assembly->RemovePart(normalActor);

    renderer->RemoveActor(assembly);
    renderer->RemoveActor(actor);
    renderer->RemoveActor(normalActor);

    assembly = nullptr;

    actor = nullptr;
    mapper = nullptr;
    polyData = nullptr;

    normalActor = nullptr;
    normalMapper = nullptr;
    normalPolyData = nullptr;
}

void Entity::FromPointCloud(PointCloud* pointCloud)
{
    Clear();

    assembly = vtkSmartPointer<vtkAssembly>::New();
    renderer->AddActor(assembly);

    actor = vtkSmartPointer<vtkActor>::New();
    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    polyData = vtkSmartPointer<vtkPolyData>::New();

    normalActor = vtkSmartPointer<vtkActor>::New();
    normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    normalPolyData = vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
    normals->SetNumberOfComponents(3);
    normals->SetName("Normals");

    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(4);
    colors->SetName("Colors");

    vtkSmartPointer<vtkPoints> normalLinesPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> normalLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> normalColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    normalColors->SetNumberOfComponents(4);
    normalColors->SetName("Colors");

    auto& buffers = pointCloud->GetHostBuffers();
    const size_t numPoints = pointCloud->GetNumberOfPoints();

    auto vertices = vtkSmartPointer<vtkCellArray>::New();

    for (vtkIdType i = 0; i < numPoints; ++i)
    {
        const auto& p = buffers.positions[i];
        const auto& n = buffers.normals[i];
        const auto& c = buffers.colors[i];

        points->InsertNextPoint(p.x(), p.y(), p.z());
        normals->InsertNextTuple3(n.x(), n.y(), n.z());

        unsigned char color[4] = { c.x(), c.y(), c.z(), 255 };
        colors->InsertNextTypedTuple(color);

        vtkIdType pid = i;
        vertices->InsertNextCell(1, &pid);

        double startPoint[3] = { p.x(), p.y(), p.z() };
        double endPoint[3] = {
            p.x() + n.x() * 0.1,
            p.y() + n.y() * 0.1,
            p.z() + n.z() * 0.1
        };

        vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
        vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, idStart);
        line->GetPointIds()->SetId(1, idEnd);

        normalLines->InsertNextCell(line);

        unsigned char endColor[4] = {
            static_cast<unsigned char>((n.x() * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.y() * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.z() * 0.5f + 0.5f) * 255),
            255
        };
        normalColors->InsertNextTypedTuple(color);
        normalColors->InsertNextTypedTuple(endColor);
    }

    polyData->SetPoints(points);
    polyData->SetVerts(vertices);
    polyData->GetPointData()->SetScalars(colors);
    polyData->GetPointData()->SetNormals(normals);

    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->SetScalarVisibility(true);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2.0f);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetLighting(true);

    assembly->AddPart(actor);

    normalPolyData->SetPoints(normalLinesPoints);
    normalPolyData->SetLines(normalLines);
    normalPolyData->GetPointData()->SetScalars(normalColors);

    normalMapper->SetInputData(normalPolyData);
    normalMapper->SetScalarModeToUsePointData();
    normalMapper->SetColorModeToDirectScalars();
    normalMapper->SetScalarVisibility(true);

    normalActor->SetMapper(normalMapper);
    normalActor->GetProperty()->SetLineWidth(2.0f);

    normalActor->SetVisibility(false);

    assembly->AddPart(normalActor);
}

void Entity::CopyFrom(Entity* other)
{
    if (nullptr == other) return;

    Clear();

    assembly = vtkSmartPointer<vtkAssembly>::New();
    renderer->AddActor(assembly);

    actor = vtkSmartPointer<vtkActor>::New();
    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    polyData = vtkSmartPointer<vtkPolyData>::New();

    normalActor = vtkSmartPointer<vtkActor>::New();
    normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    normalPolyData = vtkSmartPointer<vtkPolyData>::New();

    polyData->DeepCopy(other->polyData);

    std::cout << "CopyFrom: other num points = " << other->polyData->GetNumberOfPoints() << std::endl;
    std::cout << "CopyFrom: num points = " << polyData->GetNumberOfPoints() << std::endl;

    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->SetScalarVisibility(true);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2.0f);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetLighting(true);

    assembly->AddPart(actor);

    normalPolyData->DeepCopy(other->normalPolyData);

    normalMapper->SetInputData(normalPolyData);
    normalMapper->SetScalarModeToUsePointData();
    normalMapper->SetColorModeToDirectScalars();
    normalMapper->SetScalarVisibility(true);

    normalActor->SetMapper(normalMapper);
    normalActor->GetProperty()->SetLineWidth(2.0f);
    normalActor->SetVisibility(false);

    assembly->AddPart(normalActor);
}

void Entity::CopyTo(Entity* other)
{
    if (nullptr == other) return;

    other->CopyFrom(this);
}

void Entity::UpdateColorFromBuffer(const PointCloudBuffers& buffer)
{
    if (!polyData) return;

    vtkSmartPointer<vtkUnsignedCharArray> newColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    newColors->SetNumberOfComponents(4);
    newColors->SetName("Colors");

    for (size_t i = 0; i < buffer.numberOfPoints; ++i)
    {
        unsigned char color[4] = {
            buffer.colors[i].x(),
            buffer.colors[i].y(),
            buffer.colors[i].z(),
            255
        };
        newColors->InsertNextTypedTuple(color);
    }

    polyData->GetPointData()->SetScalars(newColors);
    polyData->Modified(); // Notify pipeline
}

void Entity::SetVisibility(bool visible)
{
    assembly->SetVisibility(visible);
}

void Entity::ToggleVisibility()
{
    bool visible = assembly->GetVisibility();
    assembly->SetVisibility(!visible);
}

void Entity::SetNormalVisibility(bool visible)
{
    normalActor->SetVisibility(visible);
}

void Entity::ToggleNormalVisibility()
{
    bool visible = normalActor->GetVisibility();
    normalActor->SetVisibility(!visible);
}

void Entity::SetLighting(bool lighting)
{
    actor->GetProperty()->SetLighting(lighting);
}

void Entity::ToggleLighting()
{
    bool lighting = actor->GetProperty()->GetLighting();
    actor->GetProperty()->SetLighting(!lighting);
}

void Entity::SetPointSize(unsigned int pointSize)
{
    actor->GetProperty()->SetPointSize(pointSize);
}

void Entity::IncreasePointSize()
{
    auto pointSize = actor->GetProperty()->GetPointSize();
    if (pointSize < 100)
    {
        actor->GetProperty()->SetPointSize(pointSize + 1);
    }
}

void Entity::DecreasePointSize()
{
    auto pointSize = actor->GetProperty()->GetPointSize();
    if (pointSize > 0)
    {
        actor->GetProperty()->SetPointSize(pointSize - 1);
    }
}
