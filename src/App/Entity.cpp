#include <App/Entity.h>
#include <CUDA/PointCloud.cuh>

Entity::Entity(vtkSmartPointer<vtkRenderer> renderer, const string& name)
    : renderer(renderer), name(name)
{
}

Entity::~Entity()
{
}

void Entity::Clear()
{
    roi = Eigen::AlignedBox3f(
        Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX),
        Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX));

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

void Entity::FromPointCloud(DevicePointCloud* pointCloud)
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

    const size_t numPoints = pointCloud->GetNumberOfElements();

    auto vertices = vtkSmartPointer<vtkCellArray>::New();

    thrust::host_vector<float3> h_positions(numPoints);
    thrust::copy(pointCloud->GetPositions().begin(), pointCloud->GetPositions().end(), h_positions.begin());

    thrust::host_vector<float3> h_normals(numPoints);
    thrust::copy(pointCloud->GetNormals().begin(), pointCloud->GetNormals().end(), h_normals.begin());

    thrust::host_vector<uchar4> h_colors(numPoints);
    thrust::copy(pointCloud->GetColors().begin(), pointCloud->GetColors().end(), h_colors.begin());

    for (vtkIdType i = 0; i < numPoints; ++i)
    {
        auto& p = h_positions[i];
        auto& n = h_normals[i];
        auto& c = h_colors[i];

        if (FLT_MAX == p.x) p.x = 0.0f;
        if (FLT_MAX == p.y) p.y = 0.0f;
        if (FLT_MAX == p.z) p.z = 0.0f;

        points->InsertNextPoint(p.x, p.y, p.z);
        normals->InsertNextTuple3(n.x, n.y, n.z);

        unsigned char color[4] = { c.x, c.y, c.z, c.w };
        colors->InsertNextTypedTuple(color);

        vtkIdType pid = i;
        vertices->InsertNextCell(1, &pid);

        double startPoint[3] = { p.x, p.y, p.z };
        double endPoint[3] = {
            p.x + n.x * 0.1,
            p.y + n.y * 0.1,
            p.z + n.z * 0.1
        };

        vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
        vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, idStart);
        line->GetPointIds()->SetId(1, idEnd);

        normalLines->InsertNextCell(line);

        unsigned char endColor[4] = {
            static_cast<unsigned char>((n.x * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.y * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.z * 0.5f + 0.5f) * 255),
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

void Entity::FromPointCloud(DevicePointCloud* pointCloud, const Eigen::AlignedBox3f& roi)
{
    Clear();

    this->roi = roi;

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

    const size_t numPoints = pointCloud->GetNumberOfElements();

    auto vertices = vtkSmartPointer<vtkCellArray>::New();

    int skipCount = 0;

    thrust::host_vector<float3> h_positions(numPoints);
    thrust::copy(pointCloud->GetPositions().begin(), pointCloud->GetPositions().end(), h_positions.begin());

    thrust::host_vector<float3> h_normals(numPoints);
    thrust::copy(pointCloud->GetNormals().begin(), pointCloud->GetNormals().end(), h_normals.begin());

    thrust::host_vector<uchar4> h_colors(numPoints);
    thrust::copy(pointCloud->GetColors().begin(), pointCloud->GetColors().end(), h_colors.begin());

    for (vtkIdType i = 0; i < numPoints; ++i)
    {
        auto& p = h_positions[i];
        auto& n = h_normals[i];
        auto& c = h_colors[i];

        if (FLT_MAX == p.x) p.x = 0.0f;
        if (FLT_MAX == p.y) p.y = 0.0f;
        if (FLT_MAX == p.z) p.z = 0.0f;

        if (false == roi.contains(Eigen::Vector3f(p.x, p.y, p.z)))
        {
            skipCount++;
            continue;
        }

        points->InsertNextPoint(p.x, p.y, p.z);
        normals->InsertNextTuple3(n.x, n.y, n.z);

        unsigned char color[4] = { c.x, c.y, c.z, c.w };
        colors->InsertNextTypedTuple(color);

        vtkIdType pid = i - skipCount;
        vertices->InsertNextCell(1, &pid);

        double startPoint[3] = { p.x, p.y, p.z };
        double endPoint[3] = {
            p.x + n.x * 0.1,
            p.y + n.y * 0.1,
            p.z + n.z * 0.1
        };

        vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
        vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, idStart);
        line->GetPointIds()->SetId(1, idEnd);

        normalLines->InsertNextCell(line);

        unsigned char endColor[4] = {
            static_cast<unsigned char>((n.x * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.y * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.z * 0.5f + 0.5f) * 255),
            255
        };
        normalColors->InsertNextTypedTuple(color);
        normalColors->InsertNextTypedTuple(endColor);
    }

    std::cout << "skipCount: " << skipCount << std::endl;

    std::cout << "Points: " << points->GetNumberOfPoints() << std::endl;
    std::cout << "Colors: " << colors->GetNumberOfTuples() << std::endl;
    std::cout << "Normals: " << normals->GetNumberOfTuples() << std::endl;

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

void Entity::FromPointCloud(HostPointCloud* pointCloud)
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

    const size_t numPoints = pointCloud->GetNumberOfElements();

    auto vertices = vtkSmartPointer<vtkCellArray>::New();

    for (vtkIdType i = 0; i < numPoints; ++i)
    {
        const auto& p = pointCloud->GetPositions()[i];
        const auto& n = pointCloud->GetNormals()[i];
        const auto& c = pointCloud->GetColors()[i];

        points->InsertNextPoint(p.x, p.y, p.z);
        normals->InsertNextTuple3(n.x, n.y, n.z);

        unsigned char color[4] = { c.x, c.y, c.z, c.w };
        colors->InsertNextTypedTuple(color);

        vtkIdType pid = i;
        vertices->InsertNextCell(1, &pid);

        double startPoint[3] = { p.x, p.y, p.z };
        double endPoint[3] = {
            p.x + n.x * 0.1,
            p.y + n.y * 0.1,
            p.z + n.z * 0.1
        };

        vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
        vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, idStart);
        line->GetPointIds()->SetId(1, idEnd);

        normalLines->InsertNextCell(line);

        unsigned char endColor[4] = {
            static_cast<unsigned char>((n.x * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.y * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.z * 0.5f + 0.5f) * 255),
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

void Entity::FromPointCloud(HostPointCloud* pointCloud, const Eigen::AlignedBox3f& roi)
{
    Clear();

    this->roi = roi;

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

    const size_t numPoints = pointCloud->GetNumberOfElements();

    auto vertices = vtkSmartPointer<vtkCellArray>::New();

    int skipCount = 0;

    for (vtkIdType i = 0; i < numPoints; ++i)
    {
        const auto& p = pointCloud->GetPositions()[i];
        const auto& n = pointCloud->GetNormals()[i];
        const auto& c = pointCloud->GetColors()[i];

        if (false == roi.contains(Eigen::Vector3f(p.x, p.y, p.z)))
        {
            skipCount++;
            continue;
        }

        points->InsertNextPoint(p.x, p.y, p.z);
        normals->InsertNextTuple3(n.x, n.y, n.z);

        unsigned char color[4] = { c.x, c.y, c.z, c.w };
        colors->InsertNextTypedTuple(color);

        vtkIdType pid = i - skipCount;
        vertices->InsertNextCell(1, &pid);

        double startPoint[3] = { p.x, p.y, p.z };
        double endPoint[3] = {
            p.x + n.x * 0.1,
            p.y + n.y * 0.1,
            p.z + n.z * 0.1
        };

        vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
        vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, idStart);
        line->GetPointIds()->SetId(1, idEnd);

        normalLines->InsertNextCell(line);

        unsigned char endColor[4] = {
            static_cast<unsigned char>((n.x * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.y * 0.5f + 0.5f) * 255),
            static_cast<unsigned char>((n.z * 0.5f + 0.5f) * 255),
            255
        };
        normalColors->InsertNextTypedTuple(color);
        normalColors->InsertNextTypedTuple(endColor);
    }

    std::cout << "skipCount: " << skipCount << std::endl;

    std::cout << "Points: " << points->GetNumberOfPoints() << std::endl;
    std::cout << "Colors: " << colors->GetNumberOfTuples() << std::endl;
    std::cout << "Normals: " << normals->GetNumberOfTuples() << std::endl;

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

//void Entity::FromPointCloud(DevicePointCloud* pointCloud)
//{
//    Clear();
//
//    assembly = vtkSmartPointer<vtkAssembly>::New();
//    renderer->AddActor(assembly);
//
//    actor = vtkSmartPointer<vtkActor>::New();
//    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    polyData = vtkSmartPointer<vtkPolyData>::New();
//
//    normalActor = vtkSmartPointer<vtkActor>::New();
//    normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    normalPolyData = vtkSmartPointer<vtkPolyData>::New();
//
//    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
//    normals->SetNumberOfComponents(3);
//    normals->SetName("Normals");
//
//    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
//    colors->SetNumberOfComponents(4);
//    colors->SetName("Colors");
//
//    vtkSmartPointer<vtkPoints> normalLinesPoints = vtkSmartPointer<vtkPoints>::New();
//    vtkSmartPointer<vtkCellArray> normalLines = vtkSmartPointer<vtkCellArray>::New();
//    vtkSmartPointer<vtkUnsignedCharArray> normalColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
//    normalColors->SetNumberOfComponents(4);
//    normalColors->SetName("Colors");
//
//    auto vertices = vtkSmartPointer<vtkCellArray>::New();
//
//    thrust::host_vector<Eigen::Vector3f> h_points(pointCloud->GetPoints());
//    thrust::host_vector<Eigen::Vector3f> h_normals(pointCloud->GetNormals());
//    thrust::host_vector<Eigen::Vector4b> h_colors(pointCloud->GetColors());
//
//    for (vtkIdType i = 0; i < h_points.size(); ++i)
//    {
//        const auto& p = h_points[i];
//        const auto& n = h_normals[i];
//        const auto& c = h_colors[i];
//
//        points->InsertNextPoint(p.x(), p.y(), p.z());
//        normals->InsertNextTuple3(n.x(), n.y(), n.z());
//
//        unsigned char color[4] = { c.x(), c.y(), c.z(), c.w() };
//        colors->InsertNextTypedTuple(color);
//
//        vtkIdType pid = i;
//        vertices->InsertNextCell(1, &pid);
//
//        double startPoint[3] = { p.x(), p.y(), p.z() };
//        double endPoint[3] = {
//            p.x() + n.x() * 0.1,
//            p.y() + n.y() * 0.1,
//            p.z() + n.z() * 0.1
//        };
//
//        vtkIdType idStart = normalLinesPoints->InsertNextPoint(startPoint);
//        vtkIdType idEnd = normalLinesPoints->InsertNextPoint(endPoint);
//
//        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
//        line->GetPointIds()->SetId(0, idStart);
//        line->GetPointIds()->SetId(1, idEnd);
//
//        normalLines->InsertNextCell(line);
//
//        unsigned char endColor[4] = {
//            static_cast<unsigned char>((n.x() * 0.5f + 0.5f) * 255),
//            static_cast<unsigned char>((n.y() * 0.5f + 0.5f) * 255),
//            static_cast<unsigned char>((n.z() * 0.5f + 0.5f) * 255),
//            255
//        };
//        normalColors->InsertNextTypedTuple(color);
//        normalColors->InsertNextTypedTuple(endColor);
//    }
//
//    polyData->SetPoints(points);
//    polyData->SetVerts(vertices);
//    polyData->GetPointData()->SetScalars(colors);
//    polyData->GetPointData()->SetNormals(normals);
//
//    mapper->SetInputData(polyData);
//    mapper->SetScalarModeToUsePointData();
//    mapper->SetColorModeToDirectScalars();
//    mapper->SetScalarVisibility(true);
//
//    actor->SetMapper(mapper);
//    actor->GetProperty()->SetPointSize(2.0f);
//    actor->GetProperty()->SetRepresentationToPoints();
//    actor->GetProperty()->SetLighting(true);
//
//    assembly->AddPart(actor);
//
//    normalPolyData->SetPoints(normalLinesPoints);
//    normalPolyData->SetLines(normalLines);
//    normalPolyData->GetPointData()->SetScalars(normalColors);
//
//    normalMapper->SetInputData(normalPolyData);
//    normalMapper->SetScalarModeToUsePointData();
//    normalMapper->SetColorModeToDirectScalars();
//    normalMapper->SetScalarVisibility(true);
//
//    normalActor->SetMapper(normalMapper);
//    normalActor->GetProperty()->SetLineWidth(2.0f);
//
//    normalActor->SetVisibility(false);
//
//    assembly->AddPart(normalActor);
//}
//
//void Entity::FromPointCloud(DevicePointCloud* pointCloud, const Eigen::AlignedBox3f& roi)
//{
//
//}
//
//void Entity::FromPointCloud(HostPointCloud* pointCloud)
//{
//
//}
//
//void Entity::FromPointCloud(HostPointCloud* pointCloud, const Eigen::AlignedBox3f& roi)
//{
//
//}

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

void Entity::CopyFrom(Entity* other, const Eigen::AlignedBox3f& roi)
{
    if (nullptr == other) return;

    Clear();

    this->roi = roi;

    assembly = vtkSmartPointer<vtkAssembly>::New();
    renderer->AddActor(assembly);

    actor = vtkSmartPointer<vtkActor>::New();
    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    polyData = vtkSmartPointer<vtkPolyData>::New();

    normalActor = vtkSmartPointer<vtkActor>::New();
    normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    normalPolyData = vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkPoints> newPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> newColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtkSmartPointer<vtkFloatArray> newNormals = vtkSmartPointer<vtkFloatArray>::New();
    vtkSmartPointer<vtkCellArray> newVertices = vtkSmartPointer<vtkCellArray>::New();

    newColors->SetNumberOfComponents(4);
    newColors->SetName("Colors");

    newNormals->SetNumberOfComponents(3);
    newNormals->SetName("Normals");

    vtkPoints* oldPoints = other->polyData->GetPoints();
    auto* oldColors = vtkUnsignedCharArray::SafeDownCast(other->polyData->GetPointData()->GetScalars());
    auto* oldNormals = vtkFloatArray::SafeDownCast(other->polyData->GetPointData()->GetNormals());

    vtkIdType count = oldPoints->GetNumberOfPoints();

    for (vtkIdType i = 0; i < count; ++i)
    {
        double pt[3];
        oldPoints->GetPoint(i, pt);
        Eigen::Vector3f pos(pt[0], pt[1], pt[2]);

        if (false == roi.contains(pos)) continue;

        vtkIdType newId = newPoints->InsertNextPoint(pt);

        double normal[3] = { 0, 0, 0 };
        if (oldNormals) oldNormals->GetTuple(i, normal);
        newNormals->InsertNextTuple3(normal[0], normal[1], normal[2]);

        unsigned char color[4] = { 255, 255, 255, 255 };
        if (oldColors) oldColors->GetTypedTuple(i, color);
        newColors->InsertNextTypedTuple(color);

        newVertices->InsertNextCell(1, &newId);
    }

    polyData->SetPoints(newPoints);
    polyData->SetVerts(newVertices);
    polyData->GetPointData()->SetNormals(newNormals);
    polyData->GetPointData()->SetScalars(newColors);

    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->SetScalarVisibility(true);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2.0f);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetLighting(true);

    assembly->AddPart(actor);

    vtkSmartPointer<vtkPoints> filteredNormalPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> filteredNormalLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> filteredNormalColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    filteredNormalColors->SetNumberOfComponents(4);
    filteredNormalColors->SetName("Colors");

    vtkPoints* oldNormalPoints = other->normalPolyData->GetPoints();
    vtkCellArray* oldNormalLines = other->normalPolyData->GetLines();
    vtkUnsignedCharArray* oldNormalColors = vtkUnsignedCharArray::SafeDownCast(
        other->normalPolyData->GetPointData()->GetScalars());

    std::map<vtkIdType, vtkIdType> pointMap;
    vtkIdType pid = 0;
    vtkIdType newPid = 0;

    vtkIdType npts = 0;
    const vtkIdType* pts = nullptr;
    oldNormalLines->InitTraversal();

    while (oldNormalLines->GetNextCell(npts, pts))
    {
        if (npts != 2) continue;

        double p0[3], p1[3];
        oldNormalPoints->GetPoint(pts[0], p0);
        oldNormalPoints->GetPoint(pts[1], p1);

        Eigen::Vector3f p(p0[0], p0[1], p0[2]);
        if (false == roi.contains(p)) continue;

        vtkIdType id0 = filteredNormalPoints->InsertNextPoint(p0);
        vtkIdType id1 = filteredNormalPoints->InsertNextPoint(p1);

        if (oldNormalColors)
        {
            unsigned char c0[4], c1[4];
            oldNormalColors->GetTypedTuple(pts[0], c0);
            oldNormalColors->GetTypedTuple(pts[1], c1);
            filteredNormalColors->InsertNextTypedTuple(c0);
            filteredNormalColors->InsertNextTypedTuple(c1);
        }

        vtkSmartPointer<vtkLine> newLine = vtkSmartPointer<vtkLine>::New();
        newLine->GetPointIds()->SetId(0, id0);
        newLine->GetPointIds()->SetId(1, id1);
        filteredNormalLines->InsertNextCell(newLine);
    }
    normalPolyData->SetPoints(filteredNormalPoints);
    normalPolyData->SetLines(filteredNormalLines);
    normalPolyData->GetPointData()->SetScalars(filteredNormalColors);

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

void Entity::CopyTo(Entity* other, const Eigen::AlignedBox3f& roi)
{
    if (nullptr == other) return;

    other->CopyFrom(this);
}

void Entity::UpdateColorFromBuffer(DevicePointCloud* pointCloud)
{
    if (!polyData) return;

    vtkSmartPointer<vtkUnsignedCharArray> newColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    newColors->SetNumberOfComponents(4);
    newColors->SetName("Colors");

    thrust::host_vector<uchar4> h_colors(pointCloud->GetColors());

    for (size_t i = 0; i < pointCloud->GetNumberOfElements(); ++i)
    {
        auto c = h_colors[i];

        unsigned char color[4] = { c.x, c.y, c.z, c.w };
        newColors->InsertNextTypedTuple(color);
    }

    polyData->GetPointData()->SetScalars(newColors);
    polyData->Modified();
}

void Entity::UpdateColorFromBuffer(HostPointCloud* pointCloud)
{
    if (!polyData) return;

    vtkSmartPointer<vtkUnsignedCharArray> newColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    newColors->SetNumberOfComponents(4);
    newColors->SetName("Colors");

    for (size_t i = 0; i < pointCloud->GetNumberOfElements(); ++i)
    {
        auto c = pointCloud->GetColors()[i];
        unsigned char color[4] = { c.x, c.y, c.z, c.w };
        newColors->InsertNextTypedTuple(color);
    }

    polyData->GetPointData()->SetScalars(newColors);
    polyData->Modified();
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
