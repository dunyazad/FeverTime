#include "App.h"

#include <main.cuh>

const string resource_file_name = "Compound";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

int main(int argc, char** argv)
{
    App app;

    PointCloud pointCloud;

    auto pointCloudActor = vtkSmartPointer<vtkActor>::New();
    auto pointCloudNormalActor = vtkSmartPointer<vtkActor>::New();
    auto pointCloudClusteringActor = vtkSmartPointer<vtkActor>::New();

    app.SetInitializeCallback([&](App& app) {
        {
            auto picker = vtkSmartPointer<vtkPointPicker>::New();
            app.GetInteractor()->SetPicker(picker);

            auto doubleClickPickerCallback = vtkSmartPointer<DoubleClickPickerCallback>::New();
            doubleClickPickerCallback->SetApp(&app);

            app.GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, doubleClickPickerCallback);
        }

        {
            auto keyCallback = vtkSmartPointer<KeyPressCallback>::New();
            keyCallback->SetApp(&app);
            keyCallback->pointCloud = &pointCloud;
            keyCallback->pointCloudActor = pointCloudActor;
            keyCallback->pointCloudNormalActor = pointCloudNormalActor;
            keyCallback->pointCloudClusteringActor = pointCloudClusteringActor;
            app.GetInteractor()->AddObserver(vtkCommand::KeyPressEvent, keyCallback);
        }

        app.GetRenderer()->SetBackground(0.3, 0.5, 0.7);

        if (false == pointCloud.LoadFromALP(resource_file_name_alp))
        {
            if (false == pointCloud.LoadFromPLY(resource_file_name_ply))
            {
                return;
            }
        }

        //cout << pointCloud.GetHostBuffers().aabb.min() << endl;
        //cout << pointCloud.GetHostBuffers().aabb.max() << endl;
        cout << pointCloud.GetHostBuffers().aabb.max() - pointCloud.GetHostBuffers().aabb.min() << endl;

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

        for (unsigned int i = 0; i < pointCloud.GetNumberOfPoints(); i++)
        {
            auto& p = pointCloud.GetHostBuffers().positions[i];
            auto& n = pointCloud.GetHostBuffers().normals[i];
            auto& c = pointCloud.GetHostBuffers().colors[i];

            points->InsertNextPoint(p.x(), p.y(), p.z());
            normals->InsertNextTuple3(n.x(), n.y(), n.z());

            auto hashToFloat = [](uint32_t seed) -> float {
                seed ^= seed >> 13;
                seed *= 0x5bd1e995;
                seed ^= seed >> 15;
                return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
            };

            unsigned char color[4] = { c.x(), c.y(), c.z(), 255 };
            colors->InsertNextTypedTuple(color);

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

        { // Main
            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < pointCloud.GetNumberOfPoints(); ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(colors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudActor->SetMapper(mapper);
            pointCloudActor->GetProperty()->SetPointSize(2);
            pointCloudActor->GetProperty()->SetRepresentationToPoints();
            pointCloudActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudActor);
        }

        { // Normal
            auto normalPolyData = vtkSmartPointer<vtkPolyData>::New();
            normalPolyData->SetPoints(normalLinesPoints);
            normalPolyData->SetLines(normalLines);
            normalPolyData->GetPointData()->SetScalars(normalColors);

            auto normalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            normalMapper->SetInputData(normalPolyData);

            normalMapper->SetScalarModeToUsePointData();
            normalMapper->SetColorModeToDirectScalars();
            normalMapper->SetScalarVisibility(true);

            pointCloudNormalActor->SetMapper(normalMapper);
            pointCloudNormalActor->GetProperty()->SetLineWidth(1.5);

            app.GetRenderer()->AddActor(pointCloudNormalActor);
        }

        // Normal Gradient
        /*
        {
            pointCloud.ComputeNormalGradient();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNormalGradient(0.05f, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            d_tempBuffers.CopyTo(h_tempBuffers);

            vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            clusteringColors->SetNumberOfComponents(4);
            clusteringColors->SetName("Colors");

            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);

                unsigned char color[4] = {
                    h_tempBuffers.colors[i].x(),
                    h_tempBuffers.colors[i].y(),
                    h_tempBuffers.colors[i].z(),
                    255 };
                clusteringColors->InsertNextTypedTuple(color);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(clusteringColors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudClusteringActor->SetMapper(mapper);
            pointCloudClusteringActor->GetProperty()->SetPointSize(2);
            pointCloudClusteringActor->GetProperty()->SetRepresentationToPoints();
            pointCloudClusteringActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudClusteringActor);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();
        }
        */

        // Normal Divergence
        /*
        {
            pointCloud.ComputeNormalDivergence();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNormalDivergence(0.09f, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            d_tempBuffers.CopyTo(h_tempBuffers);

            vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            clusteringColors->SetNumberOfComponents(4);
            clusteringColors->SetName("Colors");

            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);

                unsigned char color[4] = {
                    h_tempBuffers.colors[i].x(),
                    h_tempBuffers.colors[i].y(),
                    h_tempBuffers.colors[i].z(),
                    255 };
                clusteringColors->InsertNextTypedTuple(color);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(clusteringColors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudClusteringActor->SetMapper(mapper);
            pointCloudClusteringActor->GetProperty()->SetPointSize(2);
            pointCloudClusteringActor->GetProperty()->SetRepresentationToPoints();
            pointCloudClusteringActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudClusteringActor);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();
        }
        */

        // Compute Neighbor Count
        /*
        {
            pointCloud.ComputeNeighborCount();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNeighborCount(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            d_tempBuffers.CopyTo(h_tempBuffers);

            vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            clusteringColors->SetNumberOfComponents(4);
            clusteringColors->SetName("Colors");

            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);

                unsigned char color[4] = {
                    h_tempBuffers.colors[i].x(),
                    h_tempBuffers.colors[i].y(),
                    h_tempBuffers.colors[i].z(),
                    255 };
                clusteringColors->InsertNextTypedTuple(color);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(clusteringColors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudClusteringActor->SetMapper(mapper);
            pointCloudClusteringActor->GetProperty()->SetPointSize(2);
            pointCloudClusteringActor->GetProperty()->SetRepresentationToPoints();
            pointCloudClusteringActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudClusteringActor);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();
        }
        */

        // Compute Color Distance
        {
            pointCloud.ComputeColorMultiplication();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByColorMultiplication(390.0f, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            d_tempBuffers.CopyTo(h_tempBuffers);

            vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            clusteringColors->SetNumberOfComponents(4);
            clusteringColors->SetName("Colors");

            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);

                unsigned char color[4] = {
                    h_tempBuffers.colors[i].x(),
                    h_tempBuffers.colors[i].y(),
                    h_tempBuffers.colors[i].z(),
                    255 };
                clusteringColors->InsertNextTypedTuple(color);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(clusteringColors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudClusteringActor->SetMapper(mapper);
            pointCloudClusteringActor->GetProperty()->SetPointSize(2);
            pointCloudClusteringActor->GetProperty()->SetRepresentationToPoints();
            pointCloudClusteringActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudClusteringActor);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();
        }

        // Clustering
        /*
        {
            pointCloud.Clustering();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByLabel(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            d_tempBuffers.CopyTo(h_tempBuffers);

            vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            clusteringColors->SetNumberOfComponents(4);
            clusteringColors->SetName("Colors");

            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);

                unsigned char color[4] = {
                    h_tempBuffers.colors[i].x(),
                    h_tempBuffers.colors[i].y(),
                    h_tempBuffers.colors[i].z(),
                    255 };
                clusteringColors->InsertNextTypedTuple(color);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(clusteringColors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudClusteringActor->SetMapper(mapper);
            pointCloudClusteringActor->GetProperty()->SetPointSize(2);
            pointCloudClusteringActor->GetProperty()->SetRepresentationToPoints();
            pointCloudClusteringActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudClusteringActor);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();
        }
        */

        // Split
        /*
        {
            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SplitByNormal(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            d_tempBuffers.CopyTo(h_tempBuffers);

            vtkSmartPointer<vtkUnsignedCharArray> clusteringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            clusteringColors->SetNumberOfComponents(4);
            clusteringColors->SetName("Colors");

            auto vertices = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType i = 0; i < h_tempBuffers.numberOfPoints; ++i)
            {
                vtkIdType pid = i;
                vertices->InsertNextCell(1, &pid);

                unsigned char color[4] = {
                    h_tempBuffers.colors[i].x(),
                    h_tempBuffers.colors[i].y(),
                    h_tempBuffers.colors[i].z(),
                    255 };
                clusteringColors->InsertNextTypedTuple(color);
            }

            auto polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetVerts(vertices);
            polyData->GetPointData()->SetScalars(clusteringColors);
            polyData->GetPointData()->SetNormals(normals);

            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            mapper->SetScalarVisibility(true);

            pointCloudClusteringActor->SetMapper(mapper);
            pointCloudClusteringActor->GetProperty()->SetPointSize(2);
            pointCloudClusteringActor->GetProperty()->SetRepresentationToPoints();
            pointCloudClusteringActor->GetProperty()->SetLighting(false);

            app.GetRenderer()->AddActor(pointCloudClusteringActor);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();
        }
        */
    });

    app.Initialize();

    app.Run();

    pointCloud.Terminate();

    app.Terminate();

    return EXIT_SUCCESS;
}
