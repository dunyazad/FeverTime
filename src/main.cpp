#pragma warning(disable : 4819)

#include <App/App.h>
#include <App/Entity.h>

#include <CUDA/main.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Clustering.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_ClusteringFilter.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_CheckOverlap.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_FindSurfaceNeighbor.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Laplacian.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_NormalSimilarity.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Smoothing.cuh>

#include <Debugging/VisualDebugging.h>

bool operator == (uint3 a, uint3 b)
{
    if (a.x != b.x) return false;
    if (a.y != b.y) return false;
    if (a.z != b.z) return false;
    return true;
}

bool operator<(const uint3& a, const uint3& b) {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
}

ostream& operator<<(ostream& os, const uint3& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

pair<uint3, unsigned int> max1, max2;

void findTopTwo(const map<uint3, unsigned int>& colorHistogram) {
    max1.second = max2.second = 0;

    for (const auto& kv : colorHistogram) {
        if (kv.second > max1.second) {
            max2 = max1;
            max1 = kv;
        }
        else if (kv.second > max2.second) {
            max2 = kv;
        }
    }

    cout << "1st Max: " << max1.first << " -> " << max1.second << "\n";
    cout << "2nd Max: " << max2.first << " -> " << max2.second << "\n";
}


//const string resource_file_name = "Tooth";
const string resource_file_name = "BasePoints";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

string lscFile = "../../res/Upper.lsc";
string lspFile = "../../res/Upper.lsp";
string binFile = "../../res/data_2.bin";

struct HPatch
{
    size_t streamPos;
    Eigen::Matrix4f transform_0;
    Eigen::Matrix4f transform_45;
    Eigen::AlignedBox3f aabb_0;
    Eigen::AlignedBox3f aabb_45;
    bool aiMode;
    bool metalMode;
    bool softTissueMode;
    bool ICPSuccessAfterGlobal;
    unsigned short matchedStartPatchIDAfterGlobal;
};

int main(int argc, char** argv)
{
    App app;

    DevicePointCloud pointCloud;

    app.AddKeyPressCallback([&](App* app) {
        auto interactor = app->GetInteractor();
        std::string key = interactor->GetKeySym();

        std::cout << "[KEY] Pressed: " << key << std::endl;

        if (key == "Escape")
        {
            std::cout << "종료" << std::endl;
            interactor->GetRenderWindow()->Finalize();
            interactor->TerminateApp();
        }
        else if (key == "BackSpace")
        {
            VisualDebugging::Clear("Picked");
        }
        else if (key == "r")
        {
            std::cout << "R 키가 눌렸습니다. 카메라 리셋" << std::endl;
            if (app->GetRenderer())
            {
                app->GetRenderer()->ResetCamera();
                app->GetRenderer()->GetRenderWindow()->Render();
            }
        }
        else if (key == "grave")
        {
            app->GetActiveEntity()->ToggleLighting();
            app->GetRenderWindow()->Render();
        }
        else if (key == "equal")
        {
            app->GetActiveEntity()->IncreasePointSize();
            app->GetRenderWindow()->Render();
        }
        else if (key == "minus")
        {
            app->GetActiveEntity()->DecreasePointSize();
            app->GetRenderWindow()->Render();
        }
        else if (key == "Tab")
        {
            app->GetActiveEntity()->ToggleNormalVisibility();
            app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "space")
        {
            std::cout << "Space 키가 눌렸습니다." << std::endl;
            app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "Prior")
        {
            auto entity = app->GetActiveEntity();
            if (nullptr != entity)
            {
                if ("Clustering" == entity->GetName())
                {
                    //auto degree = app->GetClusteringDegree();
                    //degree += 1.0f;

                    //printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                    //app->SetClusteringDegree(degree);

                    //pointCloud->Clustering(degree);

                    //PointCloudBuffers d_tempBuffers;
                    //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    //pointCloud->SerializeColoringByLabel(d_tempBuffers);

                    //PointCloudBuffers h_tempBuffers;
                    //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    //d_tempBuffers.CopyTo(h_tempBuffers);

                    //entity->UpdateColorFromBuffer(h_tempBuffers);

                    //d_tempBuffers.Terminate();
                    //h_tempBuffers.Terminate();
                }
                else if ("Clustering Sub" == entity->GetName())
                {
                    //auto degree = app->GetClusteringDegree();
                    //degree += 1.0f;

                    //printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                    //app->SetClusteringDegree(degree);

                    //pointCloud->Clustering(degree);

                    //PointCloudBuffers d_tempBuffers;
                    //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    //pointCloud->SerializeColoringByLabel(d_tempBuffers);

                    //PointCloudBuffers h_tempBuffers;
                    //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    //d_tempBuffers.CopyTo(h_tempBuffers);

                    //entity->UpdateColorFromBuffer(h_tempBuffers);

                    //d_tempBuffers.Terminate();
                    //h_tempBuffers.Terminate();
                }
                /*
                else if ("Empty Neighbor Count" == entity->GetName())
                {
                    auto threshold = app->GetEmptyNeighborCountThreshold();
                    threshold += 1;

                    printf("Empty NeighborCountThreshold : %d\n", threshold);

                    app->SetEmptyNeighborCountThreshold(threshold);

                    PointCloudBuffers d_tempBuffers;
                    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    pointCloud->SerializeColoringByEmptyNeighborCount(threshold, d_tempBuffers);

                    PointCloudBuffers h_tempBuffers;
                    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    d_tempBuffers.CopyTo(h_tempBuffers);

                    entity->UpdateColorFromBuffer(h_tempBuffers);

                    d_tempBuffers.Terminate();
                    h_tempBuffers.Terminate();
                }
                else if ("Normal Discontinuity" == entity->GetName())
                {
                    auto threshold = app->GetNormalDiscontinuityThreshold();
                    threshold += 1.0f;

                    printf("Normal Discontinuity Threshold : %f = %f\n", threshold, threshold * M_PI / 180);

                    app->SetNormalDiscontinuityThreshold(threshold);

                    pointCloud->ComputeNormalDiscontinuity(threshold);

                    PointCloudBuffers d_tempBuffers;
                    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    pointCloud->SerializeColoringByNormalDiscontinuity(d_tempBuffers);

                    PointCloudBuffers h_tempBuffers;
                    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    d_tempBuffers.CopyTo(h_tempBuffers);

                    entity->UpdateColorFromBuffer(h_tempBuffers);

                    d_tempBuffers.Terminate();
                    h_tempBuffers.Terminate();
                }
                else if ("Normal Divergence" == entity->GetName())
                {
                    auto threshold = app->GetNormalDivergenceThreshold();
                    threshold += 0.01f;

                    printf("Normal Divergence Threshold : %f\n", threshold);

                    app->SetNormalDivergenceThreshold(threshold);

                    pointCloud->ComputeNormalDivergence();

                    PointCloudBuffers d_tempBuffers;
                    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    pointCloud->SerializeColoringByNormalDivergence(threshold, d_tempBuffers);

                    PointCloudBuffers h_tempBuffers;
                    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    d_tempBuffers.CopyTo(h_tempBuffers);

                    entity->UpdateColorFromBuffer(h_tempBuffers);

                    d_tempBuffers.Terminate();
                    h_tempBuffers.Terminate();
                }
                */
            }

            app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "Next")
        {
            auto entity = app->GetActiveEntity();
            if (nullptr != entity)
            {
                if ("Clustering" == entity->GetName())
                {
                    //auto degree = app->GetClusteringDegree();
                    //degree -= 1.0f;

                    //printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                    //app->SetClusteringDegree(degree);

                    //pointCloud->Clustering(degree);

                    //PointCloudBuffers d_tempBuffers;
                    //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    //pointCloud->SerializeColoringByLabel(d_tempBuffers);

                    //PointCloudBuffers h_tempBuffers;
                    //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    //d_tempBuffers.CopyTo(h_tempBuffers);

                    //entity->UpdateColorFromBuffer(h_tempBuffers);

                    //d_tempBuffers.Terminate();
                    //h_tempBuffers.Terminate();
                }
                else if ("Clustering Sub" == entity->GetName())
                {
                    //auto degree = app->GetClusteringDegree();
                    //degree -= 1.0f;
                    
                    //printf("Clustering Degree : %f = %f\n", degree, degree * M_PI / 180);

                    //app->SetClusteringDegree(degree);

                    //pointCloud->Clustering(degree);

                    //PointCloudBuffers d_tempBuffers;
                    //d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    //pointCloud->SerializeColoringByLabel(d_tempBuffers);

                    //PointCloudBuffers h_tempBuffers;
                    //h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    //d_tempBuffers.CopyTo(h_tempBuffers);

                    //entity->UpdateColorFromBuffer(h_tempBuffers);

                    //d_tempBuffers.Terminate();
                    //h_tempBuffers.Terminate();
                }
                /*
                else if ("Empty Neighbor Count" == entity->GetName())
                {
                    auto threshold = app->GetEmptyNeighborCountThreshold();
                    threshold -= 1;

                    printf("Empty NeighborCountThreshold : %d\n", threshold);

                    app->SetEmptyNeighborCountThreshold(threshold);

                    PointCloudBuffers d_tempBuffers;
                    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    pointCloud->SerializeColoringByEmptyNeighborCount(threshold, d_tempBuffers);

                    PointCloudBuffers h_tempBuffers;
                    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    d_tempBuffers.CopyTo(h_tempBuffers);

                    entity->UpdateColorFromBuffer(h_tempBuffers);

                    d_tempBuffers.Terminate();
                    h_tempBuffers.Terminate();
                }
                else if("Normal Discontinuity" == entity->GetName())
                {
                    auto threshold = app->GetNormalDiscontinuityThreshold();
                    threshold -= 1.0f;

                    printf("Normal Discontinuity Threshold : %f = %f\n", threshold, threshold* M_PI / 180);

                    app->SetNormalDiscontinuityThreshold(threshold);

                    pointCloud->ComputeNormalDiscontinuity(threshold);

                    PointCloudBuffers d_tempBuffers;
                    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    pointCloud->SerializeColoringByNormalDiscontinuity(d_tempBuffers);

                    PointCloudBuffers h_tempBuffers;
                    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    d_tempBuffers.CopyTo(h_tempBuffers);

                    entity->UpdateColorFromBuffer(h_tempBuffers);

                    d_tempBuffers.Terminate();
                    h_tempBuffers.Terminate();
                }
                else if ("Normal Divergence" == entity->GetName())
                {
                    auto threshold = app->GetNormalDivergenceThreshold();
                    threshold -= 0.01f;

                    printf("Normal Divergence Threshold : %f\n", threshold);

                    app->SetNormalDivergenceThreshold(threshold);

                    pointCloud->ComputeNormalDivergence();

                    PointCloudBuffers d_tempBuffers;
                    d_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), false);

                    pointCloud->SerializeColoringByNormalDivergence(threshold, d_tempBuffers);

                    PointCloudBuffers h_tempBuffers;
                    h_tempBuffers.Initialize(pointCloud->GetNumberOfPoints(), true);
                    d_tempBuffers.CopyTo(h_tempBuffers);

                    entity->UpdateColorFromBuffer(h_tempBuffers);

                    d_tempBuffers.Terminate();
                    h_tempBuffers.Terminate();
                }
                */
            }

            app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "Left")
        {
            app->DecreaseActiveEntityIndex();
            app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "Right")
        {
            app->IncreaseActiveEntityIndex();
            app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "3")
        {
            //this->SetAbortFlag(1);
        }
    });

    app.AddMouseButtonReleaseCallback([&](App* app, int buttonIndex) {
        if (0 == buttonIndex)
        {
            auto interactor = app->GetInteractor();
            auto renderer = app->GetRenderer();
            vtkCamera* camera = app->GetRenderer()->GetActiveCamera();

            int* clickPos = interactor->GetEventPosition();

            vtkSmartPointer<vtkCoordinate> coordinate = vtkSmartPointer<vtkCoordinate>::New();
            coordinate->SetCoordinateSystemToDisplay();
            coordinate->SetValue(clickPos[0], clickPos[1], 0.0);
            double* world = coordinate->GetComputedWorldValue(renderer);

            double cameraPos[3];
            renderer->GetActiveCamera()->GetPosition(cameraPos);

            Eigen::Vector3d origin(cameraPos);
            Eigen::Vector3d pick(world);
            Eigen::Vector3d dir = (pick - origin).normalized();

            std::cout << "Ray: origin = " << origin.transpose()
                << ", direction = " << dir.transpose() << std::endl;

            auto index = pointCloud.Pick(
                make_float3(cameraPos[0], cameraPos[1], cameraPos[2]),
                make_float3(dir[0], dir[1], dir[2]));

            if (-1 != index)
            {
                thrust::host_vector<float3> positions(pointCloud.GetPositions());
                thrust::host_vector<float3> normals(pointCloud.GetNormals());
                thrust::host_vector<uchar4> colors(pointCloud.GetColors());
                auto pickedPosition = positions[index];
                auto pickedNormal = normals[index];
                auto pickedColor = colors[index];

                //VisualDebugging::AddLine("Picked",
                //    { (float)cameraPos[0], (float)cameraPos[1], (float)cameraPos[2] },
                //    {
                //        (float)cameraPos[0] + (float)dir[0] * 250.0f,
                //        (float)cameraPos[1] + (float)dir[1] * 250.0f,
                //        (float)cameraPos[2] + (float)dir[2] * 250.0f
                //    }, Color4::White);

                //VisualDebugging::AddSphere("Picked",
                //    { pickedPosition.x, pickedPosition.y, pickedPosition.z },
                //    { 0.05f, 0.05f, 0.05f },
                //    Eigen::Vector3f::UnitZ(),
                //    Color4::Red);

                VisualDebugging::AddPlane("Picked",
                    { pickedPosition.x, pickedPosition.y, pickedPosition.z },
                    { 0.5f, 0.5f, 0.5f }, 
                    { pickedNormal.x, pickedNormal.y, pickedNormal.z },
                    Color4(pickedNormal.x * 255.0f, pickedNormal.y * 255.0f, pickedNormal.z * 255.0f, 255));

                camera->SetFocalPoint(pickedPosition.x, pickedPosition.y, pickedPosition.z);

                //app->GetActiveEntity()->UpdateColorFromBuffer(&pointCloud);

                app->GetRenderer()->ResetCameraClippingRange();
                app->GetRenderer()->GetRenderWindow()->Render();
            }
        }
        });

    app.AddAppStartCallback([&](App* app) {
        VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 100.0f * 0.5f, 0.0f, 0.0f }, Color4::Red);
        VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 100.0f * 0.5f, 0.0f }, Color4::Green);
        VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 0.0f, 100.0f * 0.5f }, Color4::Blue);

        app->GetRenderer()->SetBackground(0.3, 0.5, 0.7);

        /*
        {
            FILE* fp = NULL;
            fopen_s(&fp, lspFile.c_str(), "rb");

            int iFormatVersionMajor = -1;
            int iFormatVersionMinor = -1;
            int iFormatVersionPatch = -1;

            if (fp != NULL) {
                fread(&iFormatVersionMajor, sizeof(int), 1, fp);
                fread(&iFormatVersionMinor, sizeof(int), 1, fp);
                fread(&iFormatVersionPatch, sizeof(int), 1, fp);

                //printf("%d, %d, %d\n", iFormatVersionMajor, iFormatVersionMinor, iFormatVersionPatch);

                //unsigned long ulVertexCount = 0;
                //unsigned long ulPointIndexCount = 0;
                //if (fread(&ulVertexCount, sizeof(unsigned long), 1, fp) != 1) return;
                //if (fread(&ulPointIndexCount, sizeof(unsigned long), 1, fp) != 1) return;

                //printf("ulVertexCount : %llu, ulPointIndexCount : %llu\n", ulVertexCount, ulPointIndexCount);

                unsigned int iPatchCount = 0;
                fread(&iPatchCount, sizeof(int), 1, fp);

                printf("iPatchCount : %d\n", iPatchCount);

                for (size_t i = 0; i < iPatchCount; i++)
                {
                    size_t streamPos;
                    fread(&streamPos, sizeof(size_t), 1, fp);

                    printf("streamPos : %llu\n", streamPos);

                    Eigen::Matrix4f transform_0;
                    fread(&transform_0, sizeof(float) * 16, 1, fp);

                    Eigen::Matrix4f transform_45;
                    fread(&transform_45, sizeof(float) * 16, 1, fp);

                    Eigen::Vector3f min_0, max_0;
                    fread(&min_0, sizeof(float) * 3, 1, fp);
                    fread(&max_0, sizeof(float) * 3, 1, fp);

                    printf("%.4f, %.4f, %.4f\n", min_0.x(), min_0.y(), min_0.z());

                    Eigen::Vector3f min_45, max_45;
                    fread(&min_45, sizeof(float) * 3, 1, fp);
                    fread(&max_45, sizeof(float) * 3, 1, fp);

                    Eigen::AlignedBox3f aabb_0;
                    Eigen::AlignedBox3f aabb_45;

                    bool aiMode;
                    fread(&aiMode, sizeof(bool), 1, fp);

                    bool metalMode;
                    fread(&metalMode, sizeof(bool), 1, fp);

                    bool softTissueMode;
                    fread(&softTissueMode, sizeof(bool), 1, fp);

                    bool ICPSuccessAfterGlobal;
                    fread(&ICPSuccessAfterGlobal, sizeof(bool), 1, fp);

                    unsigned short matchedStartPatchIDAfterGlobal;
                    fread(&matchedStartPatchIDAfterGlobal, sizeof(unsigned short), 1, fp);
                }

                for (size_t i = 0; i < iPatchCount; i++)
                {
                    int imageDataSize = 0;
                    size_t readCount = fread(&imageDataSize, sizeof(imageDataSize), 1, fp);

                    printf("imageDataSize : %d\n", imageDataSize);
                }
                //for (int i = 0; i < iPatchCount; i++) {
                //    HPatch patch;
                //    patchLoader.LoadPatchInfo_1_0_3(patchInfoFp, patch);
                //    kpPatchGroup->AddPatch(std::move(patch));
                //}
                //fclose(patchInfoFp);
            }
        }

        return;
        */

        //auto roi = Eigen::AlignedBox3f(Eigen::Vector3f(0.0f, -60.0f, -5.0f), Eigen::Vector3f(20.0f, -30.0f, 25.0f));
        auto roi = Eigen::AlignedBox3f(Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX), Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX));

        if (false == pointCloud.LoadFromALP(resource_file_name_alp, roi))
        {
            if (false == pointCloud.LoadFromPLY(resource_file_name_ply, roi))
            {
                return;
            }
            else
            {
                pointCloud.SaveToALP(resource_file_name_alp);
            }
        }

        auto defaultEntity = app->CreateEntity("Default");
        //defaultEntity->FromPointCloud(&pointCloud, roi);
        defaultEntity->FromPointCloud(&pointCloud);

        app->GetRenderer()->ResetCamera();
        app->GetRenderWindow()->Render();
        
        
        {
            DevicePointCloud pcd;

            pcd.LoadFromALP(resource_file_name_alp);

            PointCloudAlgorithm_FindSurfaceNeighbor algorithm;
            algorithm.SetRemoveCheckedPoints(false);
            algorithm.RunAlgorithm(&pcd);

            pcd.Compact();

            auto entity = app->CreateEntity("Find Surface Neighbor");
            entity->FromPointCloud(&pcd);

            entity->SetVisibility(false);



            //auto roi = Eigen::AlignedBox3f(Eigen::Vector3f(0.0f, -60.0f, -5.0f), Eigen::Vector3f(20.0f, -30.0f, 25.0f));
            //pcd.LoadFromPLY(resource_file_name_ply, roi);
            //pcd.LoadFromPLY(resource_file_name_ply, roi);

            //pcd.GetHashMap().SerializeToPLY("C:\\Resources\\Debug\\HashMap.ply");

            //pcd.SaveToPLY("C:\\Resources\\Debug\\Test.ply");
            //pcd.SaveToALP(resource_file_name_alp);
        }

        return;

        {
            DevicePointCloud pcd;

            pcd.LoadFromALP(resource_file_name_alp);

            {
                PointCloudAlgorithm_NormalSimilarity_UsingCount algorithm;
                algorithm.SetRemoveCheckedPoints(false);

                algorithm.RunAlgorithm(&pcd);
            }

            auto entity = app->CreateEntity("Clustering");
            entity->FromPointCloud(&pcd);

            entity->SetVisibility(false);
        }


        //{
        //    DevicePointCloud pcd;

        //    pcd.LoadFromALP(resource_file_name_alp);

        //    {
        //        PointCloudAlgorithm_CheckOverlap algorithm;
        //        algorithm.SetStep(3);
        //        algorithm.SetRemoveCheckedPoints(false);
        //        algorithm.RunAlgorithm(&pcd);
        //    }

        //    auto entity = app->CreateEntity("Check Overlap");
        //    entity->FromPointCloud(&pcd);

        //    entity->SetVisibility(false);
        //}

        {
            DevicePointCloud pcd;

            pcd.LoadFromALP(resource_file_name_alp);

            TS(Cleaning);
            {
                PointCloudAlgorithm_ClusteringFilter algorithm;
                algorithm.SetApplyColor(false);
                algorithm.SetRemoveCheckedPoints(true);
                algorithm.RunAlgorithm(&pcd);
            }
            {
                PointCloudAlgorithm_CheckOverlap algorithm;
                algorithm.SetStep(7);
                algorithm.SetRemoveCheckedPoints(true);
                algorithm.RunAlgorithm(&pcd);
            }
            {
                PointCloudAlgorithm_NormalSimilarity_UsingCount algorithm;
                algorithm.SetRemoveCheckedPoints(true);

                algorithm.RunAlgorithm(&pcd);
            }
            {
                PointCloudAlgorithm_ClusteringFilter algorithm;
                algorithm.SetApplyColor(false);
                algorithm.SetRemoveCheckedPoints(true);
                algorithm.RunAlgorithm(&pcd);
            }
            TE(Cleaning);

            //pcd.SaveToPLY("D:\\Debug\\3D\\points_26.ply");

            auto entity = app->CreateEntity("Cleaning");
            entity->FromPointCloud(&pcd);

            entity->SetVisibility(false);
        }
    });

    app.Run();

    return EXIT_SUCCESS;
}
