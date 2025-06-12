#pragma warning(disable : 4819)

#include <App/App.h>
#include <App/Entity.h>

#include <CUDA/main.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_Clustering.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_ClusteringFilter.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_CheckOverlap.cuh>
#include <CUDA/PointCloudAlgorithms/PointCloudAlgorithm_FindSurfaceNeighbor.cuh>
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


//const string resource_file_name = "Compound_Full";
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
            //app->DecreaseActiveEntityIndex();
            //app->GetRenderer()->GetRenderWindow()->Render();
        }
        else if (key == "Right")
        {
            //app->IncreaseActiveEntityIndex();
            //app->GetRenderer()->GetRenderWindow()->Render();
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
                auto pickedNormal = normals[index] ;
                auto pickedColor = colors[index];

                VisualDebugging::Clear("Picked");
                VisualDebugging::AddLine("Picked",
                    { (float)cameraPos[0], (float)cameraPos[1], (float)cameraPos[2] },
                    {
                        (float)cameraPos[0] + (float)dir[0] * 250.0f,
                        (float)cameraPos[1] + (float)dir[1] * 250.0f,
                        (float)cameraPos[2] + (float)dir[2] * 250.0f
                    }, Color4::White);

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

                app->GetActiveEntity()->UpdateColorFromBuffer(&pointCloud);

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

            algorithm.RunAlgorithm(&pcd);

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
        

        /*
        {
            DevicePointCloud pcd;

            pcd.LoadFromALP(resource_file_name_alp);

            PointCloudAlgorithm_ClusteringFilter clusteringFilter;

            clusteringFilter.RunAlgorithm(&pcd);

            pcd.Compact();

            auto entity = app.CreateEntity("Clustering Filter");
            entity->FromPointCloud(&pcd);

            entity->SetVisibility(false);
        }
        */

        /*
        {
            DevicePointCloud pcd;

            pcd.LoadFromALP(resource_file_name_alp);

            PointCloudAlgorithm_ClusteringFilter clusteringFilter;
            clusteringFilter.SetApplyColor(false);
            clusteringFilter.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step1.ply");

            PointCloudAlgorithm_CheckOverlap checkOverlap;
            checkOverlap.SetApplyColor(false);
            checkOverlap.RunAlgorithm(&pcd);

            pcd.Compact();

            pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_A.ply");

            PointCloudAlgorithm_Smoothing smoothing;
            for (size_t i = 0; i < 10; i++)
            {
                smoothing.RunAlgorithm(&pcd);
            }

            pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_B.ply");

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step2.ply");

            checkOverlap.SetStep(5);
            checkOverlap.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step3.ply");

            clusteringFilter.SetApplyColor(false);
            clusteringFilter.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step4.ply");

            checkOverlap.SetStep(10);
            checkOverlap.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step5.ply");

            clusteringFilter.SetApplyColor(false);
            clusteringFilter.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step6.ply");

            checkOverlap.SetStep(50);
            checkOverlap.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step7.ply");

            clusteringFilter.SetApplyColor(false);
            clusteringFilter.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step8.ply");

            checkOverlap.SetStep(100);
            checkOverlap.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step9.ply");

            clusteringFilter.SetApplyColor(false);
            clusteringFilter.RunAlgorithm(&pcd);

            pcd.Compact();

            //pcd.SaveToPLY("D:\\Resources\\Debug\\HD\\BasePoints_Step10.ply");

            auto entity = app.CreateEntity("Check Overlap");
            entity->FromPointCloud(&pcd);

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Clustering");
            entity->CopyFrom(defaultEntity);

            auto clusteringResult = pointCloud.Clustering(25.0f);
            map<unsigned int, unsigned int> labelCounts;
            map<unsigned int, unsigned int> subLabelCounts;
            for (auto& lc : clusteringResult)
            {
                labelCounts[lc.x] = lc.y;
                subLabelCounts[lc.x] = lc.z;
            }

            //for (auto& kvp : labelCounts)
            //{
            //    printf("[%3d] : %5d\n", kvp.first, kvp.second);
            //}

            //printf("=================================================\n");

            //for (auto& kvp : labelCounts)
            //{
            //    printf("[%3d] : %5d\n", kvp.first, kvp.second);
            //}

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByLabel(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            pointCloud.GetHashMap().SerializeSDFToPLY("C:\\Resources\\Debug\\Serialized\\SDF.ply");
        }
        */

        //pointCloud.ComputeVoxelNormalPCA();
        //pointCloud.ComputeVoxelNormalAverage();

        /*
        {
            auto entity = app.CreateEntity("Check Overlap");

            PointCloudAlgorithm_CheckOverlap checkOverlap;

            //PointCloud tempPointCloud;
            //tempPointCloud.Initialize(pointCloud.GetNumberOfPoints());

            //pointCloud.CopyTo(tempPointCloud);

            //checkOverlap.RunAlgorithm(&tempPointCloud);

            ////pointCloud.SerializeVoxelsColoringByLabel(tempPointCloud.GetDeviceBuffers());

            //entity->FromPointCloudBuffers(&tempPointCloud.GetDeviceBuffers());

            //tempPointCloud.Terminate();

            checkOverlap.RunAlgorithm(&pointCloud);

            //PointCloud tempPointCloud;
            //tempPointCloud.Initialize(pointCloud.GetHashMap().info.h_numberOfOccupiedVoxels);

            //pointCloud.SerializeVoxels(tempPointCloud.GetDeviceBuffers());

            //pointCloud.DtoH();

            entity->FromPointCloudBuffers(&pointCloud.GetDeviceBuffers());

            //auto voxelsEntity = app.CreateEntity("Check Overlap Voxels");
            //voxelsEntity->FromPointCloud(&tempPointCloud);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Result_ComputeTSDF");

            PointCloudAlgorithm_ComputeTSDF computeTSDF;

            PointCloud tempPointCloud;
            tempPointCloud.Initialize(pointCloud.GetNumberOfPoints() * 10);

            pointCloud.GetDeviceBuffers().CopyTo(tempPointCloud.GetDeviceBuffers());

            computeTSDF.RunAlgorithm(&tempPointCloud);

            //pointCloud.SerializeVoxelsColoringByLabel(tempPointCloud.GetDeviceBuffers());

            entity->FromPointCloudBuffers(&tempPointCloud.GetDeviceBuffers());

            tempPointCloud.Terminate();
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Empty Neighbor Count");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeEmptyNeighborCount();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByEmptyNeighborCount(20, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Clustering");
            entity->CopyFrom(defaultEntity);

            auto clusteringResult = pointCloud.Clustering(10.0f);
            map<unsigned int, unsigned int> labelCounts;
            map<unsigned int, unsigned int> subLabelCounts;
            for (auto& lc : clusteringResult)
            {
                labelCounts[lc.x] = lc.y;
                subLabelCounts[lc.x] = lc.z;
            }

            //for (auto& kvp : labelCounts)
            //{
            //    printf("[%3d] : %5d\n", kvp.first, kvp.second);
            //}

            //printf("=================================================\n");

            //for (auto& kvp : labelCounts)
            //{
            //    printf("[%3d] : %5d\n", kvp.first, kvp.second);
            //}

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByLabel(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Clustering Sub");
            entity->CopyFrom(defaultEntity);

            pointCloud.Clustering(10.0f);

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringBySubLabel(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Normal Discontinuity");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeNormalDiscontinuity();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNormalDiscontinuity(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Normal Divergence");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeNormalDivergence();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNormalDivergence(1.0f, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Normal Gradient");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeNormalGradient();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNormalGradient(0.025f, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers);

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Neighbor Count");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeNeighborCount();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByNeighborCount(d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto entity = app.CreateEntity("Color Mutiplication");
            entity->CopyFrom(defaultEntity);

            pointCloud.ComputeColorMultiplication();

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeColoringByColorMultiplication(400, d_tempBuffers);

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);
            d_tempBuffers.CopyTo(h_tempBuffers);

            entity->UpdateColorFromBuffer(h_tempBuffers); // Add this function in Entity

            d_tempBuffers.Terminate();
            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto distance =
                [&](const uint3& a, const uint3& b) {
                return std::sqrt(
                    static_cast<float>(a.x - b.x) * (a.x - b.x) +
                    static_cast<float>(a.y - b.y) * (a.y - b.y) +
                    static_cast<float>(a.z - b.z) * (a.z - b.z)
                );
            };

            auto average =
                [&](const std::vector<uint3>& points) -> uint3 {
                if (points.empty()) return { 0, 0, 0 };
                unsigned long long sx = 0, sy = 0, sz = 0;
                for (const auto& p : points) {
                    sx += p.x; sy += p.y; sz += p.z;
                }
                return {
                    static_cast<unsigned int>(sx / points.size()),
                    static_cast<unsigned int>(sy / points.size()),
                    static_cast<unsigned int>(sz / points.size())
                };
            };

            auto kmeans =
                [&](const std::vector<uint3>& points, int K, int iterations,
                    std::vector<int>& labels, std::vector<uint3>& centroids) {
                        const int N = points.size();
                        labels.resize(N);
                        centroids.resize(K);

                        // Initialize centroids randomly
                        for (int k = 0; k < K; ++k)
                            centroids[k] = points[rand() % N];

                        for (int iter = 0; iter < iterations; ++iter) {
                            // Assign points to the nearest centroid
                            for (int i = 0; i < N; ++i) {
                                float minDist = std::numeric_limits<float>::max();
                                int bestCluster = 0;
                                for (int k = 0; k < K; ++k) {
                                    float d = distance(points[i], centroids[k]);
                                    if (d < minDist) {
                                        minDist = d;
                                        bestCluster = k;
                                    }
                                }
                                labels[i] = bestCluster;
                            }

                            // Recalculate centroids
                            std::vector<std::vector<uint3>> clusters(K);
                            for (int i = 0; i < N; ++i)
                                clusters[labels[i]].push_back(points[i]);

                            for (int k = 0; k < K; ++k)
                                centroids[k] = average(clusters[k]);
                        }
                };

            vector<uint3> inputColors;
            for (size_t i = 0; i < pointCloud.GetNumberOfPoints(); i++)
            {
                auto c = pointCloud.GetHostBuffers().colors[i];
                inputColors.push_back(make_uint3(c.x(), c.y(), c.z()));
            }

            vector<int> labels;
            vector<uint3> centroids;
            kmeans(inputColors, 2, 10, labels, centroids);


            auto entity = app.CreateEntity("K-Means");
            entity->CopyFrom(defaultEntity);

            // 클러스터 색상 지정
            std::vector<uint3> clusterColors = {
                make_uint3(255, 0, 0),   // Cluster 0: Red
                make_uint3(0, 255, 0),   // Cluster 1: Green
            };

            PointCloudBuffers h_tempBuffers;
            h_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), true);

            for (size_t i = 0; i < pointCloud.GetNumberOfPoints(); ++i) {
                uint3 c = clusterColors[labels[i]];
                h_tempBuffers.colors[i].x() = c.x;
                h_tempBuffers.colors[i].y() = c.y;
                h_tempBuffers.colors[i].z() = c.z;
            }

            entity->UpdateColorFromBuffer(h_tempBuffers);

            h_tempBuffers.Terminate();

            entity->SetVisibility(false);
        }
        */

        /*
        {
            auto colorBinarization = app.CreateEntity("Color Binarization");
            entity->CopyTo(colorBinarization);

            colorBinarization->SetVisibility(false);
        }
        */
  
        /*
        auto hashToFloat = [](uint32_t seed) -> float {
            seed ^= seed >> 13;
            seed *= 0x5bd1e995;
            seed ^= seed >> 15;
            return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
        };
        */

        /*
        {
            auto entity = app.CreateEntity("Voxels");

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeVoxelsColoringByLabel(d_tempBuffers);

            entity->FromPointCloudBuffers(&d_tempBuffers);

            d_tempBuffers.Terminate();
        }
        */
    });

    app.Run();

    return EXIT_SUCCESS;
}
