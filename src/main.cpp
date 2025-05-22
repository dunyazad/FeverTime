#pragma warning(disable : 4819)

#include "App.h"
#include <main.cuh>

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
const string resource_file_name = "Serialized";
const string resource_file_name_ply = "../../res/3D/" + resource_file_name + ".ply";
const string resource_file_name_alp = "../../res/3D/" + resource_file_name + ".alp";

int main(int argc, char** argv)
{
    App app;

    PointCloud pointCloud;

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
            app.GetInteractor()->AddObserver(vtkCommand::KeyPressEvent, keyCallback);
        }

        app.GetRenderer()->SetBackground(0.3, 0.5, 0.7);

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

        //map<uint3, unsigned int> colorHistogram;
        //for (size_t i = 0; i < pointCloud.GetNumberOfPoints(); i++)
        //{
        //    auto& c = pointCloud.GetHostBuffers().colors[i];
        //    colorHistogram[make_uint3(c.x(), c.y(), c.z())]++;
        //}

        //for (auto& kvp : colorHistogram)
        //{
        //    //if (1 < kvp.second)
        //    {
        //        printf("[%3d, %3d, %3d] : %d\n", kvp.first.x, kvp.first.y, kvp.first.z, kvp.second);
        //    }
        //}

        auto defaultEntity = app.CreateEntity("Default");
        //defaultEntity->FromPointCloud(&pointCloud, roi);
        defaultEntity->FromPointCloud(&pointCloud);

        app.GetRenderer()->ResetCamera();
        app.GetRenderWindow()->Render();

        //pointCloud.ComputeVoxelNormalPCA();
        pointCloud.ComputeVoxelNormalAverage();

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

            for (auto& kvp : labelCounts)
            {
                printf("[%3d] : %5d\n", kvp.first, kvp.second);
            }

            printf("=================================================\n");

            for (auto& kvp : labelCounts)
            {
                printf("[%3d] : %5d\n", kvp.first, kvp.second);
            }

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
        
        {
            auto entity = app.CreateEntity("Voxels");

            PointCloudBuffers d_tempBuffers;
            d_tempBuffers.Initialize(pointCloud.GetNumberOfPoints(), false);

            pointCloud.SerializeVoxelsColoringByLabel(d_tempBuffers);

            entity->FromPointCloudBuffers(&d_tempBuffers);

            d_tempBuffers.Terminate();
        }

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
    });

    app.Initialize();

    app.Run();

    pointCloud.Terminate();

    app.Terminate();

    return EXIT_SUCCESS;
}
