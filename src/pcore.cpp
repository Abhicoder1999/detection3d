#include "pcore.h"

Pcore::Pcore()
{}

Pcore::~Pcore()
{}

void Pcore::FilterCloud(PointXYZI::Ptr incloud, PointXYZI::Ptr outcloud, float leafsize)
{
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(incloud);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*outcloud);

}

void Pcore::CropCloud(PointXYZI::Ptr incloud, PointXYZI::Ptr outcloud, const Eigen::Vector4f minRange, const Eigen::Vector4f maxRange)
{
    pcl::CropBox<pcl::PointXYZI> region(true);
    region.setMin(minRange);
    region.setMax(maxRange);
    region.setInputCloud(incloud);
    region.filter(*outcloud);

}

// std::pair<PointXYZI::Ptr, PointXYZI::Ptr> Pcore::myRansacPlane(PointXYZI::Ptr incloud, int maxIterations, float distanceThreshold)
// {
//     // My implementation of RANSAC for segmenting planes.

//     PointXYZI::Ptr cloud_obstacle(new PointXYZI());
//     PointXYZI::Ptr cloud_road(new PointXYZI());

//     // reducing ROI for improvement in road plane finding
//     PointXYZI::Ptr cloud_temp(new PointXYZI);
//     Eigen::Vector4f minPoint(-2, -4, -1, 1);
//     Eigen::Vector4f maxPoint(3, 5, 1, 1);

//     pcl::CropBox<pcl::PointXYZI> region(true);
//     region.setMin(minPoint);
//     region.setMax(maxPoint);
//     region.setInputCloud(incloud);
//     region.filter(*cloud_temp);

//     std::unordered_set<int> inliersResult;
//     srand(time(NULL));

//     // TODO: Add permenant plane equation and not compute everytime!
//     while (maxIterations-- >= 0)
//     {
//         std::unordered_set<int> inliers_set;

//         while (inliers_set.size() < 3)
//             inliers_set.insert(rand() % (cloud_temp->points.size()));

//         float x1{}, x2{}, x3{}, y1{}, y2{}, y3{}, z1{}, z2{}, z3{}, a{}, b{}, c{}, d{};

//         auto itr = inliers_set.begin();
//         x1 = cloud_temp->points[*itr].x;
//         y1 = cloud_temp->points[*itr].y;
//         z1 = cloud_temp->points[*itr].z;
//         itr++;
//         x2 = cloud_temp->points[*itr].x;
//         y2 = cloud_temp->points[*itr].y;
//         z2 = cloud_temp->points[*itr].z;
//         itr++;
//         x3 = cloud_temp->points[*itr].x;
//         y3 = cloud_temp->points[*itr].y;
//         z3 = cloud_temp->points[*itr].z;

//         a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
//         b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
//         c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
//         d = -(a * x1 + b * y1 + c * z1);

//         inliers_set.clear();

//         for (int i = 0; i < incloud->points.size(); i++)
//         {
//             // if(inliers_set.count(i))
//             // 	continue;

//             float x0{}, y0{}, z0{}, dist{};
//             x0 = incloud->points[i].x;
//             y0 = incloud->points[i].y;
//             z0 = incloud->points[i].z;

//             dist = fabs((a * x0) + (b * y0) + (c * z0) + d) / sqrt((a * a) + (b * b) + (c * c));

//             if (dist <= distanceThreshold)
//                 inliers_set.insert(i);
//         }

//         if (inliers_set.size() > inliersResult.size())
//             inliersResult = inliers_set;
//     }

//     for (int index = 0; index < incloud->points.size(); index++)
//     {
//         pcl::PointXYZI point = incloud->points[index];
//         if (inliersResult.count(index))
//             cloud_road->points.push_back(point);
//         else
//             cloud_obstacle->points.push_back(point);
//     }

//     // std::cout<<"size of inobject cloud:"<<cloud_obstacle->points.size()<<"size of inobject road:"<<cloud_road->points.size()<<std::endl;
//     return std::pair<PointXYZI::Ptr, PointXYZI::Ptr>(cloud_obstacle, cloud_road);
// }

void Pcore::Clustering(PointXYZI::Ptr incloud, std::vector<PointXYZI::Ptr>& clusters, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(incloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(incloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices clust_indices : cluster_indices)
    {
        PointXYZI::Ptr cloud_cluster(new PointXYZI());

        for (auto i : clust_indices.indices)
            cloud_cluster->points.push_back(incloud->points[i]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

}
