#include "pcloudProcessor.h"

PcloudProcessor::PcloudProcessor(ros::NodeHandle &nh) : viewer_(new pcl::visualization::PCLVisualizer("3D viewer_")), pcore_()
{
    subLidar_ = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &PcloudProcessor::cloud_cb, this);
    pubObstacleCloud_ = nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/obstacle_cloud", 1);
    pubPlaneCloud_ = nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/road_cloud", 1);

    double ground_clearance = 0.04;
    minRange = Eigen::Vector4f(-10, -6, -1, 1);
    maxRange = Eigen::Vector4f(15, 5, 3, 1);
    bool visualise = true;
    ROS_INFO("Initialization done");
}

PcloudProcessor::~PcloudProcessor()
{
}

void PcloudProcessor::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Convert to PCL data type
    pcl::PCLPointCloud2 cloud_temp;
    pcl_conversions::toPCL(*cloud_msg, cloud_temp);
    PointXYZI::Ptr cloud(new PointXYZI);
    pcl::fromPCLPointCloud2(cloud_temp, *cloud);

    // Preprocessing
    PointXYZI::Ptr cloud_voxel(new PointXYZI);
    pcore_.FilterCloud(cloud, cloud_voxel, 0.1);
    pcore_.CropCloud(cloud_voxel, cloud_voxel, minRange, maxRange);

    // Perform RANSAC floor and obstacle detection
    std::pair<PointXYZI::Ptr, PointXYZI::Ptr> pair_cloud = pcore_.RansacPlane(cloud_voxel, 150, 0.15);

    // Clustering
    std::vector<PointXYZI::Ptr> cloudClusters;
    pcore_.Clustering(pair_cloud.first, cloudClusters, 0.3, 50, 5000);

    int clusterID{};
    int cluster_size = 0;
    std::vector<Color> colors{Color(1, 0, 0), Color(1, 0, 1), Color(0, 0, 1)};

    if (!viewer_->wasStopped() && visualise)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        renderPointCloud(viewer_, pair_cloud.first, "obstacle_cloud", Color(1, 0, 0));
        renderPointCloud(viewer_, pair_cloud.second, "obstacle_road", Color(0, 1, 0));
    }

    for (PointXYZI::Ptr cluster : cloudClusters)
    {
        renderPointCloud(viewer_, cluster, "obstacle_cloud" + std::to_string(clusterID), colors[clusterID % 3]);
        Box box = BoundingBox(cluster);
        ++clusterID;

        if (!viewer_->wasStopped() && visualise)
        {
            renderBox(viewer_, box, clusterID, colors[clusterID % 3], 1);
            viewer_->spinOnce(150);
        }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr road_cloud(new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*pair_cloud.first, *obstacle_cloud);
    pcl::toROSMsg(*pair_cloud.second, *road_cloud);

    road_cloud->header.frame_id = "os_sensor";
    obstacle_cloud->header.frame_id = "os_sensor";

    // Publish the data
    pubObstacleCloud_.publish(obstacle_cloud);
    pubPlaneCloud_.publish(road_cloud);
}

Box PcloudProcessor::BoundingBox(PointXYZI::Ptr cluster)
{

    // Oriented Bounding Box for Object dimension measurements
    pcl::PointXYZI ObbminPoint, ObbmaxPoint, translation;
    Eigen::Matrix3f rotation;
    pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cluster);
    feature_extractor.compute();
    feature_extractor.getOBB(ObbminPoint, ObbmaxPoint, translation, rotation);

    // For Axis Aligned Bounding Box for rendering
    pcl::PointXYZI AabbminPoint, AabbmaxPoint;
    feature_extractor.getAABB(AabbminPoint, AabbmaxPoint);
    Box box;

    box.x_min = AabbminPoint.x;
    box.y_min = AabbminPoint.y;
    box.z_min = AabbminPoint.z;
    box.x_max = AabbmaxPoint.x;
    box.y_max = AabbmaxPoint.y;
    box.z_max = AabbmaxPoint.z;

    box.length = ObbminPoint.x - ObbmaxPoint.x;
    box.width = ObbminPoint.y - ObbmaxPoint.y;
    box.height = ObbminPoint.z - ObbmaxPoint.z;

    return box;
}

void PcloudProcessor::renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer_, Box box, int id, Color color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    viewer_->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    viewer_->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
}

void PcloudProcessor::renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer_, const PointXYZI::Ptr &cloud, std::string name, Color color)
{

    // Select color based off input value
    viewer_->addPointCloud<pcl::PointXYZI>(cloud, name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}
