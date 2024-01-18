#include <iostream>
#include "pcloudProcessor.h"
#include "rosEnv.h"

ros::Publisher pub1;
ros::Publisher pub2;
pcl::PCLPointCloud2 cloud_voxel, cloud_roi;
sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr road_cloud(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr filt_cloud(new sensor_msgs::PointCloud2);
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    // 1. Voxel reduction
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_voxel);

    // 2. ROI setting based on required area of supervision
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(cloud_voxel, *cloud_temp);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Vector4f minPoint(-10, -6, -1, 1);
    Eigen::Vector4f maxPoint(15, 7, 1, 1);

    pcl::CropBox<pcl::PointXYZI> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_temp);
    region.filter(*cloud_roi);

    // Perform RANSAC floor and obstacle detection
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pair_cloud = myRansacPlane(cloud_roi, 150, 0.15);

    // Visualization
    if (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        renderPointCloud(viewer, pair_cloud.first, "obstacle_cloud", Color(1, 0, 0));
        renderPointCloud(viewer, pair_cloud.second, "obstacle_road", Color(0,1,0));
        viewer->spinOnce();
    }


    // Convert to ROS data type
    pcl::toROSMsg(*pair_cloud.first, *obstacle_cloud);
    pcl::toROSMsg(*pair_cloud.second, *road_cloud);
    pcl::toROSMsg(*cloud_roi, *filt_cloud);

    road_cloud->header.frame_id = "os_sensor";
    obstacle_cloud->header.frame_id = "os_sensor";
    filt_cloud->header.frame_id = "os_sensor";

    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_voxel, output);

    // Publish the data
    pub1.publish(obstacle_cloud);
    pub2.publish(road_cloud);

    // pub1.publish(output);
    // pub2.publish(filt_cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection3d_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, cloud_cb);
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/output_check1", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/output_check2", 1);
    ros::spin();

    return 0;
}
