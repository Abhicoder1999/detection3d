#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <iostream>
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcore.h"


using namespace std;


struct Color
{

	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{
	}
};

struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};


class PcloudProcessor
{
	ros::Publisher pubObstacleCloud_;
    ros::Publisher pubPlaneCloud_;
    ros::Subscriber subLidar_;
	pcl::visualization::PCLVisualizer::Ptr viewer_;
	Pcore pcore_;

	double ground_clearance = 0.04;
	Eigen::Vector4f minRange;
	Eigen::Vector4f maxRange;

	

	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
	std::pair<PointXYZI::Ptr, PointXYZI::Ptr> RansacPlane(PointXYZI::Ptr cloud, int maxIterations, float distanceThreshold);
	std::vector<PointXYZI::Ptr> Clustering(PointXYZI::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
	void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id, Color color, float opacity);
	Box BoundingBox(PointXYZI::Ptr cluster);
	void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const PointXYZI::Ptr &cloud, std::string name, Color color);


public:
	// constructor
	PcloudProcessor(ros::NodeHandle& nh);
	// deconstructor
	~PcloudProcessor();
};
