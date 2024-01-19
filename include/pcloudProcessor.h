#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
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

	float length,width,height;
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
	bool visualise;

	
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
	void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id, Color color, float opacity);
	Box BoundingBox(PointXYZI::Ptr cluster);
	Box BoundingBox2(PointXYZI::Ptr cluster);
	void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const PointXYZI::Ptr &cloud, std::string name, Color color);


public:
	// constructor
	PcloudProcessor(ros::NodeHandle& nh);
	// deconstructor
	~PcloudProcessor();
};
