
// #ifndef PROCESSPOINTCLOUDS_H_
// #define PROCESSPOINTCLOUDS_H_

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
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
double ground_clearance = 0.04;

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

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> myRansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// My implementation of RANSAC for segmenting planes.

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZI>());

	// reducing ROI for improvement in road plane finding
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZI>);

	Eigen::Vector4f minPoint(-2, -4, -1, 1);
	Eigen::Vector4f maxPoint(3, 5, 1, 1);

	pcl::CropBox<pcl::PointXYZI> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloud);
	region.filter(*cloud_temp2);

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Add permenant plane equation and not compute everytime!

	while (maxIterations-- >= 0)
	{
		std::unordered_set<int> inliers_set;

		while (inliers_set.size() < 3)
			inliers_set.insert(rand() % (cloud_temp2->points.size()));

		float x1{}, x2{}, x3{}, y1{}, y2{}, y3{}, z1{}, z2{}, z3{}, a{}, b{}, c{}, d{};

		auto itr = inliers_set.begin();
		x1 = cloud_temp2->points[*itr].x;
		y1 = cloud_temp2->points[*itr].y;
		z1 = cloud_temp2->points[*itr].z;
		itr++;
		x2 = cloud_temp2->points[*itr].x;
		y2 = cloud_temp2->points[*itr].y;
		z2 = cloud_temp2->points[*itr].z;
		itr++;
		x3 = cloud_temp2->points[*itr].x;
		y3 = cloud_temp2->points[*itr].y;
		z3 = cloud_temp2->points[*itr].z;

		a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		d = -(a * x1 + b * y1 + c * z1);

		inliers_set.clear();

		for (int i = 0; i < cloud->points.size(); i++)
		{
			// if(inliers_set.count(i))
			// 	continue;

			float x0{}, y0{}, z0{}, dist{};
			x0 = cloud->points[i].x;
			y0 = cloud->points[i].y;
			z0 = cloud->points[i].z;

			dist = fabs((a * x0) + (b * y0) + (c * z0) + d) / sqrt((a * a) + (b * b) + (c * c));

			if (dist <= distanceThreshold)
				inliers_set.insert(i);
		}

		if (inliers_set.size() > inliersResult.size())
			inliersResult = inliers_set;
	}

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if (inliersResult.count(index))
			cloud_road->points.push_back(point);
		else
			cloud_obstacle->points.push_back(point);
	}

	return std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>(cloud_obstacle, cloud_road);
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	for (pcl::PointIndices clust_indices : cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>());

		for (auto i : clust_indices.indices)
			cloud_cluster->points.push_back(cloud->points[i]);

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;
}


Box BoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color)
{

	// Select color based off input value
	viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}