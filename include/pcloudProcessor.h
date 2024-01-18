
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
// #include "render/box.h"
// #include "render/render.h"
// #include "kdtree.h"
// #include "cluster.h"
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

using namespace std;


std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> myRansacPlane( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // My implementation of RANSAC for segmenting planes. 

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacle (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_road (new pcl::PointCloud<pcl::PointXYZI> ());
    
    //reducing ROI for improvement in road plane finding
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

	while(maxIterations-- >= 0) {
		std::unordered_set<int> inliers_set;

		while(inliers_set.size() < 3) 
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

		a = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		b = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1));
		c = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1));
		d = -(a*x1 + b*y1 + c*z1);

        inliers_set.clear();

		for(int i=0; i < cloud->points.size(); i++) {
			// if(inliers_set.count(i))
			// 	continue;

			float x0{}, y0{}, z0{}, dist{};
			x0 = cloud->points[i].x;
			y0 = cloud->points[i].y;
			z0 = cloud->points[i].z;

			dist = fabs((a*x0)+(b*y0)+(c*z0)+d) / sqrt((a*a) + (b*b) + (c*c));

			if(dist <= distanceThreshold)
				inliers_set.insert(i);
		}

		if(inliers_set.size() > inliersResult.size())
			inliersResult = inliers_set;

	}

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			cloud_road->points.push_back(point);
		else
			cloud_obstacle->points.push_back(point);
	}

    return std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr>(cloud_obstacle, cloud_road);
}