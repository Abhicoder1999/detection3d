
#include<iostream>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_set>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointXYZI;
struct RansacParam
{
	float lidar_height;
	float ground_clearance;
	float robot_length;
	float robot_width;
    int maxIterations;
    float distanceThreshold;
	Eigen::Vector4f minRandSearchRange;
	Eigen::Vector4f maxRandSearchRange;

};

struct ClustParam
{
    int minClustersize;
    int maxClustersize;
    float clusterTolerance;
};

class Pcore
{

public:
    Pcore();
    ~Pcore();

    void FilterCloud(PointXYZI::Ptr incloud, PointXYZI::Ptr outcloud, float leafsize);
    void CropCloud(PointXYZI::Ptr incloud, PointXYZI::Ptr outcloud, const Eigen::Vector4f minRange, const Eigen::Vector4f maxRange);
    std::pair<PointXYZI::Ptr, PointXYZI::Ptr> RansacPlane(PointXYZI::Ptr incloud, RansacParam& ransParam);
    void Clustering(PointXYZI::Ptr incloud, std::vector<PointXYZI::Ptr>& clusters, ClustParam& clusParam);

};
