#include "pcloudProcessor.h"

int main(int argc, char **argv)
{
    ROS_INFO("detection_3d node starts");
    ros::init(argc, argv, "detection3d_node");
    ros::NodeHandle nh;
    PcloudProcessor p(nh);
    ros::spin();

    return 0;
}
