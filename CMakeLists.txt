cmake_minimum_required(VERSION 3.0.2)
project(detection3d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++14)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED
roscpp
rospy
std_msgs
pcl_ros
pcl_conversions
sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES detection3d
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)
# link_directories(${PCL_LIBRARY_DIRS})

add_library(detection_lib
  src/pcore.cpp
  src/pcloudProcessor.cpp
)


add_executable(${PROJECT_NAME}_node src/detection3d_node.cpp src/pcloudProcessor.cpp src/pcore.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
   pcl_visualization
   detection_lib
 )
