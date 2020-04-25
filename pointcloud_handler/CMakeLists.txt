cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_handler)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  laser_geometry
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
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_handler
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_publisher src/pcl_publisher.cpp)
target_link_libraries(pcl_publisher ${catkin_LIBRARIES})

add_executable(cloud_publisher src/cloud_publisher.cpp)
target_link_libraries(cloud_publisher ${catkin_LIBRARIES})

add_executable(filter_cloud src/filter_cloud.cpp)
target_link_libraries(filter_cloud ${catkin_LIBRARIES})

add_executable(laser_to_cloud src/laser_to_cloud.cpp)
target_link_libraries(laser_to_cloud ${catkin_LIBRARIES})

add_executable(euclidean_clustering src/euclidean_clustering.cpp)
target_link_libraries(euclidean_clustering ${catkin_LIBRARIES})