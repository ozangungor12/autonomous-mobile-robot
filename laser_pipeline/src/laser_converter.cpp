#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "laser_geometry/laser_geometry.h"
#include "pcl_ros/point_cloud.h"

class LaserConverter
{
    public:
        LaserConverter()
        {
            // Create a subscriber that waits for chatter topic and executes callback function
            laser_sub = nh.subscribe("/scan", 1, &LaserConverter::laserCallback, this);
            // PointCloud publisher
            cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/raw_cloud", 1);
        }

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {   
            // Convert laser scan to PointCloud
            laser_geometry::LaserProjection projector;
            sensor_msgs::PointCloud2 cloud;
            projector.projectLaser(*scan, cloud);
            
            // Publish raw PointCloud
            cloud_pub.publish(cloud);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber laser_sub;
        ros::Publisher cloud_pub;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "laser_converter");
  //Create an object of class SubscribeAndPublish that will take care of everything
  LaserConverter laser_converter;
  ros::spin();
  return 0;
}
