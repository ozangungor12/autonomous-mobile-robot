#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LaserToPointCloud
{
    public:
        LaserToPointCloud()
        {
        laser_sub = nh.subscribe("/scan", 1, &LaserToPointCloud::laserCallback, this);
        cloud_pub = nh.advertise<PointCloud>("/raw_cloud", 1);
        }

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
        {   
            // Convert LaserScan to ROS definiton of PointCloud2
            laser_geometry::LaserProjection projector;
            sensor_msgs::PointCloud2 ros_cloud;
            projector.projectLaser(*laser_scan, ros_cloud);

            // Convert ROS PointCloud2 to PointCloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
            // PointCloud pcl_cloud;
            pcl::fromROSMsg (ros_cloud, *pcl_cloud);
            cloud_pub.publish(pcl_cloud);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber laser_sub;
        ros::Publisher cloud_pub;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "laser_to_pointcloud");

  //Create an object of class SubscribeAndPublish that will take care of everything
  LaserToPointCloud laser_to_pointcloud;

  ros::spin();

  return 0;
}
