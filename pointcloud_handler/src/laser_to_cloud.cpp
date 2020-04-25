#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"
#include <pcl/filters/passthrough.h>
#include <string>

class LaserToCloud
{
    public:
        LaserToCloud()
        {
            // Create a subscriber that waits for chatter topic and executes callback function
            laser_sub = nh.subscribe("/scan", 1, &LaserToCloud::laserCallback, this);
            // Publish PointCloud2
            cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
            // Publish PointCloudXYZ
            // pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pcl_cloud",1);
        }

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            // Convert LaserScan to PointCloud2
            laser_geometry::LaserProjection projector;
            sensor_msgs::PointCloud2 ros_cloud;
            projector.projectLaser(*scan, ros_cloud);
            
            // Convert PointCloud2 to PointXYZ
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(ros_cloud, *pcl_cloud);

            // Apply filters to PointXYZ cloud
            filterPointCloud(pcl_cloud, "x", 2.0);
            filterPointCloud(pcl_cloud, "y", 2.0);

            // Convert PointXYZ to PointCloud2
            pcl::toROSMsg(*pcl_cloud, ros_cloud);
            ros_cloud.header.frame_id = "base_scan";

            // Publish PointCloud2
            cloud_pub.publish(ros_cloud);
        }
    
    private:
        ros::NodeHandle nh;
        ros::Publisher cloud_pub;
        ros::Publisher pcl_pub;
        ros::Subscriber laser_sub;

        void filterPointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const std::string axis, const float threshold)
        {
            pcl::PassThrough<pcl::PointXYZ> bandpass_filter;
            bandpass_filter.setInputCloud (pcl_cloud);
            bandpass_filter.setFilterFieldName (axis);
            bandpass_filter.setFilterLimits (0.0, threshold);
            bandpass_filter.filter (*pcl_cloud);
        }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "laser_to_cloud");

  //Create an object of class SubscribeAndPublish that will take care of everything
  LaserToCloud laser_to_cloud;

  ros::spin();

  return 0;
}