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

class Cluster
{
    public:
        Cluster()
        {
            // Subscribe filtered PointCloud2
            cloud_sub = nh.subscribe("/filtered_cloud", 1, &Cluster::cloudCallback, this);
            // Publish clustered PointCloud2
            cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud", 1);
        }

        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            // TODO


        }
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "laser_to_cloud");
  //Create an object of class SubscribeAndPublish that will take care of everything
  Cluster cluster;
  ros::spin();
  return 0;
}