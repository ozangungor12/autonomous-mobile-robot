#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/point_cloud2_iterator.h>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_publisher");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/raw_cloud", 1);
  ros::Rate loop_rate(10);

  // Create random pointclouds
  
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = ros::Time::now();
  cloud.width  = 100;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(cloud.width);

  sensor_msgs::PointCloud2Iterator <float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator <float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator <float> iter_z(cloud, "z");

  for (int i=0; i<cloud.width; i++)
  {
    *iter_x = 1024 * rand () / (RAND_MAX + 1.0f);
    *iter_y = 1024 * rand () / (RAND_MAX + 1.0f);
    *iter_z = 0;
    ++iter_x; ++iter_y; ++iter_z;
  }

  // Publish the PointCloud
  while (ros::ok())
  {
    pub.publish(cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  return 0;
}