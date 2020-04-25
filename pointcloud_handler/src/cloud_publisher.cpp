#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cloud_publisher");
    ros::NodeHandle nh;

    // Create a ROS publisher for the output point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    ros::Rate loop_rate(10);
    
    // Create random pointclouds
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    cloud.width = 1000;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    for(std::size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    pcl::toROSMsg(cloud, output);

    while (ros::ok())
    {
         pub.publish(output);
         ros::spinOnce();
         loop_rate.sleep();
    }

    return 0;
}


