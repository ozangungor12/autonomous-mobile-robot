#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "filter_cloud");
    ros::NodeHandle nh;

    // Create a ROS publisher for the output point cloud
    ros::Publisher pub_raw = nh.advertise<sensor_msgs::PointCloud2>("/raw_cloud", 1);
    ros::Publisher pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
    ros::Rate loop_rate(10);
    
    // Create random pointclouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud->width = 1000;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    for(std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 0;
    }
    // Create raw PointCloud publisher
    sensor_msgs::PointCloud2 output_raw;
    pcl::toROSMsg(*cloud, output_raw);
    output_raw.header.frame_id = "map";
    
    // Apply filtering and create a new PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PassThrough<pcl::PointXYZ> bandpass_filter;
    bandpass_filter.setInputCloud (cloud);
    bandpass_filter.setFilterFieldName ("x");
    bandpass_filter.setFilterLimits (0.0, 1.0);
    bandpass_filter.filter (*cloud_filtered);

    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*cloud_filtered, output_filtered);
    output_filtered.header.frame_id = "map";

    while (ros::ok())
    {
         pub_raw.publish(output_raw);
         pub_filtered.publish(output_filtered);
         ros::spinOnce();
         loop_rate.sleep();
    }

    return 0;
}