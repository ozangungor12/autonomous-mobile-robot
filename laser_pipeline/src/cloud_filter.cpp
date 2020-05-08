#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/passthrough.h>

class CloudFilter
{
    public:
        CloudFilter()
        {
            // Create a subscriber that waits for chatter topic and executes callback function
            cloud_sub = nh.subscribe("/raw_cloud", 1, &CloudFilter::cloudCallback, this);
            // PointCloud publisher
            cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/filtered_cloud", 1);
        }

        void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
        {   
            // Read incoming cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            *filtered_cloud = *msg;
            
            // Apply filter
            filterPointCloud(filtered_cloud, "x", 3.0);

            // Publish filtered cloud
            cloud_pub.publish(filtered_cloud);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        
        void filterPointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const std::string axis, const float threshold)
        {
            pcl::PassThrough<pcl::PointXYZ> bandpass_filter;
            bandpass_filter.setInputCloud (input_cloud);
            bandpass_filter.setFilterFieldName (axis);
            bandpass_filter.setFilterLimits (0.0, threshold);
            bandpass_filter.filter (*input_cloud);
        }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "cloud_filter");
  //Create an object of class SubscribeAndPublish that will take care of everything
  CloudFilter cloud_filter;
  ros::spin();
  return 0;
}
