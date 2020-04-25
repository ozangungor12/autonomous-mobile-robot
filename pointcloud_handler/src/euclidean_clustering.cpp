#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// Segmentation specific includes
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud, *cloud_filtered);

            // Set segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (0.02);

            int i=0, nr_points = (int) cloud_filtered->points.size ();
            while (cloud_filtered->points.size () > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud (cloud_filtered);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0)
                {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud (cloud_filtered);
                extract.setIndices (inliers);
                extract.setNegative (false);

                // Get the points associated with the planar surface
                extract.filter (*cloud_plane);

                // Remove the planar inliers, extract the rest
                
                extract.setNegative (true);
                extract.filter (*cloud_rest);
                *cloud_filtered = *cloud_rest;
            }

            // publish the clustered PointCloud
            sensor_msgs::PointCloud2 ros_cloud;
            pcl::toROSMsg(*cloud_rest, ros_cloud);
            ros_cloud.header.frame_id = "base_scan";
            cloud_pub.publish(ros_cloud);
        }
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "clustering");
  //Create an object of class SubscribeAndPublish that will take care of everything
  Cluster cluster;
  ros::spin();
  return 0;
}