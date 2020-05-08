#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// Segmentation specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
// markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class CloudCluster
{
    public:
        CloudCluster()
        {
            // Create a subscriber that waits for chatter topic and executes callback function
            cloud_sub = nh.subscribe("/filtered_cloud", 1, &CloudCluster::cloudCallback, this);
            // PointCloud publisher
            cluster_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/clustered_cloud", 1);
        }

        void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
        {
            // Read incoming cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            *filtered_cloud = *msg;

            // Call euclidean clustering

            // Call pcd_write
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cluster_pub;
        

};