#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"
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
            cloud_sub = nh.subscribe("/raw_cloud", 1, &CloudCluster::cloudCallback, this);
            // PointCloud publisher
            cluster_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/clustered_cloud", 1);
        }

        void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
        {
            // Read incoming cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            *filtered_cloud = *msg;
            
            // Call euclidean clustering
            pcl::PointCloud<pcl::PointXYZ> clustered_cloud;
            clustered_cloud = euclideanClustering(filtered_cloud);
            
            // Publish clustered_cloud
            cluster_pub.publish(clustered_cloud);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cluster_pub;
        const int MIN_CLUSTER_SIZE = 3;
        const int MAX_CLUSTER_SIZE= 250;
        const float CLUSTER_TOLERANCE = 0.08;
        
        pcl::PointCloud<pcl::PointXYZ> euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
        {   
            // Clustered cloud to be created by the method
            pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (input_cloud);

            // Set Euclidean Clustering parameters
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (CLUSTER_TOLERANCE); // 8 cm
            ec.setMinClusterSize (MIN_CLUSTER_SIZE);      // min 3 points
            ec.setMaxClusterSize (MAX_CLUSTER_SIZE);    // min 250 points
            ec.setSearchMethod (tree);
            ec.setInputCloud (input_cloud);
            ec.extract (cluster_indices);
            std::vector<pcl::PointIndices>::const_iterator it;
            std::vector<int>::const_iterator pit;

            int cluster_count = 0;
            for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
            {   
                pcl::PointCloud<pcl::PointXYZ>::Ptr single_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
                {
                    // Push_back adds a point to the end of the existing vector
                    single_cluster->points.push_back(input_cloud->points[*pit]); 
                }

                single_cluster->width = single_cluster->points.size ();
                single_cluster->height = 1;
                single_cluster->is_dense = true;

                // Merge current clusters to a single PointCloud to publish
                *clustered_cloud += *single_cluster;

                // Increase the cluster counter
                ++cluster_count;
            }
            
            clustered_cloud->header.frame_id = "/base_scan";
            return *clustered_cloud;
            // cluster_pub.publish(clustered_cloud);
        
        }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "cloud_cluster");
  //Create an object of class SubscribeAndPublish that will take care of everything
  CloudCluster cloud_cluster;
  ros::spin();
  return 0;
}