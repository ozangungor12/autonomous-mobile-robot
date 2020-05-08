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
            // MarkerArray publisher
            markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/cluster_markers", 0 );
        }

        void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
        {
            // Read incoming cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            *filtered_cloud = *msg;
            
            // Call euclidean clustering
            pcl::PointCloud<pcl::PointXYZ> clustered_cloud;
            clustered_cloud = euclideanClustering(filtered_cloud, true);
            
            // Publish clustered_cloud
            cluster_pub.publish(clustered_cloud);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cluster_pub;
        ros::Publisher markers_pub;
        const int MIN_CLUSTER_SIZE = 3;
        const int MAX_CLUSTER_SIZE= 250;
        const float CLUSTER_TOLERANCE = 0.08;
        
        pcl::PointCloud<pcl::PointXYZ> euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, bool publish_markers)
        { 
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

            // Clustered cloud to be created by the method
            pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            clustered_cloud->header.frame_id = "/base_scan";
            
            // Create marker and markerArray
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            
            if (publish_markers)
            {
                marker.header.frame_id = "/base_scan";
                marker.header.stamp = ros::Time();
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
            }

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

                if (publish_markers)
                {
                    // Find min and max of cluster points to fit a box around them
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D(*single_cluster, minPt, maxPt);
    
                    // Modify the marker
                    marker.id = cluster_count;
                    marker.pose.position.x = (maxPt.x+minPt.x)/2;
                    marker.pose.position.y = (maxPt.y+minPt.y)/2;
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = (maxPt.x - minPt.x) + 0.2;
                    marker.scale.y = (maxPt.y - minPt.y) + 0.2;
                    marker.scale.z = 0.0;
                    marker.color.a = 1.0; // Don't forget to set the alpha!
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    
                    // Add marker to markerArray
                    markerArray.markers.push_back(marker);       
                }

                // Increase the cluster counter
                ++cluster_count;
            }

            if (publish_markers)
            {
                markers_pub.publish(markerArray);
            }
            
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