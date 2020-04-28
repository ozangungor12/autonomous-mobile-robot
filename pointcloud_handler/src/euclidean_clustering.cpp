#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// Segmentation specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
// iostream
#include <iostream>
// markers
#include <visualization_msgs/Marker.h>

class Cluster
{
    public:
        Cluster()
        {
            // Subscribe to filtered PointCloud2
            cloud_sub = nh.subscribe("/filtered_cloud", 1, &Cluster::cloudCallback, this);
            // Publish clustered PointCloud2
            cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud", 1);
            // Publish cluster boxes 
            boxes_pub = nh.advertise<visualization_msgs::Marker>("/cluster_boxes", 0 );
        }

        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
        {
            // Get the input cloud and convert it to PointXYZ 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*input, *cloud_filtered);
            
            // Create ROS and PCL cluster variables
            sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);  
            pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            
            // Create PointCloud writer object
            // pcl::PCDWriter writer;
            
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);

            // Set Euclidean Clustering parameters
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.08); // 8 cm
            ec.setMinClusterSize (3);      // min 3 points
            ec.setMaxClusterSize (250);    // min 250 points
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);
            std::vector<pcl::PointIndices>::const_iterator it;
            std::vector<int>::const_iterator pit;

            int cluster_count = 1;
            for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
                {
                    // push_back adds a point to the end of the existing vector
                    cloud_cluster->points.push_back(cloud_filtered->points[*pit]); 
                }

                // Set the parameters of individual cluster
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                // Find min an max of cluster points
                pcl::PointXYZ minPt, maxPt;
                pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

                std::cout << "Size: " << cloud_cluster->width << " " << "Min: " << minPt << " "  << "Max: " << maxPt << std::endl;

                // Publish the markers

                visualization_msgs::Marker marker;
                marker.header.frame_id = "/base_footprint";
                marker.header.stamp = ros::Time();
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.id = cluster_count;
                marker.pose.position.x = 1;
                marker.pose.position.y = 1;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 0.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                //Merge current clusters to a single PointCloud to publish
                *clustered_cloud += *cloud_cluster;
                
                // Write clusters to seperate pcd files
                // std::stringstream ss;
                // ss << "cloud_cluster_" << cluster_count << ".pcd";
                // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

                // Increase the cluster counter
                ++cluster_count;

                boxes_pub.publish(marker);
            }      

            std::cout << "Number of clusters: " << cluster_count << std::endl;
            
            // Publish the clusters
            pcl::toROSMsg (*clustered_cloud , *clusters);
            clusters->header.frame_id = "/base_scan";
            cloud_pub.publish (*clusters);

            // End of laserCallback
        }
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub, boxes_pub;  
};

int main(int argc, char **argv)
{
  // Initialize clustering Node
  ros::init(argc, argv, "clustering");
  // Create an object of Cluster class
  Cluster cluster;
  ros::spin();
  return 0;
}