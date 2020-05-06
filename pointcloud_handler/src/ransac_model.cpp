#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// Ransac 
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

class RansacModel
{
    public:
        RansacModel()
        {
            // Subscribe to filtered PointCloud2
            cloud_sub = nh.subscribe("/filtered_cloud", 1, &RansacModel::cloudCallback, this);
            // Publish clustered PointCloud2
            ransac_pub = nh.advertise<sensor_msgs::PointCloud2>("/ransac_cloud", 1);
        }

        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*input, *cloud_filtered);

            pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

            
            
            
            std::vector<int> inliers;

            // created RandomSampleConsensus object and compute the appropriated model
            // pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
            //     model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud_filtered));
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
                model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_filtered));
            
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
            ransac.setDistanceThreshold (.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
            
            pcl::copyPointCloud (*cloud_filtered, inliers, *final);
            
            
            
            sensor_msgs::PointCloud2::Ptr ransac_cloud (new sensor_msgs::PointCloud2); 
            pcl::toROSMsg (*final , *ransac_cloud);
            ransac_cloud->header.frame_id = "/base_scan";
            ransac_pub.publish(*ransac_cloud);
        }
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher ransac_pub;
};

int main(int argc, char **argv)
{
  // Initialize clustering Node
  ros::init(argc, argv, "ransac");
  // Create an object of Cluster class
  RansacModel ransac;
  ros::spin();
  return 0;
}