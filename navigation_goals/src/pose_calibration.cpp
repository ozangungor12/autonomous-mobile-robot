#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PoseCalibration
{
    public:
        PoseCalibration(){
            initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
        }
    
    private:
        ros::NodeHandle nh;
        ros::Publisher initial_pose_pub;
        
        void publishInitialPose(){
            geometry_msgs::PoseWithCovarianceStamped initialPose;
            initialPose.header.frame_id = "map";
            initialPose.pose.pose.position.x = -1.975;
            initialPose.pose.pose.position.y = -0.540;
            initialPose.pose.pose.position.z = 0.0;
            initialPose.pose.pose.orientation.x = initialPose.pose.pose.orientation.y = initialPose.pose.pose.orientation.z = 0.0;
            initialPose.pose.pose.orientation.w = 1.0;
            initialPose.pose.covariance = { 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.06853892326654787 };
        
            initial_pose_pub.publish(initialPose);
        
        }
};



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "PoseCalibration");
  //Create an object of class SubscribeAndPublish that will take care of everything
  PoseCalibration poseCalibration;
  ros::spin();
  return 0;
}
