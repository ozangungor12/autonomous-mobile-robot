#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class CaptureCam
{
    public:
        CaptureCam(){
            image_transport::ImageTransport it(nh);
            image_pub = it.advertise("raw_image",1);
            stream();
        }

    private:
        ros::NodeHandle nh;
        image_transport::Publisher image_pub;
        
        // Method to capture frames from camera
        void stream(){
            
            // Set video source
            cv::VideoCapture cap(0);
            // Check if video device can be opened with the given index
            cv::Mat frame;
            sensor_msgs::ImagePtr msg;

            while (nh.ok()) {
                cap >> frame;
                // Check if grabbed frame is actually full with some content
                if(!frame.empty()) {
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                image_pub.publish(msg);
                cv::waitKey(1);
                }
            }
        }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "capture_cam");
  //Create an object of class SubscribeAndPublish that will take care of everything
  CaptureCam capture_cam;
  ros::spin();
  return 0;
}
