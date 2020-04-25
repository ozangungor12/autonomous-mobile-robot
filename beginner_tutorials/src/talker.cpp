#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main (int argc, char **argv)
{
    // Initiliaze the node by the given name
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    // Initiliaze the publisher and topic properties
    ros::Publisher publisher = n.advertise<std_msgs::String>("chatter", 1000);
    // Run the node in given Hz
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "hello world " << count;
        msg.data = ss.str();

        publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}