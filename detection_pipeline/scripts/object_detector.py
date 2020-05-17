#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ObjectDetector():
    ''' 
    ObjectDetector class works as a bridge between
    ROS and gRPC frameworks. It subscribes by ROS to camera images
    and sends them to darknet container via gRPC. The detection results
    are published as Bounding Boxes.

    Subscribes: Image (raw_image)
    Publishes: BBox (detections)
    '''
    
    def __init__(self):
        rospy.init_node("object_detector")
        rospy.Subscriber("raw_image", Image, self.callback)
        self.cv_bridge = CvBridge()
        rospy.spin()
        
    # Callback method to subscribe to images via topic raw_image
    def callback(self, msg):
        # Read incoming image
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        (width, height, _) = cv_img.shape
        
        # log the messages
        rospy.loginfo("width: {0}, height: {1}".format(width,height))
        
        # Display the received img from the publisher'''
        cv2.imshow('window', cv_img)
        cv2.waitKey(1)    

if __name__ == "__main__":
    try:
        ObjectDetector()
    except rospy.ROSInterruptException:
        pass