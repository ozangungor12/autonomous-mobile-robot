#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import sys, time

class CompressedSubscriber():
    ''' initialize and configure the subscriber node '''
    def __init__(self):
        self.image_pub = rospy.Publisher("/detection/compressed", CompressedImage, queue_size=1)
        rospy.Subscriber("/raw_image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=52428800)
        
    ''' callback method to run when msg arrives from the publisher '''
    def callback(self, msg):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv_img)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
 
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('compressed_subscriber')
    try:
        CompressedSubscriber()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
