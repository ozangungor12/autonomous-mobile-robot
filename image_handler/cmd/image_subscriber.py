import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber():
    ''' initialize and configure the subscriber node '''
    def __init__(self):
        rospy.init_node("image_subscriber")
        rospy.Subscriber("image_transport", Image, self.callback)
        rospy.spin()
        
    ''' callback method to run when msg arrives from the publisher '''
    def callback(self, msg):
        self.cv_bridge = CvBridge()
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        (width, height, _) = cv_img.shape
        ''' log the messages to ros'''
        rospy.loginfo("width: {0}, height: {1}".format(width,height))
        ''' display the received img from the publisher'''
        cv2.imshow('window', cv_img)
        cv2.waitKey(1)    

if __name__ == "__main__":
    try:
        ImageSubscriber()
    except rospy.ROSInterruptException:
        pass

