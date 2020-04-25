import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImagePublisher():
    ''' initialize and configure the node and the publisher '''
    def __init__(self):
        rospy.init_node("image_publisher")
        self.img_publisher = rospy.Publisher("image_transport", Image, queue_size=10)
        self.rate = rospy.Rate(10)
        self.cv_bridge = CvBridge()
        self.cam = cv2.VideoCapture(0)
        self.publish()
    
    ''' main method to publish the images'''
    def publish(self):
        while not rospy.is_shutdown():
            _ , cv_img = self.cam.read()
            self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8"))
            self.rate.sleep()
    
if __name__ == "__main__":
    try:
        ImagePublisher()
    except rospy.ROSInterruptException:
        pass

    