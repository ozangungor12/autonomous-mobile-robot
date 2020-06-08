#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import grpc
# import the grpc files
from api import object_detection_pb2_grpc
from api import object_detection_pb2

class TestClient():
    ''' 
        Test client for tensorflow2 grpc connection
    '''
    
    def __init__(self):
        # Init ROS pubs and subs
        rospy.Subscriber("image_raw", Image, self.callback, queue_size=1, buff_size=52428800)
        self.img_publisher = rospy.Publisher("test_detection_tf", Image, queue_size=1)
        self.cv_bridge = CvBridge()
        
        # Create stubs 
        self.tf_channel = grpc.insecure_channel("localhost:50055")
        self.tf_stub =  object_detection_pb2_grpc.ObjectDetectionStub(self.tf_channel)  
        
    def callback(self, msg):
        # Read incoming image
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        _, img_jpg = cv2.imencode('.jpg', cv_img)
        grpc_msg = object_detection_pb2.Image(data = img_jpg.tostring())
        
        # Connect to test server
        test_detections = self.tf_stub.objectDetection(grpc_msg)

        # Display the test BBox
        for box in test_detections.objects:
            cv2.rectangle(cv_img, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 3)
            cv2.putText(cv_img, box.label, (box.xmin + 5 , box.ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

        # Publish the final image with Bboxes
        self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8"))

if __name__ == "__main__":
    try:
        rospy.init_node("test_client_tf")
        TestClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass