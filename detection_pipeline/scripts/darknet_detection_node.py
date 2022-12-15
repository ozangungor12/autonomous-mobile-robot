#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# import the grpc files
import grpc
from api import darknet_detection_pb2_grpc
from api import darknet_detection_pb2

class DetectorDarknet():
    ''' 
    ObjectDetector class works as a bridge between ROS and gRPC.
    It subscribes by ROS to camera images and sends them to darknet 
    container via gRPC. The final image with detection Bboxes are
    published with detection topic.

    Subscribes: Image (raw_image)
    Publishes: Image with BBoxes (detection)
    '''
    
    def __init__(self):
        # Init ROS pubs and subs
        rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=52428800)
        self.img_publisher = rospy.Publisher("detection", Image, queue_size=1)
        self.cv_bridge = CvBridge()
        
        # Darknet stubs
        self.darknet_channel = grpc.insecure_channel("localhost:50054")
        self.darknet_stub =  darknet_detection_pb2_grpc.DarknetDetectionStub(self.darknet_channel) 
    
    def callback(self, msg):
        # Read incoming compressed image
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        print("Received image with shape: ", cv_img.shape)
        
        # Convert incoming image to grpc msg
        _, img_jpg = cv2.imencode('.jpg', cv_img)
        grpc_msg = darknet_detection_pb2.Image(data = img_jpg.tostring())

        # Connect to darknet server          
        detections = self.darknet_stub.darknetDetection(grpc_msg)
        print("Received detections from darknet")
        
        box_count = 0
        for box in detections.objects:
            cv2.rectangle(cv_img, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 3)
            cv2.putText(cv_img, box.label, (box.xmin + 5 , box.ymax - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
            if box.label == "eurobox":
                box_count+=1

        text = "Total Number of Boxes in Frame: " + str(box_count)
        cv2.rectangle(cv_img, (5,10), (480,40), (0,0,0), -1)
        cv2.putText(cv_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX , 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        
        # Publish the final image with BBoxes
        self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8"))

if __name__ == "__main__":
    try:
        rospy.init_node("Detector_Darknet")
        DetectorDarknet()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass