#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
# import the grpc files
import grpc
from api import darknet_detection_pb2_grpc
from api import darknet_detection_pb2

class ObjectDetector():
    ''' 
    ObjectDetector class works as a bridge between ROS and gRPC.
    It subscribes by ROS to camera images and sends them to darknet 
    container via gRPC. The final image with detection Bboxes are
    published with detection topic.

    Subscribes: Image (raw_image)
    Publishes: Image with BBoxes (detection)
    '''
    
    def __init__(self):
        
        rospy.Subscriber("raw_image/compressed", CompressedImage, self.callback, queue_size = 1, buff_size=52428800)
        self.img_publisher_comp = rospy.Publisher("detection/compressed", CompressedImage, queue_size = 1)
        self.img_publisher = rospy.Publisher("detection", Image, queue_size = 1)
        self.cv_bridge = CvBridge()
        self.darknet_channel = grpc.insecure_channel("localhost:50053")
        self.stub =  darknet_detection_pb2_grpc.DarknetDetectionStub(self.darknet_channel)

    def callback(self, msg):
        # Read incoming image
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        # Perform detection
        detection = self.get_detections(cv_img)

        # Print the detection
        # rospy.loginfo(detections)

        # Display the received img from the publisher'''
        # cv2.imshow('window', detected_cv_img)
        # cv2.waitKey(1)

        # Publish the final image with Bboxes
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', detection)[1]).tostring()
        
        # Publish new image
        self.img_publisher_comp.publish(msg)
        # self.img_publisher_comp.publish(detection)

        # self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(detection, "bgr8"))

    def get_detections(self, cv_img):
        # Convert opencv_image to grpc_msg
        _, img_jpg = cv2.imencode('.jpg', cv_img)
        grpc_msg = darknet_detection_pb2.Image(data = img_jpg.tostring())
        
        # Send a request the server for detection results
        detections = self.stub.darknetDetection(grpc_msg)
        for box in detections.objects:
            cv2.rectangle(cv_img, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 3)
            cv2.putText(cv_img, box.label, (box.xmin + 5 , box.ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)
        
        return cv_img

if __name__ == "__main__":
    try:
        rospy.init_node("object_detector_compressed")
        ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass