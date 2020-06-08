#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
# import the grpc files
import grpc
from api import darknet_detection_pb2_grpc
from api import darknet_detection_pb2

from api import object_detection_pb2_grpc
from api import object_detection_pb2

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
        rospy.Subscriber("image_raw", Image, self.callback, queue_size=1, buff_size=52428800)
        self.img_publisher = rospy.Publisher("detection", Image, queue_size=1)
        self.cv_bridge = CvBridge()
        
        # Darknet stubs
        # self.darknet_channel = grpc.insecure_channel("localhost:50053")
        # self.darknet_stub =  darknet_detection_pb2_grpc.DarknetDetectionStub(self.darknet_channel)
        
        # Tensorflow stubs
        self.tf_channel = grpc.insecure_channel("localhost:50055")
        self.tf_stub =  object_detection_pb2_grpc.ObjectDetectionStub(self.tf_channel)    
    
    def callback(self, msg):
        # Read incoming image
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        #(width, height, _) = cv_img.shape
        
        # Log image size
        # rospy.loginfo("width: {0}, height: {1}".format(width,height))
                  
        # darknet_detection = self.get_detections_darknet(cv_img)
        # tf_detection = self.get_detections_tensorflow(cv_img)
        rospy.loginfo("Received detection results")
        # print(tf_detection)
        
        # Print the detection
        #rospy.loginfo(detection)

        # Display the image
        # cv2.imshow('window', detected_cv_img)
        # cv2.waitKey(1)

        # Publish the final image with Bboxes
        self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8"))

    def get_detections_darknet(self, cv_img):
        # Convert opencv_image to grpc_msg
        _, img_jpg = cv2.imencode('.jpg', cv_img)
        grpc_msg = darknet_detection_pb2.Image(data = img_jpg.tostring())
        
        # Send a request the server for detection results
        detections = self.darknet_stub.darknetDetection(grpc_msg)
        for box in detections.objects:
            cv2.rectangle(cv_img, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 3)
            cv2.putText(cv_img, box.label, (box.xmin + 5 , box.ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)
        
        return cv_img

    def get_detections_tensorflow(self, cv_img):
         # Convert opencv_image to grpc_msg
        _, img_jpg = cv2.imencode('.jpg', cv_img)
        grpc_msg = darknet_detection_pb2.Image(data = img_jpg.tostring())

        # Send a request the server for detection results
        detections = self.tf_stub.objectDetection(grpc_msg)
        return detections

if __name__ == "__main__":
    try:
        rospy.init_node("object_detector")
        ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass