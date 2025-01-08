import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import ffmpeg


class UavCamera(Node):
    def __init__(self):
        super().__init__('uav_camera')
        qos = QoSProfile(depth=10)
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.raw_image_subscription_1 = self.create_subscription(
        Image, 
        '/camera', 
        self.get_raw_image_callback_1,
        qos)
        
        self.raw_image_subscription_2 = self.create_subscription(
        Image, 
        '/camera_1', 
        self.get_raw_image_callback_2,
        qos)
        
        self.raw_image_subscription_3 = self.create_subscription(
        Image, 
        '/camera_2', 
        self.get_raw_image_callback_3,
        qos)
        
        self.raw_image_subscription_1
        self.raw_image_subscription_2
        self.raw_image_subscription_3
        
        
    def get_raw_image_callback_1(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image_1 = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        
        # Show Results
        cv2.imshow("streaming_1", cv_image_1)
        cv2.waitKey(1)
        
    def get_raw_image_callback_2(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image_2 = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        
        # Show Results
        # cv2.imshow("streaming_2", cv_image_2)
        cv2.waitKey(1)
        
    def get_raw_image_callback_3(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image_3 = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        
        # Show Results
        # cv2.imshow("streaming_3", cv_image_3)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    uav_camera = UavCamera()
    
    rclpy.spin(uav_camera)
    
    uav_camera.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()