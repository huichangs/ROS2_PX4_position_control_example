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
        self.Image_subscription = self.create_subscription(
        Image, 
        '/depth_camera/theora', 
        self.get_image_callback, 
        qos)
        self.Image_subscription # prevent unused variable warning
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
    # def get_image_callback(self, msg):
    #     # Convert ROS Image message to OpenCV image
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     # cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
    #     # cv_image_visual = cv2.convertScaleAbs(cv_image, alpha=0.03)  # Scale for visualization
        
    #     # Show Results
    #     cv2.imshow("streaming", cv_image)
    #     cv2.waitKey(1)
    
    def get_image_callback(self, msg):
        decoded_frame = self.decode_theora(msg.data)
        if decoded_frame is not None:
            cv2.imshow("Depth Camera", decoded_frame)
            cv2.waitKey(1)
            
    
    def decode_theora(self, data):
        # ffmpeg 또는 다른 디코딩 라이브러리를 사용하여 데이터 디코딩
        try:
            # ffmpeg 명령을 통해 데이터 디코딩 예시
            process = (
                ffmpeg
                .input('pipe:0', format='theora')
                .output('pipe:1', format='rawvideo', pix_fmt='bgr24')
                .run(input=data, capture_stdout=True)
            )
            frame = np.frombuffer(process, np.uint8).reshape([height, width, 3])  # 프레임 크기 설정 필요
            return frame
        except Exception as e:
            self.get_logger().error(f"Error decoding Theora: {e}")
            return None
        
def main(args=None):
    rclpy.init(args=args)
    uav_camera = UavCamera()
    
    rclpy.spin(uav_camera)
    
    uav_camera.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()