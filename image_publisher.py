#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv_bridge
import cv2
import sys

class ImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('image_publisher')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,  # <--- CHANGED from BEST_EFFORT
        history=HistoryPolicy.KEEP_LAST,
        depth=1
        )
        self.publisher_ = self.create_publisher(Image, 'uploaded_image', qos_profile=qos_profile)
        self.bridge = cv_bridge.CvBridge()
        
        # Load and publish the image
        img = cv2.imread(image_path)
        if img is None:
            self.get_logger().error(f'Could not load image from {image_path}')
            return
        
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(ros_image)
        self.get_logger().info(f'Published image from {image_path} to /uploaded_image')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python image_publisher.py <image_path>")
        return
    
    image_path = sys.argv[1]
    rclpy.init(args=args)
    node = ImagePublisher(image_path)
    rclpy.shutdown()

if __name__ == '__main__':
    main()