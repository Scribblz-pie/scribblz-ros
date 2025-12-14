#!/usr/bin/env python3
"""
Simple script to publish an image file to /uploaded_image topic
Usage: python3 publish_image.py <image_path>
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/uploaded_image', 10)
        
        # Load image
        img = cv2.imread(image_path)
        if img is None:
            self.get_logger().error(f'Failed to load image: {image_path}')
            return
        
        # Convert to ROS Image message
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = img.shape[1] * 3  # width * channels
        msg.data = img.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        
        # Publish
        self.publisher.publish(msg)
        self.get_logger().info(f'Published image {image_path} ({msg.width}x{msg.height}) to /uploaded_image')
        
        # Give it a moment to publish
        rclpy.spin_once(self, timeout_sec=0.1)

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 publish_image.py <image_path>')
        sys.exit(1)
    
    image_path = sys.argv[1]
    
    rclpy.init()
    node = ImagePublisher(image_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

