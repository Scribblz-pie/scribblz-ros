import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import time

class SingleImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('single_image_uploader')
        
        # Create a "Latched" profile (Reliable + Transient Local)
        # This guarantees delivery even if the subscriber joins LATE.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # <--- THE FIX
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Image, 'uploaded_image', qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.image_path = image_path
        
        # 2. Use a timer to publish ONCE after a short delay
        # (Giving the node 1 second to set up connections before sending)
        self.timer = self.create_timer(1.0, self.publish_once_callback)
        self.has_published = False

    def publish_once_callback(self):
        if self.has_published:
            return  # Do nothing if already published

        # Load and process image
        img = cv2.imread(self.image_path)
        if img is None:
            self.get_logger().error(f"Could not read image: {self.image_path}")
            sys.exit(1)

        # Convert and Publish
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.frame_id = "map"
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Successfully published: {self.image_path}")
        
        # Mark as done so we don't publish again
        self.has_published = True
        
        # OPTIONAL: You can destroy the timer now to save resources
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    
    # Get filename from command line
    if len(sys.argv) < 2:
        print("Usage: python3 image_publisher.py <image_file>")
        return

    node = SingleImagePublisher(sys.argv[1])

    print("Node running. Press Ctrl+C to stop.")
    
    # 3. SPIN: Keep the script alive so the image remains available
    # The script will hang here, which is what we want.
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()