import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv2
import numpy as np
import tempfile
import math
from typing import List

# Import the path pipeline modules
from docking_station.path_pipeline.image_processing import (
    DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT,
    generate_image_stages,
)
from docking_station.path_pipeline.stroke_ordering import (
    strokes_from_polylines,
    order_strokes_with_containment,
)
from docking_station.path_pipeline.orientation_planner import (
    plan_all_orientations,
)
from docking_station.path_pipeline.holonomic_motion import (
    generate_holonomic_commands,
    optimize_waypoints_for_pure_translation,
)

# Constants (copied from path_sim_graph_eulerian.py)
TARGET_CANVAS_WIDTH = 1.0
CANVAS_PADDING = 0.05
ROBOT_SIDE_LENGTH = 0.1732
MARKER_OFFSET_X = 0.05
MARKER_OFFSET_Y = -0.0577
DOCK_POSITION = (1.05, -0.15)
DOCK_APPROACH = (1.05, 0.05)
DOCK_ORIENTATION = math.pi / 2
ERASE_MARGIN = 0.01
ROBOT_SPEED = 0.1
ROBOT_TURN_SPEED_DEG_PER_SEC = 90.0
ROBOT_WHEELBASE_L = 0.0866
ROBOT_PENUP_SPEED = 0.2


class ImageToPathNode(Node):
    def __init__(self):
        super().__init__('image_to_path')
        
        # Subscriber for uploaded images
        self.image_sub = self.create_subscription(
            Image,
            '/uploaded_image',
            self.image_callback,
            10
        )
        
        # Publisher for the generated path
        self.path_pub = self.create_publisher(Path, '/execute_drawing_path', 10)
        
        self.get_logger().info('Image to Path node initialized, listening on /uploaded_image')

    def image_callback(self, msg: Image):
        """Callback for received images."""
        try:
            self.get_logger().info('Received image, processing...')
            
            # Convert ROS Image to OpenCV format
            if msg.encoding == 'rgb8':
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            else:
                self.get_logger().error(f'Unsupported image encoding: {msg.encoding}')
                return
            
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as temp_file:
                temp_path = temp_file.name
                cv2.imwrite(temp_path, image)
            
            # Process the image
            path_msg = self.process_image_to_path(temp_path)
            
            # Publish the path
            self.path_pub.publish(path_msg)
            
            # Clean up temp file
            os.unlink(temp_path)
            
            self.get_logger().info(f'Path generated and published with {len(path_msg.poses)} waypoints')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image_to_path(self, image_path: str) -> Path:
        """Process image and return Path message."""
        # Build image stages
        stages = generate_image_stages(
            image_path=image_path,
            target_width=TARGET_CANVAS_WIDTH,
            padding=CANVAS_PADDING,
            smooth_factor=0.0,
            smoothing_points=200,
            simplification_epsilon_factor=0.0,
            dedup_tolerance=DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT,
            extractor="cv2",
            approx_tol=0.0,
            use_skeleton=True,
        )
        
        polylines = stages.rescaled_polylines
        
        # Plan stroke order
        strokes = strokes_from_polylines(polylines)
        if not strokes:
            raise ValueError("No strokes found in image")
        
        ordered_strokes = order_strokes_with_containment(strokes, start_position=DOCK_APPROACH)
        
        # Compute orientations (only draw phase)
        _, draw_wps, _ = plan_all_orientations(
            ordered_strokes,
            robot_side_length=ROBOT_SIDE_LENGTH,
            marker_offset_x=MARKER_OFFSET_X,
            marker_offset_y=MARKER_OFFSET_Y,
            samples_per_stroke=100,
            collision_buffer=ERASE_MARGIN,
            smooth_rate=0.3,
            dock_position=DOCK_POSITION,
            dock_approach=DOCK_APPROACH,
            dock_orientation=DOCK_ORIENTATION,
        )
        
        # Optimize for pure translation
        draw_wps = optimize_waypoints_for_pure_translation(draw_wps, rotation_threshold_rad=0.05)
        
        # Convert to Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Assuming map frame
        
        for wp in draw_wps:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            from tf_transformations import quaternion_from_euler
            q = quaternion_from_euler(0, 0, wp.theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
        
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()