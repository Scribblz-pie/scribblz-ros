import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv2
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import tempfile
import os
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
# Moved to instance variables with ROS parameters


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # Declare parameters
        self.declare_parameter('dock_position_x', 1.05)
        self.declare_parameter('dock_position_y', 1.05)
        self.declare_parameter('dock_approach_x', 1.05)
        self.declare_parameter('dock_approach_y', 0.95)
        self.declare_parameter('dock_orientation', math.pi)
        self.declare_parameter('target_canvas_width', 1.0)
        
        # Load parameters into constants
        self.DOCK_POSITION = (
            self.get_parameter('dock_position_x').value,
            self.get_parameter('dock_position_y').value
        )
        self.DOCK_APPROACH = (
            self.get_parameter('dock_approach_x').value,
            self.get_parameter('dock_approach_y').value
        )
        self.DOCK_ORIENTATION = self.get_parameter('dock_orientation').value
        self.TARGET_CANVAS_WIDTH = self.get_parameter('target_canvas_width').value
        
        # Other constants (not parameterized yet)
        self.CANVAS_PADDING = 0.05
        self.ROBOT_SIDE_LENGTH = 0.1732
        self.MARKER_OFFSET_X = 0.05
        self.MARKER_OFFSET_Y = -0.0577
        self.ERASE_MARGIN = 0.01
        self.ROBOT_SPEED = 0.1
        self.ROBOT_TURN_SPEED_DEG_PER_SEC = 90.0
        self.ROBOT_WHEELBASE_L = 0.0866
        self.ROBOT_PENUP_SPEED = 0.2
        
        # Subscriber for uploaded images
        self.image_sub = self.create_subscription(
            Image,
            '/uploaded_image',
            self.image_callback,
            10
        )
        
        # Publisher for the generated path
        self.path_pub = self.create_publisher(Path, '/execute_drawing_path', 10)
        
        self.get_logger().info('Image processor node initialized, listening on /uploaded_image')

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
            target_width=self.TARGET_CANVAS_WIDTH,
            padding=self.CANVAS_PADDING,
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
        
        ordered_strokes = order_strokes_with_containment(strokes, start_position=self.DOCK_APPROACH)
        
        # Compute orientations (all phases)
        undock_wps, draw_wps, dock_wps = plan_all_orientations(
            ordered_strokes,
            robot_side_length=self.ROBOT_SIDE_LENGTH,
            marker_offset_x=self.MARKER_OFFSET_X,
            marker_offset_y=self.MARKER_OFFSET_Y,
            samples_per_stroke=100,
            collision_buffer=self.ERASE_MARGIN,
            smooth_rate=0.3,
            dock_position=self.DOCK_POSITION,
            dock_approach=self.DOCK_APPROACH,
            dock_orientation=self.DOCK_ORIENTATION,
        )
        
        # Combine all waypoints
        all_wps = undock_wps + draw_wps + dock_wps
        
        # Offset the path so DOCK_POSITION becomes (0, 0)
        offset_x = -self.DOCK_POSITION[0]
        offset_y = -self.DOCK_POSITION[1]
        for wp in all_wps:
            wp.x += offset_x
            wp.y += offset_y
        
        # Optimize for pure translation
        all_wps = optimize_waypoints_for_pure_translation(all_wps, rotation_threshold_rad=0.05)
        
        # Convert to Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Assuming map frame
        
        for wp in all_wps:
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
    node = ImageProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()