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


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # --- UPDATED DEFAULTS WITH YOUR SPECIFIC COORDINATES ---
        # DOCKED: x = -.1174, y = .5243, orientation = -90 (-pi/2)
        self.declare_parameter('dock_position_x', -0.1174)
        self.declare_parameter('dock_position_y', 0.5243)
        
        # APPROACH: x = .17268, y = .3546, orientation = -90 (-pi/2)
        self.declare_parameter('dock_approach_x', 0.17268)
        self.declare_parameter('dock_approach_y', 0.3546)
        
        self.declare_parameter('dock_orientation', -math.pi / 2.0)
        
        # Offsets to place the drawing relative to the dock
        self.declare_parameter('drawing_x_shift', 0.0)
        self.declare_parameter('drawing_y_shift', 0.0)
        
        self.declare_parameter('reflect_y_about_center', True)
        self.declare_parameter('target_canvas_width', 0.7)
        
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
        self.DRAWING_X_SHIFT = self.get_parameter('drawing_x_shift').value
        self.DRAWING_Y_SHIFT = self.get_parameter('drawing_y_shift').value
        self.REFLECT_Y_ABOUT_CENTER = self.get_parameter('reflect_y_about_center').value
        self.TARGET_CANVAS_WIDTH = self.get_parameter('target_canvas_width').value
        
        # Other constants
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
        
        self.get_logger().debug('Image processor node initialized, listening on /uploaded_image')

    def image_callback(self, msg: Image):
        """Callback for received images."""
        try:
            self.get_logger().debug('Received image, processing...')
            
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
            
            self.get_logger().debug(f'Path generated and published with {len(path_msg.poses)} waypoints')
            
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
        
        # 1. FLIP Y-COORDINATES OF POLYLINES (Geometry Flip)
        if self.REFLECT_Y_ABOUT_CENTER and polylines:
            all_y = [y for polyline in polylines for x, y in polyline]
            min_y = min(all_y)
            max_y = max(all_y)
            center_y = 0.5 * (min_y + max_y)
            
            flipped_polylines = []
            for polyline in polylines:
                flipped = [(x, 2 * center_y - y) for x, y in polyline]
                flipped_polylines.append(flipped)
            polylines = flipped_polylines

        # 2. TRANSFORM DRAWING TO ABSOLUTE MAP COORDINATES
        # We shift the drawing so it is placed relative to your specific Dock Position.
        # This keeps the Dock fixed at (-0.1174, 0.5243) and moves the drawing around it.
        offset_x = self.DOCK_POSITION[0] + self.DRAWING_X_SHIFT
        offset_y = self.DOCK_POSITION[1] + self.DRAWING_Y_SHIFT
        
        transformed_polylines = []
        for polyline in polylines:
            transformed = [(x + offset_x, y + offset_y) for x, y in polyline]
            transformed_polylines.append(transformed)
        polylines = transformed_polylines
        
        # 3. USE YOUR ABSOLUTE DOCK COORDINATES DIRECTLY
        # Since we moved the drawing into the Map Frame, we can use the real dock/approach values.
        dock_orientation = self.DOCK_ORIENTATION  # Use the physical orientation directly (-90 deg)
        
        # Plan stroke order
        strokes = strokes_from_polylines(polylines)
        if not strokes:
            raise ValueError("No strokes found in image")
        
        ordered_strokes = order_strokes_with_containment(strokes, start_position=self.DOCK_APPROACH)
        
        # Compute orientations (all phases) in Map Coordinates
        undock_wps, draw_wps, dock_wps = plan_all_orientations(
            ordered_strokes,
            robot_side_length=self.ROBOT_SIDE_LENGTH,
            marker_offset_x=self.MARKER_OFFSET_X,
            marker_offset_y=self.MARKER_OFFSET_Y,
            samples_per_stroke=100,
            collision_buffer=self.ERASE_MARGIN,
            smooth_rate=0.3,
            dock_position=self.DOCK_POSITION,  # e.g., (-0.1174, 0.5243)
            dock_approach=self.DOCK_APPROACH,  # e.g., (0.17268, 0.3546)
            dock_orientation=dock_orientation,
        )
        
        # Combine all waypoints (They are already in the correct Map Frame)
        all_wps = undock_wps + draw_wps + dock_wps
        
        # Optimize for pure translation
        all_wps = optimize_waypoints_for_pure_translation(all_wps, rotation_threshold_rad=0.05)
        
        # Convert to Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map' 
        
        for wp in all_wps:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            from scipy.spatial.transform import Rotation
            q = Rotation.from_euler('xyz', [0, 0, wp.theta]).as_quat()
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