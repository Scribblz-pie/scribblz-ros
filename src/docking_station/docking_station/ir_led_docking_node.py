import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu, CompressedImage
import math


# ======================================================
# =================== TOWER TRACKER ====================
# ======================================================

class TowerTracker:
    """
    Tracks two LED towers (left and right) using temporal tracking,
    IMU orientation, and geometric constraints.
    """
    
    def __init__(self, tower_offset_m=0.03, max_jump_pixels=100):
        """
        Args:
            tower_offset_m: Distance from robot center to each tower (meters)
            max_jump_pixels: Max pixels a tower can move between frames
        """
        self.tower_offset = tower_offset_m
        self.max_jump = max_jump_pixels
        
        # State tracking
        self.left_tower_last = None   # (depth_m, pixel_x, pixel_y)
        self.right_tower_last = None
        self.robot_yaw_last = None    # IMU yaw in degrees
        self.initialized = False
        self.frames_since_init = 0
        
    def initialize(self, detected_pairs, robot_yaw):
        """
        Initialize tracker with robot at known position.
        
        Args:
            detected_pairs: List of (depth_m, centroid_x, centroid_y) tuples
            robot_yaw: Current IMU yaw angle in degrees
        
        Returns:
            (left_tower, right_tower) or (None, None) if failed
        """
        if len(detected_pairs) != 2:
            return None, None
            
        # Sort by X position (left tower should be leftmost in image)
        pairs_sorted = sorted(detected_pairs, key=lambda p: p[1])
        
        self.left_tower_last = pairs_sorted[0]
        self.right_tower_last = pairs_sorted[1]
        self.robot_yaw_last = robot_yaw
        self.initialized = True
        self.frames_since_init = 0
        
        return self.left_tower_last, self.right_tower_last
    
    def associate_towers(self, detected_pairs, robot_yaw):
        """
        Main association logic: match detected LED pairs to LEFT/RIGHT towers.
        
        Args:
            detected_pairs: List of (depth_m, centroid_x, centroid_y) tuples
            robot_yaw: Current IMU yaw angle in degrees
        
        Returns:
            (left_tower, right_tower) - either can be None if not detected
        """
        # First frame: initialize
        if not self.initialized:
            return self.initialize(detected_pairs, robot_yaw)
        
        self.frames_since_init += 1
        
        # No towers detected
        if len(detected_pairs) == 0:
            return None, None
        
        # One tower detected
        if len(detected_pairs) == 1:
            return self._identify_single_tower(detected_pairs[0], robot_yaw)
        
        # Two or more towers detected
        return self._identify_two_towers(detected_pairs, robot_yaw)
    
    def _identify_single_tower(self, detected, robot_yaw):
        """
        Determine if single detected tower is LEFT or RIGHT.
        Uses distance to last known positions.
        """
        if self.left_tower_last is None or self.right_tower_last is None:
            # Can't identify without history - guess based on X position
            if detected[1] < 320:  # Left side of image (assuming 640px width)
                return detected, None
            else:
                return None, detected
        
        # Calculate distance to last known positions (using pixel coordinates)
        dist_to_left = np.sqrt((detected[1] - self.left_tower_last[1])**2 + 
                               (detected[2] - self.left_tower_last[2])**2)
        dist_to_right = np.sqrt((detected[1] - self.right_tower_last[1])**2 + 
                                (detected[2] - self.right_tower_last[2])**2)
        
        # Associate with nearest
        if dist_to_left < dist_to_right:
            self.left_tower_last = detected
            return detected, None
        else:
            self.right_tower_last = detected
            return None, detected
    
    def _identify_two_towers(self, detected_pairs, robot_yaw):
        """
        Match two detected pairs to LEFT/RIGHT using nearest-neighbor tracking.
        """
        if len(detected_pairs) > 2:
            # More than 2 detected - take the two largest/brightest
            # For now, just take first two after sorting by X
            detected_pairs = detected_pairs[:2]
        
        # Sort by X position
        pairs_sorted = sorted(detected_pairs, key=lambda p: p[1])
        candidate_left = pairs_sorted[0]
        candidate_right = pairs_sorted[1]
        
        # Validation 1: Check jump distance (continuity check)
        if self.left_tower_last is not None:
            left_jump = np.sqrt((candidate_left[1] - self.left_tower_last[1])**2 + 
                               (candidate_left[2] - self.left_tower_last[2])**2)
            
            if left_jump > self.max_jump:
                # Possible swap or tracking loss - use geometric validation
                # For now, trust X-position sorting
                pass
        
        # Update tracking state
        self.left_tower_last = candidate_left
        self.right_tower_last = candidate_right
        self.robot_yaw_last = robot_yaw
        
        return candidate_left, candidate_right
    
    def calculate_robot_center(self, left_tower, right_tower, robot_yaw, K=None):
        """
        Calculate robot center position from tower positions and IMU orientation.
        Works with BOTH towers or just ONE tower.
        
        Args:
            left_tower: (depth_m, pixel_x, pixel_y) or None
            right_tower: (depth_m, pixel_x, pixel_y) or None
            robot_yaw: Current yaw in degrees
            K: Camera intrinsic matrix (optional, for better accuracy)
        
        Returns:
            (center_depth_m, center_x_pixel, center_y_pixel, method) or None if invalid
            method: "both_towers", "left_only", "right_only"
        """
        # Case 1: Both towers detected
        if left_tower is not None and right_tower is not None:
            avg_depth = (left_tower[0] + right_tower[0]) / 2.0
            avg_x = (left_tower[1] + right_tower[1]) / 2.0
            avg_y = (left_tower[2] + right_tower[2]) / 2.0
            return (avg_depth, avg_x, avg_y, "both_towers")
        
        # Case 2: Only one tower detected - use IMU and tower offset
        yaw_rad = np.deg2rad(robot_yaw)
        
        if left_tower is not None:
            # Left tower detected - calculate robot center
            return self._estimate_center_from_tower(left_tower, yaw_rad, is_left=True, K=K)
        
        elif right_tower is not None:
            # Right tower detected - calculate robot center
            return self._estimate_center_from_tower(right_tower, yaw_rad, is_left=False, K=K)
        
        return None
    
    def _estimate_center_from_tower(self, tower, yaw_rad, is_left, K=None):
        """
        Estimate robot center from a single tower using IMU orientation.
        
        The towers are mounted horizontally offset from the robot center.
        In a top-down view (2D horizontal plane):
        - Robot coordinate system: +X forward, +Y left
        - Camera coordinate system: Looking forward, +X right in image
        
        Args:
            tower: (depth_m, pixel_x, pixel_y)
            yaw_rad: Robot yaw in radians (rotation in horizontal plane)
            is_left: True if this is the left tower, False for right
            K: Camera intrinsic matrix (optional)
        
        Returns:
            (center_depth_m, center_x_pixel, center_y_pixel, method)
        """
        depth, px, py = tower
        
        # In the robot's local frame (top-down view, 2D horizontal plane):
        # The robot center is at (0, 0)
        # Left tower is at (0, +tower_offset) - to the left
        # Right tower is at (0, -tower_offset) - to the right
        
        # Vector from TOWER to ROBOT CENTER in robot frame
        if is_left:
            # Left tower is at (0, +offset), so center is at (0, -offset) relative to tower
            offset_robot_y = -self.tower_offset
            method = "left_only"
        else:
            # Right tower is at (0, -offset), so center is at (0, +offset) relative to tower
            offset_robot_y = self.tower_offset
            method = "right_only"
        
        # There's no forward/backward offset between tower and center
        offset_robot_x = 0.0
        
        # Rotate this offset from robot frame to camera frame
        # When yaw = 0: robot faces camera's +Z direction
        # Positive yaw = counterclockwise rotation (left turn)
        cos_yaw = np.cos(yaw_rad)
        sin_yaw = np.sin(yaw_rad)
        
        # 2D rotation matrix application
        # Camera X (horizontal in image) = Robot X * cos - Robot Y * sin
        # Camera Z (depth) = Robot X * sin + Robot Y * cos
        offset_camera_x = offset_robot_x * cos_yaw - offset_robot_y * sin_yaw
        offset_camera_z = offset_robot_x * sin_yaw + offset_robot_y * cos_yaw
        
        # Convert metric offset to pixel offset
        if K is not None:
            fx = K[0, 0]
            cx = K[0, 2]
            # Pixel offset = (metric offset / depth) * focal_length
            pixel_offset_x = (offset_camera_x / depth) * fx
        else:
            # Rough approximation: assume FOV ~60 degrees horizontally, image width 640
            pixels_per_meter_at_depth = 640 / (2 * depth * np.tan(np.deg2rad(30)))
            pixel_offset_x = offset_camera_x * pixels_per_meter_at_depth
        
        # Robot center position in pixels
        center_x = px + pixel_offset_x
        center_y = py  # No vertical offset (towers are vertical, offset is horizontal)
        
        # Depth estimate (tower depth + forward offset)
        center_depth = depth + offset_camera_z
        
        return (center_depth, center_x, center_y, method)
    
    def reset(self):
        """Reset tracker state."""
        self.initialized = False
        self.left_tower_last = None
        self.right_tower_last = None
        self.robot_yaw_last = None
        self.frames_since_init = 0


# ======================================================
# ================== HELPER FUNCTIONS ==================
# ======================================================

def nothing(x):
    pass

def apply_distance_correction(camera_dist_m):
    """Apply any calibration corrections to distance measurement."""
    return camera_dist_m

def get_contour_details(frame_gray, contour):
    """
    Returns the max brightness and bounding box of a contour.
    Used to distinguish real LEDs (brightness ~255) from reflections.
    """
    x, y, w, h = cv2.boundingRect(contour)
    roi = frame_gray[y:y+h, x:x+w]
    
    if roi.size == 0:
        return 0, (x, y, w, h)
    
    min_val, max_val, _, _ = cv2.minMaxLoc(roi)
    return max_val, (x, y, w, h)


# ======================================================
# ================ ROS2 DOCKING NODE ==================
# ======================================================

class IRLedDockingNode(Node):
    """
    ROS2 node that detects IR LEDs via camera, calculates distance between LEDs,
    and publishes robot position.
    """
    
    def __init__(self):
        super().__init__('ir_led_docking')
        
        # Publishers
        self.distance_publisher = self.create_publisher(
            Float64, 
            '/led_distance', 
            10
        )
        self.position_publisher = self.create_publisher(
            PointStamped,
            '/robot_position',
            10
        )
        
        # Compressed image publishers for camera data (reduced latency)
        # Increased queue size to prevent blocking
        self.raw_image_publisher = self.create_publisher(
            CompressedImage,
            '/camera/raw/compressed',
            30
        )
        self.overlay_image_publisher = self.create_publisher(
            CompressedImage,
            '/camera/overlay/compressed',
            30
        )
        
        # Frame processing lock to prevent queue buildup
        self.processing_lock = False
        
        # Subscriber for IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Subscriber for exposure control from Foxglove slider
        self.exposure_subscriber = self.create_subscription(
            Int32,
            '/camera/exposure_setting',
            self.exposure_callback,
            10
        )
        
        # IMU state
        self.robot_yaw = 0.0  # in degrees
        self.last_imu_time = None
        
        # Exposure state (will be set from parameter initially, then updated by callback)
        self.current_exposure = 11  # Default, will be updated from parameter
        
        # Initialize tower tracker
        self.tracker = TowerTracker(tower_offset_m=0.03, max_jump_pixels=100)
        
        # Camera parameters (declare as parameters with defaults)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('led_separation_mm', 25)
        self.declare_parameter('coarse_thresh', 100)
        self.declare_parameter('min_brightness', 240)
        self.declare_parameter('fine_thresh', 200)
        self.declare_parameter('zoom_scale', 8)
        self.declare_parameter('exposure', 11)
        self.declare_parameter('show_display', False)
        self.declare_parameter('calibration_path', '')  # Empty = auto-detect
        
        # Open camera first to get image dimensions for default calibration
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            # Try default camera
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error('Could not open camera')
                raise RuntimeError('Could not open camera')
        
        # Get camera resolution for default calibration
        ret, test_frame = self.cap.read()
        if ret:
            height, width = test_frame.shape[:2]
        else:
            # Default to common resolution if can't read
            width, height = 640, 480
        
        self.get_logger().info(f'Camera opened successfully (index: {camera_index}, resolution: {width}x{height})')
        
        # Load calibration with improved path detection
        calibration_loaded = self._load_calibration(width, height)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        # Minimize camera buffer to reduce latency (read latest frame only)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Get initial exposure from parameter and store it (scaled to camera range)
        exposure_slider = self.get_parameter('exposure').get_parameter_value().integer_value
        if exposure_slider < 1:
            exposure_slider = 1
        
        # Scale 0-15 (slider) to 1-5000 (camera range) - same as callback
        if exposure_slider == 0:
            target_exposure = 10  # Minimum safe value
        else:
            target_exposure = int(exposure_slider * 333)
        
        target_exposure = max(1, min(5000, target_exposure))
        self.current_exposure = target_exposure
        
        # Set initial exposure immediately
        # IMPORTANT: STRICT ORDER OF OPERATIONS
        # 1. Force Manual Mode
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # 2. Set Absolute Exposure
        self.cap.set(cv2.CAP_PROP_EXPOSURE, float(target_exposure))
        self.get_logger().info(f'Initial exposure set - Slider: {exposure_slider}, Scaled: {target_exposure}')
        
        # Display window (optional)
        self.show_display = self.get_parameter('show_display').get_parameter_value().bool_value
        if self.show_display:
            cv2.namedWindow("Tower Tracker Vision")
        
        # Timer for main processing loop
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 Hz
        
        self.get_logger().info('IR LED Docking Node initialized')
    
    def _load_calibration(self, image_width, image_height):
        """
        Load camera calibration files from multiple possible locations.
        If not found, create a reasonable default calibration matrix.
        
        Args:
            image_width: Camera image width in pixels
            image_height: Camera image height in pixels
        
        Returns:
            bool: True if calibration files were loaded, False if using defaults
        """
        # Get calibration path parameter
        calib_path_param = self.get_parameter('calibration_path').get_parameter_value().string_value
        
        # List of possible calibration file locations (in order of preference)
        possible_paths = []
        
        if calib_path_param:
            # User-specified path
            possible_paths.append(calib_path_param)
        
        # Add common locations (in order of preference)
        possible_paths.extend([
            os.path.dirname(__file__),  # Same directory as this Python file (docking_station/docking_station/)
            os.path.join(os.path.dirname(__file__), 'calibration'),  # calibration subdirectory
            os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'docking_station'),
            os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'docking_station', 'calibration'),
            os.path.join(os.getcwd(), 'calibration'),
        ])
        
        # Try to find calibration files
        for calib_dir in possible_paths:
            k_path = os.path.join(calib_dir, 'K_matrix.npy')
            d_path = os.path.join(calib_dir, 'D_coeffs.npy')
            
            if os.path.exists(k_path) and os.path.exists(d_path):
                try:
                    self.K = np.load(k_path)
                    self.D = np.load(d_path)
                    self.get_logger().info(f'Loaded camera calibration from: {calib_dir}')
                    return True
                except Exception as e:
                    self.get_logger().warn(f'Failed to load calibration from {calib_dir}: {e}')
                    continue
        
        # No calibration files found - create reasonable default
        # Estimate focal length based on image width and typical FOV
        # Assuming ~60-70 degree horizontal FOV (common for webcams)
        fov_degrees = 65.0
        focal_length = (image_width / 2.0) / np.tan(np.deg2rad(fov_degrees / 2.0))
        
        # Create default calibration matrix
        self.K = np.array([
            [focal_length, 0.0, image_width / 2.0],
            [0.0, focal_length, image_height / 2.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        
        # Default distortion coefficients (no distortion)
        self.D = np.zeros(4, dtype=np.float64)
        
        self.get_logger().info(
            f'Using default calibration (estimated focal length: {focal_length:.1f}px). '
            f'Place K_matrix.npy and D_coeffs.npy in one of these locations to use custom calibration:\n'
            f'  - {os.path.dirname(__file__)} (same directory as Python file - RECOMMENDED)\n'
            f'  - {os.path.join(os.path.dirname(__file__), "calibration")}\n'
            f'  - {os.path.join(os.path.expanduser("~"), "ros2_ws", "src", "docking_station")}\n'
            f'  - Or set calibration_path parameter'
        )
        return False
    
    def imu_callback(self, msg):
        """
        Callback for IMU data. Extracts yaw from quaternion.
        """
        # Extract yaw from quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Convert quaternion to Euler angles (yaw)
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.robot_yaw = math.degrees(yaw_rad)
        
        self.last_imu_time = self.get_clock().now()
    
    def exposure_callback(self, msg):
        """
        Update camera exposure.
        Maps 0-15 slider -> 1-5000 camera range.
        """
        exposure_value = msg.data
        
        # 1. Scale 0-15 (slider) to 1-5000 (camera range)
        # We use a non-linear scale because low exposure needs more precision
        if exposure_value == 0:
            target_exposure = 10  # Minimum safe value
        else:
            # Simple linear mapping for now:
            # 15 * 333 = ~5000
            target_exposure = int(exposure_value * 333)
            
        # Clamp to hardware limits (found via v4l2-ctl)
        target_exposure = max(1, min(5000, target_exposure))
        
        self.current_exposure = target_exposure
        
        if self.cap.isOpened():
            # IMPORTANT: STRICT ORDER OF OPERATIONS
            
            # 1. Force Manual Mode. 
            # Try '1' first (Common for V4L2 backend). 
            # If your OpenCV version is older, it might expect 0.25, but 1 is standard for this driver output.
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
            
            # 2. Set Absolute Exposure
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(target_exposure))
            
            self.get_logger().info(f'Applied: Mode=Manual(1), Exp={target_exposure}')
    
    def calculate_distance_between_leds(self, left_tower, right_tower):
        """
        Calculate the distance between the two LED towers.
        
        Args:
            left_tower: (depth_m, pixel_x, pixel_y) or None
            right_tower: (depth_m, pixel_x, pixel_y) or None
        
        Returns:
            Distance in meters, or None if both towers not detected
        """
        if left_tower is None or right_tower is None:
            return None
        
        # Calculate 3D distance between the two towers
        # Using depth and pixel positions to estimate 3D coordinates
        depth_left = left_tower[0]
        depth_right = right_tower[0]
        
        # Average depth for distance calculation
        avg_depth = (depth_left + depth_right) / 2.0
        
        # Pixel coordinates
        px_left = left_tower[1]
        px_right = right_tower[1]
        py_left = left_tower[2]
        py_right = right_tower[2]
        
        # Convert pixel difference to meters at this depth
        # Using camera intrinsics if available
        if self.K[0, 0] > 0:  # Valid focal length
            fx = self.K[0, 0]
            pixel_diff_x = abs(px_left - px_right)
            pixel_diff_y = abs(py_left - py_right)
            
            # Convert to meters
            meter_diff_x = (pixel_diff_x / fx) * avg_depth
            meter_diff_y = (pixel_diff_y / fx) * avg_depth
            
            # 3D distance
            distance = math.sqrt(meter_diff_x**2 + meter_diff_y**2 + (depth_left - depth_right)**2)
        else:
            # Fallback: use simple approximation
            pixel_distance = math.sqrt((px_left - px_right)**2 + (py_left - py_right)**2)
            # Rough approximation assuming FOV ~60 degrees
            pixels_per_meter = 640 / (2 * avg_depth * math.tan(math.radians(30)))
            distance = pixel_distance / pixels_per_meter
        
        return distance
    
    def process_frame(self):
        """
        Main processing loop: capture frame, detect LEDs, calculate distance, publish.
        """
        # Skip frame if previous processing is still running (prevent queue buildup)
        if self.processing_lock:
            return
        
        self.processing_lock = True
        
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to read frame from camera')
                self.processing_lock = False
                return
            
            # Exposure is now set in the callback when slider changes
            # Just log the current value for debugging (less frequent logging)
            # Only log every 30 frames (~1 second) to reduce log spam
            if hasattr(self, '_frame_count'):
                self._frame_count += 1
            else:
                self._frame_count = 0
            
            if self._frame_count % 30 == 0:
                actual_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
                auto_exp = self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
                self.get_logger().debug(f'Frame {self._frame_count} - Exposure: {self.current_exposure}, Camera: {actual_exposure}, Auto-exp: {auto_exp}')
            
            # Publish raw frame immediately after capture (before processing)
            try:
                _, raw_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
                raw_image_msg = CompressedImage()
                raw_image_msg.header.stamp = self.get_clock().now().to_msg()
                raw_image_msg.header.frame_id = 'camera_frame'
                raw_image_msg.format = 'jpeg'
                raw_image_msg.data = raw_encoded.tobytes()
                self.raw_image_publisher.publish(raw_image_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish raw compressed image: {e}')
            
            height, width = frame.shape[:2]
            
            # Get parameters
            coarse_thresh = self.get_parameter('coarse_thresh').get_parameter_value().integer_value
            min_brightness_cutoff = self.get_parameter('min_brightness').get_parameter_value().integer_value
            zoom_scale = self.get_parameter('zoom_scale').get_parameter_value().integer_value
            if zoom_scale < 1:
                zoom_scale = 1
            fine_thresh = self.get_parameter('fine_thresh').get_parameter_value().integer_value
            led_sep_mm = self.get_parameter('led_separation_mm').get_parameter_value().integer_value
            
            # Always create display_frame for overlay publishing (even if not showing display)
            display_frame = frame.copy()
            
            # STEP 1: COARSE SEARCH (Find all bright blobs)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, mask_coarse = cv2.threshold(gray, coarse_thresh, 255, cv2.THRESH_BINARY)
            contours_coarse, _ = cv2.findContours(mask_coarse, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            valid_boxes = []
            if contours_coarse:
                for cnt in contours_coarse:
                    if cv2.contourArea(cnt) < 1:
                        continue
                    
                    max_val, rect = get_contour_details(gray, cnt)
                    if max_val >= min_brightness_cutoff:
                        valid_boxes.append(rect)
                        # Always draw on display_frame for overlay publishing
                        x, y, w, h = rect
                        cv2.rectangle(display_frame, (x, y), (x+w, y+h), (255, 0, 0), 1)
            
            detected_pairs = []
            
            # STEP 2: GROUP BLUE BOXES INTO TOWERS
            valid_boxes.sort(key=lambda b: b[0])
            tower_rois = []
            
            if len(valid_boxes) > 0:
                groups = [[valid_boxes[0]]]
                
                for i in range(1, len(valid_boxes)):
                    curr_box = valid_boxes[i]
                    prev_box = valid_boxes[i-1]
                    dist = curr_box[0] - (prev_box[0] + prev_box[2])
                    
                    if dist > 50:
                        groups.append([curr_box])
                    else:
                        groups[-1].append(curr_box)
                
                for g in groups:
                    g_x = min([b[0] for b in g])
                    g_y = min([b[1] for b in g])
                    g_w = max([b[0] + b[2] for b in g]) - g_x
                    g_h = max([b[1] + b[3] for b in g]) - g_y
                    
                    padding = 30
                    roi_x1 = max(0, g_x - padding)
                    roi_y1 = max(0, g_y - padding)
                    roi_x2 = min(width, g_x + g_w + padding)
                    roi_y2 = min(height, g_y + g_h + padding)
                    
                    tower_rois.append((roi_x1, roi_y1, roi_x2, roi_y2))
            
            # PROCESS EACH TOWER ROI SEPARATELY
            for i, (rx1, ry1, rx2, ry2) in enumerate(tower_rois):
                # Always draw on display_frame for overlay publishing
                cv2.rectangle(display_frame, (rx1, ry1), (rx2, ry2), (0, 255, 0), 2)
                
                roi_gray = gray[ry1:ry2, rx1:rx2]
                
                if roi_gray.size > 0:
                    roi_zoomed = cv2.resize(roi_gray, None, fx=zoom_scale, fy=zoom_scale, 
                                          interpolation=cv2.INTER_LINEAR)
                    roi_norm = cv2.normalize(roi_zoomed, None, 0, 255, cv2.NORM_MINMAX)
                    roi_float = roi_norm.astype(float)
                    roi_enhanced = (roi_float ** 2) / 255.0
                    roi_enhanced = roi_enhanced.astype(np.uint8)
                    
                    _, mask_fine = cv2.threshold(roi_enhanced, fine_thresh, 255, cv2.THRESH_BINARY)
                    
                    contours_fine, _ = cv2.findContours(mask_fine.copy(), cv2.RETR_EXTERNAL, 
                                                      cv2.CHAIN_APPROX_SIMPLE)
                    contours_fine = [c for c in contours_fine if cv2.contourArea(c) > (2 * zoom_scale)]
                    
                    centroids_tower = []
                    for c in contours_fine:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cz_x = M["m10"] / M["m00"]
                            cz_y = M["m01"] / M["m00"]
                            orig_x = rx1 + (cz_x / zoom_scale)
                            orig_y = ry1 + (cz_y / zoom_scale)
                            centroids_tower.append((orig_x, orig_y))
                            # Always draw on display_frame for overlay publishing
                            cv2.circle(display_frame, (int(orig_x), int(orig_y)), 3, (0, 0, 255), -1)
                    
                    if len(centroids_tower) >= 2:
                        max_d = -1
                        pt1_final = None
                        pt2_final = None
                        
                        for j in range(len(centroids_tower)):
                            for k in range(j+1, len(centroids_tower)):
                                p1 = np.array(centroids_tower[j])
                                p2 = np.array(centroids_tower[k])
                                d = np.linalg.norm(p1 - p2)
                                if d > max_d:
                                    max_d = d
                                    pt1_final = p1
                                    pt2_final = p2
                        
                        if pt1_final is not None:
                            points_raw = np.array([[pt1_final], [pt2_final]], dtype=np.float64)
                            points_undist = cv2.fisheye.undistortPoints(points_raw, self.K, self.D)
                            
                            p1n = points_undist[0][0]
                            p2n = points_undist[1][0]
                            
                            ray1 = np.array([p1n[0], p1n[1], 1.0])
                            ray2 = np.array([p2n[0], p2n[1], 1.0])
                            
                            dot = np.dot(ray1, ray2)
                            norm = np.linalg.norm(ray1) * np.linalg.norm(ray2)
                            theta = np.arccos(np.clip(dot/norm, -1, 1))
                            
                            if theta > 1e-5:
                                depth = ((led_sep_mm/1000.0) / 2.0) / np.tan(theta/2.0)
                                depth = apply_distance_correction(depth)
                                
                                cx = (pt1_final[0] + pt2_final[0]) / 2.0
                                cy = (pt1_final[1] + pt2_final[1]) / 2.0
                                
                                detected_pairs.append((depth, cx, cy))
            
            # STEP 3: ASSOCIATE TOWERS
            left_tower, right_tower = self.tracker.associate_towers(detected_pairs, self.robot_yaw)
            
            # STEP 4: CALCULATE AND PUBLISH DISTANCE
            led_distance = self.calculate_distance_between_leds(left_tower, right_tower)
            if led_distance is not None:
                distance_msg = Float64()
                distance_msg.data = led_distance
                self.distance_publisher.publish(distance_msg)
                self.get_logger().debug(f'Published LED distance: {led_distance:.3f}m')
            
            # STEP 5: CALCULATE AND PUBLISH POSITION (STUB)
            # TODO: Implement actual position calculation method
            position_msg = PointStamped()
            position_msg.header.stamp = self.get_clock().now().to_msg()
            position_msg.header.frame_id = 'camera_frame'  # Update with actual frame_id
            position_msg.point.x = 0.0  # Stub value
            position_msg.point.y = 0.0  # Stub value
            position_msg.point.z = 0.0  # Stub value
            self.position_publisher.publish(position_msg)
            
            # Always draw annotations on display_frame for overlay publishing
            status_y = 30
            
            if self.tracker.initialized:
                cv2.putText(display_frame, f"Tracker: ACTIVE (Frame {self.tracker.frames_since_init})", 
                           (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "Tracker: INITIALIZING...", 
                           (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            status_y += 30
            cv2.putText(display_frame, f"IMU Yaw: {self.robot_yaw:.1f} deg", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            if led_distance is not None:
                status_y += 30
                cv2.putText(display_frame, f"LED Distance: {led_distance:.3f}m", 
                           (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw tower information
            if left_tower:
                status_y += 30
                cv2.putText(display_frame, f"LEFT: {left_tower[0]:.3f}m @ ({int(left_tower[1])}, {int(left_tower[2])})", 
                           (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(display_frame, (int(left_tower[1]), int(left_tower[2])), 8, (0, 255, 0), 2)
                cv2.putText(display_frame, "L", (int(left_tower[1])-5, int(left_tower[2])-15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if right_tower:
                status_y += 30
                cv2.putText(display_frame, f"RIGHT: {right_tower[0]:.3f}m @ ({int(right_tower[1])}, {int(right_tower[2])})", 
                           (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.circle(display_frame, (int(right_tower[1]), int(right_tower[2])), 8, (255, 0, 0), 2)
                cv2.putText(display_frame, "R", (int(right_tower[1])-5, int(right_tower[2])-15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # Draw line between towers
            if left_tower and right_tower:
                cv2.line(display_frame, 
                        (int(left_tower[1]), int(left_tower[2])), 
                        (int(right_tower[1]), int(right_tower[2])), 
                        (0, 255, 255), 2)
            
            # Publish compressed overlay frame with annotations (lower quality for speed)
            try:
                _, overlay_encoded = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
                overlay_image_msg = CompressedImage()
                overlay_image_msg.header.stamp = self.get_clock().now().to_msg()
                overlay_image_msg.header.frame_id = 'camera_frame'
                overlay_image_msg.format = 'jpeg'
                overlay_image_msg.data = overlay_encoded.tobytes()
                self.overlay_image_publisher.publish(overlay_image_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish overlay compressed image: {e}')
            
            # Show display window (if enabled)
            if self.show_display:
                cv2.imshow("Tower Tracker Vision", display_frame)
                cv2.waitKey(1)
        
        finally:
            # Always release lock, even if exception occurs
            self.processing_lock = False
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        if self.cap is not None:
            self.cap.release()
        if self.show_display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IRLedDockingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

