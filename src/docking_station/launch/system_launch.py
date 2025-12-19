from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Teleop node (converts joystick input to motor commands via IK)
    teleop_node = Node(
        package='docking_station',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[{'base_length': 0.1}],  # Adjust based on your robot
        arguments=['--ros-args', '--log-level', 'info']
    )

    # UDP command sender (sends motor commands to Arduino)
    udp_command_sender_node = Node(
        package='docking_station',
        executable='udp_command_sender',
        name='udp_command_sender',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Foxglove bridge
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
        'topic_whitelist': ['.*'],  # Subscribe to ALL topics
        # Or remove the best_effort line if you want reliable QoS
    }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # State machine node
    state_machine_node = Node(
        package='docking_station',
        executable='state_machine',
        name='state_machine',
        output='screen'
    )

    # Drawing control node
    # drawing_control_node = Node(
    #     package='docking_station',
    #     executable='drawing_control',
    #     name='drawing_control',
    #     output='screen',
    #     parameters=[{
    #         'waypoint_tolerance': 0.05,
    #         'max_linear_vel': 0.3,
    #         'kp': 2.0,
    #         'control_frequency': 20.0,
    #         'obstacle_threshold': 0.15
    #     }]
    # )

    # Kinematics node (converts world-frame velocities to wheel velocities)
    kinematics_node = Node(
        package='docking_station',
        executable='kinematics',
        name='kinematics',
        output='screen',
        parameters=[{
            'waypoints_file': 'waypoints.json',
            'use_imu': True
        }]
    )

    # Image processor node (converts images to drawing paths)
    image_processor_node = Node(
        package='docking_station',
        executable='image_processor',
        name='image_processor',
        output='screen',
        parameters=[{
            'dock_position_x': -0.1,
            'dock_position_y': 0.3,
            'dock_approach_x': 0.144,
            'dock_approach_y': 0.175,
            'dock_orientation': -1.5708,
            'drawing_x_shift': 0.05,
            'drawing_y_shift': 0.31
        }]
    )

    # Path follower node (executes drawing paths with PID control)
    path_follower_node = Node(
        package='docking_station',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[{
            'waypoint_tolerance': 0.05,
            'heading_tolerance': 0.1,
            'max_forward_vel': 0.3,
            'max_lateral_vel': 0.3,
            'max_angular_vel': 0.5,
            'control_frequency': 20.0,
            'kp_forward': 2.0,
            'ki_forward': 0.0,
            'kd_forward': 0.0,
            'kp_lateral': 2.0,
            'ki_lateral': 0.0,
            'kd_lateral': 0.0,
            'kp_heading': 1.0,
            'ki_heading': 0.0,
            'kd_heading': 0.0
        }],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Docking action server node
    docking_action_server_node = Node(
        package='docking_station',
        executable='docking_action_server',
        name='docking_action_server',
        output='screen',
        parameters=[{
            'docking_tolerance': 0.1,
            'max_linear_vel': 0.2,
            'max_angular_vel': 0.3,
            'control_frequency': 20.0
        }]
    )

    lidar_pose_node = Node(
        package='docking_station',
        executable='lidar_pose',
        name='lidar_pose',
        output='screen',
        parameters=[{
            'host': '0.0.0.0',
            'port': 6794,
            'angle_tolerance': 0.05,
            'start_filter_angle': 0.0,
            'end_filter_angle': 360.0,
            'frame_id': 'map',
            'scan_frame_id': 'map',
            'print_interval': 2.0,
            'robot_orientation_offset': -1.5708,  # -90° (or 270°) - robot forward alignment
            'expected_radius_cm': 3.0,
            'radius_tolerance_cm': 3.0,  # Tighter tolerance (was 5.0) for improved accuracy
            'min_circle_points': 5,  # Increased from 3 for more stable fits
            'ransac_iterations': 25,  # Reduced from 50 for faster processing with early termination
            'inlier_threshold_cm': 1.5,  # Tighter threshold (was 2.0) for better precision
            'min_detection_distance_cm': 10.0,  # Increased from 5.0 to avoid very close detections
            'max_detection_distance_cm': 500.0
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Static transform: odom → map (defines map as child of odom for TF tree)
    # This gives us a root frame (odom) that Foxglove can use
    # 180° rotation around x-axis to flip z-axis (view from top instead of bottom)
    odom_to_map_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map_tf',
        arguments=[
            '0', '0', '0',  # translation
            '1', '0', '0', '0',  # 180° rotation around x-axis
            'odom',  # parent frame (root)
            'map'  # child frame (docking station/lidar coordinate system)
        ]
    )

    return LaunchDescription([
        teleop_node,
        udp_command_sender_node,
        foxglove_node,
        state_machine_node,
        # drawing_control_node,
        kinematics_node,
        image_processor_node,
        path_follower_node,
        docking_action_server_node,
        lidar_pose_node,
        odom_to_map_transform
    ])