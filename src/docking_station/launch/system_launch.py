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
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # UDP command sender (sends motor commands to Arduino)
    udp_command_sender_node = Node(
        package='docking_station',
        executable='udp_command_sender',
        name='udp_command_sender',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Foxglove bridge
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # State machine node
    state_machine_node = Node(
        package='docking_station',
        executable='state_machine',
        name='state_machine',
        output='screen'
    )

    # Battery monitor node
    battery_monitor_node = Node(
        package='docking_station',
        executable='battery_monitor',
        name='battery_monitor',
        output='screen',
        parameters=[{
            'battery_topic': '/battery_voltage',
            'low_battery_threshold': 0.2,
            'voltage_min': 10.0,
            'voltage_max': 12.6
        }]
    )

    # Waypoint publisher node
    waypoint_publisher_node = Node(
        package='docking_station',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        output='screen',
        parameters=[{
            'waypoints_file': 'waypoints.json',
            'watch_file': True,
            'poll_interval': 1.0
        }]
    )

    # Drawing control node
    drawing_control_node = Node(
        package='docking_station',
        executable='drawing_control',
        name='drawing_control',
        output='screen',
        parameters=[{
            'waypoint_tolerance': 0.05,
            'max_linear_vel': 0.3,
            'kp': 2.0,
            'control_frequency': 20.0,
            'obstacle_threshold': 0.15
        }]
    )

    # Drawing driver node
    drawing_driver_node = Node(
        package='docking_station',
        executable='drawing_driver',
        name='drawing_driver',
        output='screen',
        parameters=[{
            'waypoints_file': 'waypoints.json',
            'use_imu': True
        }]
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
            'scan_frame_id': 'lidar',
            'print_interval': 2.0,
            'expected_radius_cm': 3.0,
            'radius_tolerance_cm': 5.0,
            'min_circle_points': 3,
            'ransac_iterations': 50,
            'inlier_threshold_cm': 2.0,
            'min_detection_distance_cm': 5.0,
            'max_detection_distance_cm': 500.0
        }]
    )

    return LaunchDescription([
        teleop_node,
        udp_command_sender_node,
        foxglove_node,
        state_machine_node,
        battery_monitor_node,
        waypoint_publisher_node,
        drawing_control_node,
        drawing_driver_node,
        docking_action_server_node,
        lidar_pose_node
    ])

