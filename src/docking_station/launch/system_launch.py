from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    docking_node = Node(
        package='docking_station',
        executable='raspi_counter', 
        name='raspi_counter',
        output='screen'
    )

    # IR LED docking node
    ir_led_docking_node = Node(
        package='docking_station',
        executable='ir_led_docking',
        name='ir_led_docking',
        output='screen'
    )

    # Heartbeat listener node (UDP communication with Arduino)
    heartbeat_listener_node = Node(
        package='docking_station',
        executable='heartbeat_listener',
        name='heartbeat_listener',
        output='screen'
    )

    # Teleop node (converts joystick input to motor commands via IK)
    teleop_node = Node(
        package='docking_station',
        executable='teleop',
        name='teleop',
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

    # micro-ROS agent (disabled)
    # micro_ros_agent_node = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name='micro_ros_agent',
    #     output='screen',
    #     arguments=['udp4', '--port', '8888', '-i', 'wlan0']
    # )

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

    # Drawing action server node
    drawing_action_server_node = Node(
        package='docking_station',
        executable='drawing_action_server',
        name='drawing_action_server',
        output='screen',
        parameters=[{
            'waypoint_tolerance': 0.05,
            'max_linear_vel': 0.3,
            'max_angular_vel': 0.5,
            'control_frequency': 20.0,
            'target_fan_speed': 255,
            'fan_ramp_steps': 10,
            'fan_ramp_delay': 0.1,
            'imu_reset_topic': '/imu/reset',
            'initialization_timeout': 5.0
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

    return LaunchDescription([
        docking_node,
        ir_led_docking_node,
        heartbeat_listener_node,
        teleop_node,
        udp_command_sender_node,
        foxglove_node,
        state_machine_node,
        battery_monitor_node,
        drawing_action_server_node,
        docking_action_server_node
    ])

