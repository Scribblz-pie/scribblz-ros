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

    # micro-ROS agent
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888', '-i', 'wlan0']
    )

    # Foxglove bridge
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        docking_node,
        ir_led_docking_node,
        heartbeat_listener_node,
        teleop_node,
        udp_command_sender_node,
        micro_ros_agent_node,
        foxglove_node
    ])

