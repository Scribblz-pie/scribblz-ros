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
        micro_ros_agent_node,
        foxglove_node
    ])

