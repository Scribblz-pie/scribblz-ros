from glob import glob
from setuptools import find_packages, setup

package_name = 'docking_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/waypoints', glob('waypoints/*.json')),
    ],

    install_requires=['setuptools', 'scipy', 'opencv-python'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
    'test': [
    'pytest',
    ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = docking_station.teleop_node:main',
            'udp_command_sender = docking_station.udp_command_sender:main',
            'imu_udp_receiver = docking_station.imu_udp_receiver_node:main',
            'state_machine = docking_station.state_machine_node:main',
            'path_follower = docking_station.drawing.path_follower:main',
            'kinematics = docking_station.drawing.kinematics:main',
            'image_processor = docking_station.drawing.image_processor:main',
            'path_publisher = docking_station.drawing.path_publisher:main',
            'docking_action_server = docking_station.docking_action_server_node:main',
            'lidar_pose = docking_station.lidar.lidar_node:main',
            'tether_controller = docking_station.drawing.tether_controller:main',
        ],
    },
)