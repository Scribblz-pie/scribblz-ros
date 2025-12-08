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
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
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
            'raspi_counter = docking_station.raspi_counter_pub:main',
            'ir_led_docking = docking_station.ir_led_docking_node:main',
            'heartbeat_listener = docking_station.heartbeat_listener:main',
        ],
    },
)
