from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_multi_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='canozkan',
    maintainer_email='can.ozkan.de@gmail.com',
    description='Warehouse multi-robot simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_relay = warehouse_multi_robot.imu_relay:main',
            'initial_pose_pub = warehouse_multi_robot.initial_pose_publisher:main',
            'waypoint_sender = warehouse_multi_robot.waypoint_sender:main',
            'battery_monitor = warehouse_multi_robot.battery_monitor:main',
        ],
    },
)
