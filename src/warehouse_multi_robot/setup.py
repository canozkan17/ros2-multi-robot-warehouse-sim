#!/usr/bin/env python3
import os
from setuptools import find_packages, setup

package_name = 'warehouse_multi_robot'

def generate_data_files():
    """
    Recursively collects all asset files from resource folders (launch, config, maps, worlds, models)
    and maps them to their correct relative paths in the ROS2 install/share directory.
    """
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Target directories to scan and pack under share/<package_name>/
    resource_directories = ['launch', 'config', 'maps', 'worlds', 'models']
    
    for directory in resource_directories:
        for root, dirs, files in os.walk(directory):
            if files:
                # Calculate the exact destination path in the install share directory
                install_dir = os.path.join('share', package_name, root)
                file_paths = [os.path.join(root, f) for f in files]
                data_files.append((install_dir, file_paths))
                
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='canozkan',
    maintainer_email='canozkan17@users.noreply.github.com',
    description='Decentralized Fault-Tolerant Multi-Robot Warehouse Inspection Simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_sender = warehouse_multi_robot.waypoint_sender:main',
            'battery_monitor = warehouse_multi_robot.battery_monitor:main',
            'mission_gate = warehouse_multi_robot.mission_gate:main',
            'imu_relay = warehouse_multi_robot.imu_relay:main',
            'monitoring_dashboard = warehouse_multi_robot.monitoring_dashboard:main',
            'window_tiler = warehouse_multi_robot.window_tiler:main',
        ],
    },
)