import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _cleanup_stale_cartographer_processes(_context, *args, **kwargs):
    # Interrupted sessions can leave orphan Cartographer processes behind,
    # causing duplicate /map publishers and unstable visualization.
    stale_patterns = [
        r"/opt/ros/jazzy/lib/cartographer_ros/cartographer_node .*warehouse_cartographer.lua .*__node:=cartographer_node",
        r"/opt/ros/jazzy/lib/cartographer_ros/cartographer_occupancy_grid_node .*__node:=cartographer_occupancy_grid_node",
    ]

    for pattern in stale_patterns:
        subprocess.run(
            ['pkill', '-f', pattern],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    return []

def generate_launch_description():
    tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    tb3_cart     = get_package_share_directory('turtlebot3_cartographer')
    tb3_desc     = get_package_share_directory('turtlebot3_description')
    ros_gz_sim   = get_package_share_directory('ros_gz_sim')

    urdf_path    = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/models/turtlebot3_waffle/model.sdf'
    bridge_yaml  = os.path.join(tb3_gazebo, 'params', 'turtlebot3_waffle_bridge.yaml')
    cart_config_dir = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config'
    cart_rviz    = os.path.join(tb3_cart, 'rviz', 'tb3_cartographer.rviz')
    world_path   = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf'
    fuel_path    = '/home/canozkan/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/tugbot in warehouse/2/'


    # process URDF by xacro - namespace empty
    xacro_file = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    robot_desc = subprocess.check_output(
        ['xacro', xacro_file, 'namespace:='],
        stderr=subprocess.DEVNULL
    ).decode('utf-8')

    return LaunchDescription([

        # -1. Stale Cartographer process cleanup (before starting a new run)
        OpaqueFunction(function=_cleanup_stale_cartographer_processes),

        # 0. GZ model/resource path
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', fuel_path),

        # 1. Gazebo server (physics)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world_path],
                'on_exit_shutdown': 'true'
            }.items()
        ),

        # 2. Gazebo client (visual)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-g -v2',
                'on_exit_shutdown': 'true'
            }.items()
        ),

        # 3. Robot spawn
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'robot1',
                    '-file', urdf_path,
                    '-x', '0.0', '-y', '0.0', '-z', '0.01'
                ],
                output='screen'
            )
        ]),

        # 4. Sensor bridge (single bridge source including clock)
        TimerAction(period=8.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='sensor_bridge',
                arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
                output='screen'
            )
        ]),

        # 5. Robot state publisher (publish TF from URDF)
        TimerAction(period=8.0, actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True
                }],
                output='screen'
            )
        ]),

        TimerAction(period=10.0, actions=[
            Node(
                package='warehouse_multi_robot',
                executable='imu_relay',  
                name='imu_relay',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]),

        # 6. Cartographer (after TF and sensor are ready)
        TimerAction(period=15.0, actions=[
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                parameters=[{'use_sim_time': True}],
                remappings=[('imu', '/imu_fixed')],
                arguments=[
                    '-configuration_directory', cart_config_dir,
                    '-configuration_basename', 'warehouse_cartographer.lua'
                ],
                output='screen'
            )
        ]),

        # 7. Occupancy grid
        TimerAction(period=15.0, actions=[
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                parameters=[{'use_sim_time': True}],
                arguments=['-resolution', '0.05'],
                output='screen'
            )
        ]),

        # 8. RViz
        TimerAction(period=16.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', cart_rviz],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]),
    ])
