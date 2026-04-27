import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    tb3_gazebo  = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim  = get_package_share_directory('ros_gz_sim')
    tb3_desc    = get_package_share_directory('turtlebot3_description')

    urdf_path   = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/models/turtlebot3_waffle/model.sdf'
    bridge_yaml = os.path.join(tb3_gazebo, 'params', 'turtlebot3_waffle_bridge.yaml')
    nav2_yaml   = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/nav2_params.yaml'
    map_yaml    = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/maps/warehouse_map.yaml'
    world_path  = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf'
    fuel_path   = '/home/canozkan/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/tugbot in warehouse/2/'
    clock_yaml  = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/bridge_clock.yaml'

    xacro_file  = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    robot_desc  = subprocess.check_output(
        ['xacro', xacro_file, 'namespace:='],
        stderr=subprocess.DEVNULL
    ).decode('utf-8')

    robots = [
        {'name': 'robot1', 'x': '0.0',  'y':  '1.0'},
        {'name': 'robot2', 'x': '0.0',  'y':  '0.0'},
        {'name': 'robot3', 'x': '0.0',  'y': '-1.0'},
    ]

    return LaunchDescription([
        SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', fuel_path),

        # 1. Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world_path],
                'on_exit_shutdown': 'true'
            }.items()
        ),

        # 2. Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-g -v2',
                'on_exit_shutdown': 'true'
            }.items()
        ),

        # 3. Clock bridge
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                arguments=['--ros-args', '-p', f'config_file:={clock_yaml}'],
                output='screen'
            )
        ]),

        # 4. Spawn robots + bridge + RSP + Nav2 per robot
        *[item for robot in robots for item in [

            # Spawn
            TimerAction(period=5.0 + robots.index(robot) * 2.0, actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', robot['name'],
                        '-file', urdf_path,
                        '-x', robot['x'],
                        '-y', robot['y'],
                        '-z', '0.01'
                    ],
                    output='screen'
                )
            ]),

            # Sensor bridge (namespaced)
            TimerAction(period=8.0 + robots.index(robot) * 2.0, actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'bridge_{robot["name"]}',
                    namespace=robot['name'],
                    arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
                    output='screen'
                )
            ]),

            # Robot state publisher
            TimerAction(period=8.0 + robots.index(robot) * 2.0, actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    namespace=robot['name'],
                    parameters=[{
                        'robot_description': robot_desc,
                        'use_sim_time': True,
                        'frame_prefix': robot['name'] + '/'
                    }],
                    output='screen'
                )
            ]),

            # Nav2 stack for this robot
            TimerAction(period=14.0 + robots.index(robot) * 2.0, actions=[
                GroupAction(actions=[
                    PushRosNamespace(robot['name']),

                    # Map server
                    Node(
                        package='nav2_map_server',
                        executable='map_server',
                        name='map_server',
                        parameters=[{
                            'use_sim_time': True,
                            'yaml_filename': map_yaml
                        }],
                        output='screen'
                    ),

                    # AMCL localizer
                    Node(
                        package='nav2_amcl',
                        executable='amcl',
                        name='amcl',
                        parameters=[nav2_yaml],
                        remappings=[('/scan', f'/{robot["name"]}/scan'),
                                    ('/tf',   '/tf'),
                                    ('/tf_static', '/tf_static')],
                        output='screen'
                    ),

                    # Planner
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        parameters=[nav2_yaml],
                        output='screen'
                    ),

                    # Controller
                    Node(
                        package='nav2_controller',
                        executable='controller_server',
                        name='controller_server',
                        parameters=[nav2_yaml],
                        remappings=[('/cmd_vel', f'/{robot["name"]}/cmd_vel'),
                                    ('/scan',    f'/{robot["name"]}/scan'),
                                    ('/odom',    f'/{robot["name"]}/odom')],
                        output='screen'
                    ),

                    # Behaviors
                    Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='behavior_server',
                        parameters=[nav2_yaml],
                        output='screen'
                    ),

                    # BT Navigator
                    Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        parameters=[nav2_yaml],
                        output='screen'
                    ),

                    # Lifecycle manager
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation',
                        parameters=[{
                            'use_sim_time': True,
                            'autostart': True,
                            'node_names': [
                                'map_server',
                                'amcl',
                                'planner_server',
                                'controller_server',
                                'behavior_server',
                                'bt_navigator'
                            ]
                        }],
                        output='screen'
                    ),
                ])
            ]),
        ]],
    ])
