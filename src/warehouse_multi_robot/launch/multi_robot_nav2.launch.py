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

    bridge_yamls = {
        'robot1': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/bridge_robot1.yaml',
        'robot2': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/bridge_robot2.yaml',
        'robot3': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/bridge_robot3.yaml',
    }
    nav2_yamls = {
        'robot1': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/nav2_params_robot1.yaml',
        'robot2': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/nav2_params_robot2.yaml',
        'robot3': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/nav2_params_robot3.yaml',
    }
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
        {'name': 'robot1', 'x': '0.0', 'y':  '1.0',
        'sdf': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/models/turtlebot3_waffle/model_robot1.sdf'},
        {'name': 'robot2', 'x': '0.0', 'y':  '0.0',
        'sdf': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/models/turtlebot3_waffle/model_robot2.sdf'},
        {'name': 'robot3', 'x': '0.0', 'y': '-1.0',
        'sdf': '/home/canozkan/thesis_ws/src/warehouse_multi_robot/models/turtlebot3_waffle/model_robot3.sdf'},
    ]

    DRAIN_RATES = {
        'robot1': 0.5,   # finishes ~200 seconds 
        'robot2': 0.5,
        'robot3': 0.5,
    }

    actions = [
        SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', fuel_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world_path],
                'on_exit_shutdown': 'true'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items()
        ),

        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='clock_bridge',
                 arguments=['--ros-args', '-p', f'config_file:={clock_yaml}'],
                 output='screen')
        ]),
    ]

    for i, robot in enumerate(robots):
        name = robot['name']
        delay_spawn = 5.0  + i * 3.0
        delay_nav2  = 30.0 + i * 8.0

        # Spawn
        actions.append(TimerAction(period=delay_spawn, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', name, '-file', robot['sdf'],
                            '-x', robot['x'], '-y', robot['y'], '-z', '0.01'],
                 output='screen')
        ]))

        # Bridge
        actions.append(TimerAction(period=delay_spawn + 2.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name=f'bridge_{name}',
                 arguments=['--ros-args', '-p', f'config_file:={bridge_yamls[name]}'],
                 output='screen')
        ]))

        # RSP
        actions.append(TimerAction(period=delay_spawn + 2.0, actions=[
        Node(package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=name,
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
                'frame_prefix': f'{name}/'  
            }],
            output='screen')
        ]))

        # TF relay
        actions.append(TimerAction(period=delay_spawn + 3.0, actions=[
            Node(package='topic_tools', executable='relay',
                 name=f'tf_relay_{name}',
                 arguments=[f'/{name}/tf', '/tf'],
                 output='screen')
        ]))

        # Static map->odom TF 
        actions.append(TimerAction(period=delay_spawn + 3.0, actions=[
        Node(package='tf2_ros', executable='static_transform_publisher',
            name=f'map_odom_{name}',
            arguments=['--x', robot['x'], '--y', robot['y'], '--z', '0.0',
                        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                        '--frame-id', 'map', '--child-frame-id', f'{name}/odom'],
            output='screen')
        ]))

        # Nav2 nodes 
        actions.append(TimerAction(period=delay_nav2, actions=[
            GroupAction(actions=[
                PushRosNamespace(name),

                Node(package='nav2_map_server', executable='map_server',
                     name='map_server',
                     parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
                     output='screen'),

                Node(package='nav2_amcl', executable='amcl', name='amcl',
                parameters=[nav2_yamls[name], {
                    'tf_broadcast': True,
                    'odom_frame_id': f'{name}/odom',
                    'base_frame_id': f'{name}/base_footprint',
                    'global_frame_id': 'map',
                }],
                remappings=[('scan', 'scan')],
                output='screen'),

                Node(package='nav2_planner', executable='planner_server',
                     name='planner_server', parameters=[nav2_yamls[name]],
                     output='screen'),

                Node(package='nav2_controller', executable='controller_server',
                name='controller_server', parameters=[nav2_yamls[name]],
                remappings=[('cmd_vel', 'cmd_vel'),
                            ('scan',    'scan'),
                            ('odom',    'odom')],
                output='screen'),

                Node(package='nav2_behaviors', executable='behavior_server',
                     name='behavior_server', parameters=[nav2_yamls[name]],
                     output='screen'),

                Node(package='nav2_bt_navigator', executable='bt_navigator',
                name='bt_navigator',
                parameters=[nav2_yamls[name], {
                    'global_frame': 'map',
                    'robot_base_frame': f'{name}/base_link',
                    'odom_topic': 'odom',
                    'navigators': ['navigate_to_pose', 'navigate_through_poses'],
                    'navigate_to_pose': {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'},
                    'navigate_through_poses': {'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'},
                }],
                remappings=[('odom', 'odom')],
                output='screen'),

                Node(package='nav2_lifecycle_manager',
                     executable='lifecycle_manager',
                     name='lifecycle_manager_navigation',
                     parameters=[{
                         'use_sim_time': True,
                         'autostart': True,
                         'bond_timeout': 4.0,
                         'node_names': ['map_server', 'amcl', 'planner_server',
                                        'controller_server', 'behavior_server',
                                        'bt_navigator']
                     }],
                     output='screen'),
            ])
        ]))
        
        
        # Battery Depletion
        actions.append(
            Node(
                package='warehouse_multi_robot',
                executable='battery_monitor',
                name=f'battery_monitor_{name}',
                namespace=robot,
                parameters=[{
                    'robot_name': name,
                    'drain_rate': DRAIN_RATES[name],
                    'fail_at': 0.0,     # 0.0 = fail at total depletion
                    'start_at': 100.0,  # start %100 full
                }],
                output='screen',
                emulate_tty=True,
            )
        )
        
        actions.append(TimerAction(period=delay_nav2 + 8.0, actions=[
        Node(package='warehouse_multi_robot',
            executable='initial_pose_pub',
            name=f'initial_pose_pub_{name}',
            parameters=[{
                'robot_name': name,
                'x': float(robot['x']),
                'y': float(robot['y']),
            }],
            output='screen')
    ]))

    return LaunchDescription(actions)
