import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def patch_urdf(urdf: str, robot_name: str) -> str:
    """
    Adds the 'robot_name/' prefix to link names within the URDF. 

    The RSP namespace mechanism does not affect TF frame names. 
    The 'frame_prefix' option, however, applies the prefix twice (URDF + prefix = double). 
    The only correct approach: Patching the link names within the URDF on a per-robot basis. 

    Modified tags:
    <link name="X">           ->  <link name="robot1/X">
    <parent link="X">         ->  <parent link="robot1/X">
    <child link="X">          ->  <child link="robot1/X">

    Result in RSP 'tf_static':
    robot1/base_footprint -> robot1/base_link   
    robot1/base_link      -> robot1/base_scan   
    """
    prefix = robot_name + '/'
 
    def add_prefix(m):
        tag_start = m.group(1)   # '<link name="'
        link_name = m.group(2)   # 'base_footprint'
        tag_end   = m.group(3)   # '"'

        if link_name.startswith(prefix):
            return m.group(0)
        return f'{tag_start}{prefix}{link_name}{tag_end}'
 
    # <link name="...">
    urdf = re.sub(r'(<link\s+name=")([\w/]+)(")', add_prefix, urdf)
    # <parent link="...">
    urdf = re.sub(r'(<parent\s+link=")([\w/]+)(")', add_prefix, urdf)
    # <child link="...">
    urdf = re.sub(r'(<child\s+link=")([\w/]+)(")', add_prefix, urdf)
 
    return urdf

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

    xacro_file = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    
    # Raw URDF - no namespace
    base_urdf = subprocess.check_output(
        ['xacro', xacro_file],
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
        'robot1': 0.01,
        'robot2': 0.01,
        'robot3': 0.01,
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
        delay_nav2  = 45.0 + i * 15.0

        # URDF patch
        robot_desc = patch_urdf(base_urdf, name)

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


        # RSP: namespace=name, no frame_prefix, tf_static directly to global 
        actions.append(TimerAction(period=delay_spawn + 2.0, actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'robot_state_publisher',
                namespace=name, 

                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True,
                    'publish_frequency': 50.0,
                }],
                remappings=[
                # Write directly to global /tf and /tf_static - no relay
                    ('tf',        '/tf'),
                    ('tf_static', '/tf_static'),
                ],
                output='screen',
            )
        ]))

        # TF relay: Gazebo odom TF (/robotN/tf -> /tf) 
        actions.append(TimerAction(period=delay_spawn + 2.5, actions=[
            Node(package='topic_tools', executable='relay',
                 name=f'tf_relay_{name}',
                 arguments=[f'/{name}/tf', '/tf'],
                 output='screen')
        ]))

        # tf_static relay 
        actions.append(TimerAction(period=delay_spawn + 2.5, actions=[
            Node(package='topic_tools', executable='relay',
                 name=f'tf_static_relay_{name}',
                 arguments=[f'/{name}/tf_static', '/tf_static'],
                 output='screen')
        ]))

        # Nav2 stack 
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
                        'odom_frame_id':  f'{name}/odom',
                        'base_frame_id':  f'{name}/base_footprint',
                        'global_frame_id': 'map',
                        'set_initial_pose': True,
                        'initial_pose.x': float(robot['x']),
                        'initial_pose.y': float(robot['y']),
                        'initial_pose.z': 0.0,
                        'initial_pose.yaw': 0.0,
                        'initial_pose.covariance_x': 0.25,
                        'initial_pose.covariance_y': 0.25,
                        'initial_pose.covariance_yaw': 0.1,
                    }],
                    remappings=[('scan', 'scan')],
                    output='screen'),

                Node(package='nav2_planner', executable='planner_server',
                     name='planner_server', parameters=[nav2_yamls[name]],
                     output='screen'),

                Node(package='nav2_controller', executable='controller_server',
                    name='controller_server', 
                    parameters=[nav2_yamls[name],{
                        'transform_tolerance': 1.0,
                    }],
                    remappings=[
                        ('cmd_vel', 'cmd_vel'),
                        ('scan',    'scan'),
                        ('odom',    'odom'),
                    ],
                    output='screen'),

                Node(package='nav2_behaviors', executable='behavior_server',
                     name='behavior_server', parameters=[nav2_yamls[name]],
                     output='screen'),

                Node(package='nav2_bt_navigator', executable='bt_navigator',
                    name='bt_navigator',
                    parameters=[nav2_yamls[name], {
                        'global_frame':      'map',
                        'robot_base_frame':  f'{name}/base_link',
                        'odom_topic':        'odom',
                        'navigators': ['navigate_to_pose', 'navigate_through_poses'],
                        'navigate_to_pose':
                            {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'},
                        'navigate_through_poses':
                            {'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'},
                    }],
                    remappings=[('odom', 'odom')],
                    output='screen'),

                Node(package='nav2_lifecycle_manager',
                     executable='lifecycle_manager',
                     name='lifecycle_manager_navigation',
                     parameters=[{
                         'use_sim_time': True,
                         'autostart':    True,
                         'bond_timeout': 30.0,
                         'node_names': ['map_server', 'amcl', 'planner_server',
                                        'controller_server', 'behavior_server',
                                        'bt_navigator'],
                     }],
                     output='screen'),
            ])
        ]))

        # WaypointSender
        actions.append(TimerAction(period=delay_nav2 + 40.0, actions=[
            Node(
                package='warehouse_multi_robot',
                executable='waypoint_sender',
                name='waypoint_sender',
                namespace=name,
                parameters=[{'robot_name': name}],
                output='screen',
            )
        ]))

        # Agent Coordinator
        actions.append(TimerAction(period=delay_nav2 + 40.0, actions=[
            Node(
                package='warehouse_multi_robot',
                executable='agent_coordinator',
                name='agent_coordinator',
                namespace=name,
                parameters=[{'robot_name': name}],
                output='screen',
            )
        ]))

        # Battery Monitor
        actions.append(
            Node(
                package='warehouse_multi_robot',
                executable='battery_monitor',
                name=f'battery_monitor_{name}',
                namespace=name,
                parameters=[{
                    'robot_name': name,
                    'drain_rate': DRAIN_RATES[name],
                    'fail_at':    0.0,
                    'start_at':   100.0,
                }],
                output='screen',
                emulate_tty=True,
            )
        )

    return LaunchDescription(actions)