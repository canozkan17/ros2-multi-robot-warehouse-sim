#!/usr/bin/env python3
"""
multi_robot.launch.py — Deterministic multi-robot warehouse launch.

Startup sequence
----------------
gz_sim (server + client)
  └─► [gate:world]  Gazebo world is up
        └─► clock_bridge
              └─► [gate:clock]  /clock is publishing
                    └─► per robot, in parallel — each robot owns its full chain:

                        [gate:world] exits → spawn process starts
                          └─► on spawn exit: bridge + rsp + ekf_node
                                               + [gate:topics] process
                                └─► on gate:topics exit: nav2 stack
                                                          + [gate:lifecycle] process
                                      └─► on gate:lifecycle exit: waypoint_sender
                                                                   + agent_coordinator
                                                                   + battery_monitor

Design notes
------------
* Zero TimerAction — every transition is triggered by OnProcessExit.
* gate.py and spawn.py live alongside this file in launch/ and run as
  subprocesses (ExecuteProcess).  They are plain Python files with no ROS
  dependency, making them easy to test independently.
* Each robot's chain is fully independent.
* waypoint_sender has an internal AMCL-freshness gate; it will wait on its
  own until localisation is stable.
"""

import os
import re
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

# gate.py and spawn.py sit next to this file
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
_GATE_PY    = os.path.join(_LAUNCH_DIR, 'gate.py')
_SPAWN_PY   = os.path.join(_LAUNCH_DIR, 'spawn.py')


# ══════════════════════════════════════════════════════════════════════════════
# URDF patching
# ══════════════════════════════════════════════════════════════════════════════

def _patch_urdf(urdf: str, robot_name: str) -> str:
    """
    Prefix every link name in the URDF with '<robot_name>/'.

    robot_state_publisher's frame_prefix option doubles the prefix, so the
    only correct approach is patching the URDF source per robot.

    Tags modified:
        <link name="X">    →  <link name="robot1/X">
        <parent link="X">  →  <parent link="robot1/X">
        <child link="X">   →  <child link="robot1/X">
    """
    prefix = robot_name + '/'

    def _add_prefix(m):
        if m.group(2).startswith(prefix):
            return m.group(0)
        return f'{m.group(1)}{prefix}{m.group(2)}{m.group(3)}'

    urdf = re.sub(r'(<link\s+name=")([\w/]+)(")',   _add_prefix, urdf)
    urdf = re.sub(r'(<parent\s+link=")([\w/]+)(")', _add_prefix, urdf)
    urdf = re.sub(r'(<child\s+link=")([\w/]+)(")',  _add_prefix, urdf)
    return urdf


# ══════════════════════════════════════════════════════════════════════════════
# Gate process factory
# ══════════════════════════════════════════════════════════════════════════════

def _gate(mode, *, world_name=None, topics=None, nodes=None, label=None):
    """
    Return an ExecuteProcess that runs gate.py in the requested mode.

    The process exits with code 0 when the condition is met, which makes it
    a natural trigger for OnProcessExit event handlers.
    """
    cmd = [sys.executable, _GATE_PY, '--mode', mode]
    if world_name:
        cmd += ['--world-name', world_name]
    if topics:
        cmd += ['--topics'] + topics
    if nodes:
        cmd += ['--nodes'] + nodes

    name = label or f'gate_{mode}'
    return ExecuteProcess(cmd=cmd, name=name, output='screen')


# ══════════════════════════════════════════════════════════════════════════════
# Nav2 stack
# ══════════════════════════════════════════════════════════════════════════════

def _nav2_group(name, nav2_yaml, map_yaml, default_bt_xml, spawn_x, spawn_y):
    """Return a GroupAction containing the full Nav2 stack for one robot."""
    return GroupAction(actions=[
        PushRosNamespace(name),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
            output='screen',
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_yaml, {
                'tf_broadcast':     True,
                'odom_frame_id':    f'{name}/odom',
                'base_frame_id':    f'{name}/base_footprint',
                'global_frame_id':  'map',
                'set_initial_pose': True,
                # Pass nested dictionary to override nested YAML parameter
                'initial_pose': {
                    'x':   float(spawn_x),
                    'y':   float(spawn_y),
                    'z':   0.0,
                    'yaw': 0.0,
                },
                'initial_pose.covariance_x':   0.25,
                'initial_pose.covariance_y':   0.25,
                'initial_pose.covariance_yaw': 0.1,
            }],
            remappings=[('scan', 'scan')],
            output='screen',
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_yaml],
            output='screen',
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_yaml, {'transform_tolerance': 1.0}],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('scan',    'scan'),
                ('odom',    'odom_filtered'),
            ],
            output='screen',
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_yaml],
            output='screen',
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_yaml, {
                'global_frame':            'map',
                'robot_base_frame':        f'{name}/base_link',
                'odom_topic':              'odom_filtered', # Changed to EKF topic
                'default_bt_xml_filename': default_bt_xml,
                'navigators': ['navigate_to_pose', 'navigate_through_poses'],
                'navigate_to_pose':
                    {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'},
                'navigate_through_poses':
                    {'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'},
            }],
            # Remap default odom topic to the filtered odom topic
            remappings=[('odom', 'odom_filtered')],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'bond_timeout': 30.0,
                'node_names': [
                    'map_server', 'amcl', 'planner_server',
                    'controller_server', 'behavior_server', 'bt_navigator',
                ],
            }],
            output='screen',
        ),
    ])


# ══════════════════════════════════════════════════════════════════════════════
# Per-robot mission nodes
# ══════════════════════════════════════════════════════════════════════════════

def _mission_nodes(name):
    """
    The three mission nodes for one robot.

    waypoint_sender has an internal AMCL-freshness gate (_try_start); it will
    not issue any navigation goal until localisation is stable, so it is safe
    to start as soon as lifecycle nodes are active.
    """
    return [
        Node(
            package='warehouse_multi_robot',
            executable='waypoint_sender',
            name='waypoint_sender',
            namespace=name,
            parameters=[{
                'robot_name':            name,
                'use_sim_time':          True,
                'max_amcl_age_sec':      5.0,
                'max_amcl_pos_sigma':    0.5, 
                'require_amcl_age_gate': False,
                'goal_reject_retry_sec': 1.0,
            }],
            output='screen',
        ),
        Node(
            package='warehouse_multi_robot',
            executable='battery_monitor',
            name=f'battery_monitor_{name}',
            namespace=name,
            parameters=[{
                'robot_name': name,
                'drain_rate': 0.01,
                'fail_at':    0.0,
                'start_at':   100.0,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ]


# ══════════════════════════════════════════════════════════════════════════════
# Per-robot action chain builder
# ══════════════════════════════════════════════════════════════════════════════

def _robot_chain(robot, world_name, base_urdf, bridge_yaml, nav2_yaml,
                 map_yaml, default_bt_xml, start_trigger_gate):
    """
    Build and return all launch actions for one robot.

    The chain is wired via OnProcessExit so each step starts only after the
    previous one has confirmed success.

    Chain:
        clock_gate exits
          └─► spawn
                └─► bridge + rsp + ekf_node + gate:topics
                      └─► nav2 stack + gate:lifecycle
                            └─► mission nodes
    """
    name = robot['name']
    x, y = robot['x'], robot['y']

    # ── spawn ─────────────────────────────────────────────────────────────────
    spawn = ExecuteProcess(
        cmd=[
            sys.executable, _SPAWN_PY,
            '--world-name', world_name,
            '--robot-name', name,
            '--sdf-file',   robot['sdf'],
            '--x', x, '--y', y, '--z', '0.01',
        ],
        name=f'spawn_{name}',
        output='screen',
    )

    # ── post-spawn nodes ──────────────────────────────────────────────────────
    robot_desc = _patch_urdf(base_urdf, name)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{name}',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
        output='screen',
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time':      True,
            'publish_frequency': 50.0,
        }],
        # publish directly to global TF trees — no relay needed for static
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        output='screen',
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=name,
        parameters=[nav2_yaml], 
        # Added TF remappings to prevent EKF transforms from being trapped in namespace /robotX/tf
        remappings=[
            ('odometry/filtered', 'odom_filtered'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        output='screen'
    )

    # gate: wait until scan, odom_fixed, and TF topics all have publishers
    topics_gate = _gate(
        'topics',
        topics=[
            f'/{name}/scan', 
            f'/{name}/odom_filtered', 
            f'/{name}/joint_states',
            '/tf', 
            '/tf_static'
        ],
        label=f'gate_topics_{name}',
    )

    # gate: wait until all Nav2 lifecycle nodes for this robot are active
    lifecycle_gate = _gate(
        'lifecycle',
        nodes=[
            f'/{name}/map_server',
            f'/{name}/amcl',
            f'/{name}/planner_server',
            f'/{name}/controller_server',
            f'/{name}/behavior_server',
            f'/{name}/bt_navigator',
        ],
        label=f'gate_lifecycle_{name}',
    )

    # ── wire the chain ────────────────────────────────────────────────────────
    # start_trigger_gate ready → spawn this robot
    spawn_trigger = RegisterEventHandler(OnProcessExit(
        target_action=start_trigger_gate,
        on_exit=[spawn],
    ))

    actions_list = [
        spawn_trigger,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[bridge, rsp, ekf_node, topics_gate],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=topics_gate,
            on_exit=[
                _nav2_group(name, nav2_yaml, map_yaml, default_bt_xml, x, y),
                lifecycle_gate,
            ],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=lifecycle_gate,
            on_exit=_mission_nodes(name),
        )),
    ]

    if name == 'robot3':
        rviz_config_path = '/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/rviz_multi_robot_nav2.rviz'
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            # Force RViz2 to use CPU software OpenGL rasterizer to bypass Mesa D3D12 NVIDIA shader compilation bug
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            output='screen',
        )
        # Trigger rviz launch only when robot 3 is fully initialized
        rviz_trigger = RegisterEventHandler(OnProcessExit(
            target_action=lifecycle_gate,
            on_exit=[rviz_node],
        ))
        actions_list.append(rviz_trigger)
    # Return the action list AND the lifecycle gate to chain the next robot
    return actions_list, lifecycle_gate


# ══════════════════════════════════════════════════════════════════════════════
# generate_launch_description
# ══════════════════════════════════════════════════════════════════════════════

def generate_launch_description():

    # ── package paths ─────────────────────────────────────────────────────────
    ros_gz_sim  = get_package_share_directory('ros_gz_sim')
    tb3_desc    = get_package_share_directory('turtlebot3_description')
    nav2_bt_dir = get_package_share_directory('nav2_bt_navigator')

    # ── config paths ──────────────────────────────────────────────────────────
    WMR = '/home/canozkan/thesis_ws/src/warehouse_multi_robot'

    bridge_yamls = {
        'robot1': f'{WMR}/config/bridge_robot1.yaml',
        'robot2': f'{WMR}/config/bridge_robot2.yaml',
        'robot3': f'{WMR}/config/bridge_robot3.yaml',
    }
    nav2_yamls = {
        'robot1': f'{WMR}/config/nav2_params_robot1.yaml',
        'robot2': f'{WMR}/config/nav2_params_robot2.yaml',
        'robot3': f'{WMR}/config/nav2_params_robot3.yaml',
    }
    map_yaml     = f'{WMR}/maps/warehouse_map.yaml'
    world_path   = f'{WMR}/worlds/tugbot_warehouse_clean.sdf'
    clock_yaml   = f'{WMR}/config/bridge_clock.yaml'
    fuel_path    = ('/home/canozkan/.gz/fuel/fuel.gazebosim.org/'
                   'openrobotics/worlds/tugbot in warehouse/2/')
    default_bt_xml = os.path.join(
        nav2_bt_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )
    world_name = os.path.splitext(os.path.basename(world_path))[0]

    # ── URDF (parsed once, patched per robot at chain-build time) ─────────────
    xacro_file = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    base_urdf  = subprocess.check_output(
        ['xacro', xacro_file], stderr=subprocess.DEVNULL
    ).decode('utf-8')

    # ── robot definitions ─────────────────────────────────────────────────────
    robots = [
        {'name': 'robot1', 'x': '0.0', 'y':  '1.0',
         'sdf': f'{WMR}/models/turtlebot3_waffle/model_robot1.sdf'},
        {'name': 'robot2', 'x': '0.0', 'y':  '0.0',
         'sdf': f'{WMR}/models/turtlebot3_waffle/model_robot2.sdf'},
        {'name': 'robot3', 'x': '0.0', 'y': '-1.0',
         'sdf': f'{WMR}/models/turtlebot3_waffle/model_robot3.sdf'},
    ]

    # ── OpaqueFunction: builds the dynamic part of the graph ──────────────────
    def _build_chains(context, *args, **kwargs):
        """
        Called once by the launch framework after static actions are set up.
        Returns all the event-handler chains for world gate → clock → robots.
        """
        world_gate = _gate('world', world_name=world_name, label='gate_world')
        clock_gate = _gate('clock', label='gate_clock')

        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['--ros-args', '-p', f'config_file:={clock_yaml}'],
            output='screen',
        )

        # Self Note: auto_start = True is only for testing purposes.
        # In / after sprint 3 this will be triggered via GUI 
        mission_gate = Node(
            package='warehouse_multi_robot',
            executable='mission_gate',
            name='mission_gate',
            parameters=[{'auto_start': False}],
            output='screen',
        )

        actions = [
            world_gate,
            RegisterEventHandler(OnProcessExit(
                target_action=world_gate,
                on_exit=[clock_bridge, clock_gate, mission_gate],
            )),
        ]

        # Daisy-chain the robot launching sequences sequentially to prevent CPU starvation on WSL2
        # robot1 triggers immediately when clock_gate is ready
        robot1_actions, robot1_lifecycle_gate = _robot_chain(
            robots[0], world_name, base_urdf,
            bridge_yamls[robots[0]['name']],
            nav2_yamls[robots[0]['name']],
            map_yaml, default_bt_xml,
            clock_gate,
        )
        actions.extend(robot1_actions)

        # robot2 triggers only when robot1's lifecycle_gate finishes successfully (ACTIVE)
        robot2_actions, robot2_lifecycle_gate = _robot_chain(
            robots[1], world_name, base_urdf,
            bridge_yamls[robots[1]['name']],
            nav2_yamls[robots[1]['name']],
            map_yaml, default_bt_xml,
            robot1_lifecycle_gate,
        )
        actions.extend(robot2_actions)

        # robot3 triggers only when robot2's lifecycle_gate finishes successfully (ACTIVE)
        robot3_actions, robot3_lifecycle_gate = _robot_chain(
            robots[2], world_name, base_urdf,
            bridge_yamls[robots[2]['name']],
            nav2_yamls[robots[2]['name']],
            map_yaml, default_bt_xml,
            robot2_lifecycle_gate,
        )
        actions.extend(robot3_actions)

        return actions

    # ── static actions ────────────────────────────────────────────────────────
    return LaunchDescription([
        SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', fuel_path),

        # Gazebo sim server — on_exit_shutdown=true: if sim dies, kill everything
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args':          ['-r -s -v2 ', world_path],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # Gazebo GUI client — on_exit_shutdown=false: closing GUI is non-fatal
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args':          '-g -v2',
                'on_exit_shutdown': 'false',
            }.items(),
        ),

        OpaqueFunction(function=_build_chains),
    ])