import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    model_folder = 'turtlebot3_waffle'
    urdf_path = os.path.join(tb3_gazebo, 'models', model_folder, 'model.sdf')
    bridge_params = os.path.join(tb3_gazebo, 'params', model_folder + '_bridge.yaml')
    
    world_path = os.path.expanduser(
        '~/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf'
    )

    fuel_resource = os.path.expanduser(
        '~/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/tugbot in warehouse/2/'
    )

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=fuel_resource
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robots = [
        {'name': 'robot1', 'x': '0.0',  'y':  '0.0'},
        {'name': 'robot2', 'x': '0.0',  'y':  '2.0'},
        {'name': 'robot3', 'x': '0.0',  'y': '-2.0'},
    ]

    spawn_actions = []
    for i, robot in enumerate(robots):
        spawn = TimerAction(
            period=float(i * 3),
            actions=[
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
            ]
        )

        bridge = TimerAction(
            period=float(i * 3 + 1),
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    namespace=robot['name'],
                    arguments=[
                        '--ros-args', '-p',
                        f'config_file:={bridge_params}',
                    ],
                    output='screen'
                )
            ]
        )
        spawn_actions.extend([spawn, bridge])

    ld = LaunchDescription()
    ld.add_action(set_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    for action in spawn_actions:
        ld.add_action(action)
    return ld
