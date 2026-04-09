import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    model_folder = 'turtlebot3_waffle'
    urdf_path = os.path.join(tb3_gazebo, 'models', model_folder, 'model.sdf')
    bridge_params = os.path.join(tb3_gazebo, 'params', model_folder + '_bridge.yaml')
    world = os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_world.world')

    # Gazebo server (fizik - tek instance)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Gazebo client (görsel - tek instance)
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
        {'name': 'robot1', 'x': '-2.0', 'y': '-0.5'},
        {'name': 'robot2', 'x':  '0.5', 'y': '-0.5'},
        {'name': 'robot3', 'x':  '0.5', 'y':  '0.5'},
    ]

    robot_actions = []
    for robot in robots:
        spawn = Node(
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

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot['name'],
            arguments=[
                '--ros-args', '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen'
        )

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot['name'],
            parameters=[{
                'use_sim_time': True,
                'robot_description': open(urdf_path).read()
                    if urdf_path.endswith('.urdf') else ''
            }],
            output='screen'
        )

        robot_actions.extend([spawn, bridge])

    ld = LaunchDescription()
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    for action in robot_actions:
        ld.add_action(action)
    return ld
