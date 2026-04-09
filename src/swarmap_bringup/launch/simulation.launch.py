import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_robot_spawn_actions(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    sensor_range = LaunchConfiguration('sensor_range').perform(context)
    comm_radius = LaunchConfiguration('comm_radius').perform(context)
    noise_level = LaunchConfiguration('noise_level').perform(context)
    failure_rate = LaunchConfiguration('failure_rate').perform(context)
    map_res = LaunchConfiguration('map_resolution').perform(context)

    bringup_share = get_package_share_directory('swarmap_bringup')
    model_path = os.path.join(bringup_share, 'models', 'diff_drive_robot', 'model.sdf')
    params_path = os.path.join(bringup_share, 'config', 'default_params.yaml')

    actions = []

    cols = max(1, math.ceil(math.sqrt(num_robots)))
    spacing = 2.5

    for i in range(num_robots):
        robot_id = f'robot_{i}'
        col = i % cols
        row = i // cols
        x = (col - cols / 2.0) * spacing
        y = (row - cols / 2.0) * spacing

        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_id}',
            arguments=[
                '-name', robot_id,
                '-file', model_path,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.1',
                '-Y', '0.0',
            ],
            output='screen',
        )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_id,
            name='gz_bridge',
            arguments=[
                f'/model/{robot_id}/odometry'
                f'@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

                f'/model/{robot_id}/scan'
                f'@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

                f'/model/{robot_id}/cmd_vel'
                f'@geometry_msgs/msg/Twist]ignition.msgs.Twist',

                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            remappings=[
                (f'/model/{robot_id}/odometry', f'/{robot_id}/odom'),
                (f'/model/{robot_id}/scan', f'/{robot_id}/scan'),
                (f'/model/{robot_id}/cmd_vel', f'/{robot_id}/cmd_vel'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        robot_node = Node(
            package='swarmap_core',
            executable='robot_node',
            name='robot_node',
            namespace=robot_id,
            parameters=[
                params_path,
                {
                    'robot_id': robot_id,
                    'sensor_range': float(sensor_range),
                    'comm_radius': float(comm_radius),
                    'noise_level': float(noise_level),
                    'failure_rate': float(failure_rate),
                    'map_resolution': float(map_res),
                    'use_sim_time': True,
                }
            ],
            output='screen',
        )

        delay = float(i) * 0.5 + 1.0
        actions.append(spawn)
        actions.append(TimerAction(period=delay, actions=[bridge, robot_node]))

    return actions


def launch_gazebo(context, *args, **kwargs):
    bringup_share = get_package_share_directory('swarmap_bringup')
    world_name = LaunchConfiguration('world').perform(context)
    world_file = os.path.join(bringup_share, 'worlds', world_name + '.sdf')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
                'gz_version': '6',
                'on_exit_shutdown': 'true',
            }.items(),
        )
    ]


def launch_swarm_nodes(context, *args, **kwargs):
    bringup_share = get_package_share_directory('swarmap_bringup')
    params_path = os.path.join(bringup_share, 'config', 'default_params.yaml')
    num_robots = LaunchConfiguration('num_robots').perform(context)
    failure_rate = LaunchConfiguration('failure_rate').perform(context)
    map_res = LaunchConfiguration('map_resolution').perform(context)

    return [
        Node(
            package='swarmap_core',
            executable='map_aggregator_node',
            name='map_aggregator_node',
            parameters=[
                params_path,
                {'use_sim_time': True, 'num_robots': int(num_robots)},
            ],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='swarm_monitor_node.py',
            name='swarm_monitor_node',
            parameters=[
                params_path,
                {'use_sim_time': True, 'num_robots': int(num_robots)},
            ],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='param_service_node.py',
            name='param_service_node',
            parameters=[
                params_path,
                {'use_sim_time': True, 'num_robots': int(num_robots)},
            ],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='failure_injector_node',
            name='failure_injector_node',
            parameters=[
                params_path,
                {
                    'use_sim_time': True,
                    'num_robots': int(num_robots),
                    'failure_rate': float(failure_rate),
                },
            ],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='robot_spawner_node.py',
            name='robot_spawner_node',
            parameters=[
                params_path,
                {
                    'use_sim_time': True,
                    'num_robots': int(num_robots),
                    'sensor_range': float(LaunchConfiguration('sensor_range').perform(context)),
                    'comm_radius': float(LaunchConfiguration('comm_radius').perform(context)),
                    'noise_level': float(LaunchConfiguration('noise_level').perform(context)),
                    'failure_rate': float(failure_rate),
                    'map_resolution': float(LaunchConfiguration('map_resolution').perform(context)),
                },
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='5'),
        DeclareLaunchArgument('sensor_range', default_value='5.0'),
        DeclareLaunchArgument('comm_radius', default_value='8.0'),
        DeclareLaunchArgument('noise_level', default_value='0.0'),
        DeclareLaunchArgument('failure_rate', default_value='0.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('world', default_value='warehouse'),

        OpaqueFunction(function=launch_gazebo),

        OpaqueFunction(function=generate_robot_spawn_actions),

        OpaqueFunction(function=launch_swarm_nodes),
    ])
