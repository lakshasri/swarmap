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
    """
    OpaqueFunction so we can read num_robots at launch time and
    generate one set of nodes per robot dynamically.
    """
    num_robots   = int(LaunchConfiguration('num_robots').perform(context))
    sensor_range = LaunchConfiguration('sensor_range').perform(context)
    comm_radius  = LaunchConfiguration('comm_radius').perform(context)
    noise_level  = LaunchConfiguration('noise_level').perform(context)
    failure_rate = LaunchConfiguration('failure_rate').perform(context)
    map_res      = LaunchConfiguration('map_resolution').perform(context)

    bringup_share = get_package_share_directory('swarmap_bringup')
    # SDF model file for Ignition Gazebo Fortress
    model_path    = os.path.join(bringup_share, 'models', 'diff_drive_robot', 'model.sdf')
    # URDF kept for robot_state_publisher only (TF tree, no Gazebo plugins needed)
    urdf_path     = os.path.join(bringup_share, 'urdf', 'diff_drive_robot.urdf.xacro')
    params_path   = os.path.join(bringup_share, 'config', 'default_params.yaml')

    actions = []

    # Spread robots in a grid pattern around the origin
    cols    = max(1, math.ceil(math.sqrt(num_robots)))
    spacing = 2.5   # metres between spawn points

    for i in range(num_robots):
        robot_id = f'robot_{i}'
        col = i % cols
        row = i // cols
        x   = (col - cols / 2.0) * spacing
        y   = (row - cols / 2.0) * spacing

        # ── Spawn robot SDF model into Ignition Gazebo ───────────────────────
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_id}',
            arguments=[
                '-name',  robot_id,
                '-file',  model_path,
                '-x',     str(x),
                '-y',     str(y),
                '-z',     '0.1',
                '-Y',     '0.0',
            ],
            output='screen',
        )

        # ── ros_gz_bridge: Ignition topics ↔ ROS2 topics ────────────────────
        # Ignition Fortress uses ignition.msgs.* (not gz.msgs.*)
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
                (f'/model/{robot_id}/scan',     f'/{robot_id}/scan'),
                (f'/model/{robot_id}/cmd_vel',  f'/{robot_id}/cmd_vel'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # ── swarmap robot_node ───────────────────────────────────────────────
        robot_node = Node(
            package='swarmap_core',
            executable='robot_node',
            name='robot_node',
            namespace=robot_id,
            parameters=[
                params_path,
                {
                    'robot_id':       robot_id,
                    'sensor_range':   float(sensor_range),
                    'comm_radius':    float(comm_radius),
                    'noise_level':    float(noise_level),
                    'failure_rate':   float(failure_rate),
                    'map_resolution': float(map_res),
                    'use_sim_time':   True,
                }
            ],
            output='screen',
        )

        # Stagger spawns so Gazebo isn't overwhelmed at t=0
        delay = float(i) * 0.5 + 1.0
        actions.append(spawn)
        actions.append(TimerAction(period=delay, actions=[bridge, robot_node]))

    return actions


def launch_gazebo(context, *args, **kwargs):
    bringup_share = get_package_share_directory('swarmap_bringup')
    world_name    = LaunchConfiguration('world').perform(context)
    world_file    = os.path.join(bringup_share, 'worlds', world_name + '.sdf')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args':          f'-r {world_file}',
                'gz_version':       '6',
                'on_exit_shutdown': 'true',
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('num_robots',    default_value='5'),
        DeclareLaunchArgument('sensor_range',  default_value='5.0'),
        DeclareLaunchArgument('comm_radius',   default_value='8.0'),
        DeclareLaunchArgument('noise_level',   default_value='0.0'),
        DeclareLaunchArgument('failure_rate',  default_value='0.0'),
        DeclareLaunchArgument('map_resolution',default_value='0.1'),
        DeclareLaunchArgument('world',         default_value='warehouse'),

        # ── Ignition Gazebo Fortress (version 6) ─────────────────────────────
        OpaqueFunction(function=launch_gazebo),

        # ── Dynamic robot spawning ────────────────────────────────────────────
        OpaqueFunction(function=generate_robot_spawn_actions),
    ])
