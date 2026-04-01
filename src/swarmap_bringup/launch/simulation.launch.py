import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
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
    urdf_path     = os.path.join(bringup_share, 'urdf', 'diff_drive_robot.urdf.xacro')
    params_path   = os.path.join(bringup_share, 'config', 'default_params.yaml')

    actions = []

    # Spread robots in a grid pattern around the origin
    cols = max(1, math.ceil(math.sqrt(num_robots)))
    spacing = 2.0   # metres between spawn points

    for i in range(num_robots):
        robot_id = f'robot_{i}'
        col = i % cols
        row = i // cols
        x = (col - cols / 2.0) * spacing
        y = (row - cols / 2.0) * spacing

        # ── Spawn robot model into Gazebo ────────────────────────────────────
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_id}',
            arguments=[
                '-name',  robot_id,
                '-file',  urdf_path,
                '-x',     str(x),
                '-y',     str(y),
                '-z',     '0.1',
                '-Y',     '0.0',
            ],
            output='screen',
        )

        # ── Robot state publisher (URDF → TF) ───────────────────────────────
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_id,
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )

        # ── ros_gz_bridge: Gazebo topics ↔ ROS2 topics ──────────────────────
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_id,
            name='gz_bridge',
            arguments=[
                f'/model/{robot_id}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/model/{robot_id}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/model/{robot_id}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
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
                    'robot_id':     robot_id,
                    'sensor_range': float(sensor_range),
                    'comm_radius':  float(comm_radius),
                    'noise_level':  float(noise_level),
                    'failure_rate': float(failure_rate),
                    'map_resolution': float(map_res),
                    'use_sim_time': True,
                }
            ],
            output='screen',
        )

        # Delay each robot by 0.5s so Gazebo doesn't get flooded at t=0
        actions.append(spawn)
        actions.append(TimerAction(period=float(i) * 0.5 + 1.0,
                                   actions=[rsp, bridge, robot_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('num_robots',    default_value='10',
            description='Number of robots to spawn'),
        DeclareLaunchArgument('sensor_range',  default_value='5.0',
            description='LiDAR sensor range in metres'),
        DeclareLaunchArgument('comm_radius',   default_value='8.0',
            description='Inter-robot communication radius in metres'),
        DeclareLaunchArgument('noise_level',   default_value='0.0',
            description='Gaussian noise fraction applied to sensor readings (0-1)'),
        DeclareLaunchArgument('failure_rate',  default_value='0.0',
            description='Robot failure probability per minute (0-0.45)'),
        DeclareLaunchArgument('map_resolution',default_value='0.1',
            description='Occupancy grid resolution in m/cell'),
        DeclareLaunchArgument('world',         default_value='warehouse',
            description='World name to load (no extension)'),

        # ── Gazebo simulation ─────────────────────────────────────────────────
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            name='gazebo',
            arguments=[
                PathJoinSubstitution([
                    FindPackageShare('swarmap_bringup'), 'worlds',
                    [LaunchConfiguration('world'), '.sdf']
                ]),
                '--verbose',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # ── Dynamic robot spawning ────────────────────────────────────────────
        OpaqueFunction(function=generate_robot_spawn_actions),
    ])
