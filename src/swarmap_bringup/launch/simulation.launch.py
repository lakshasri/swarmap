"""Swarmap simulation launch — pure ROS2, optional RViz + dashboard.

Brings up:
  - world_sim_node: ground-truth grid, ray-cast LiDAR, odom + cmd_vel feedback
  - N robot_node lifecycle nodes (each in its own /robot_i namespace)
  - map_aggregator + swarm_monitor + dashboard_bridge
  - RViz2 with swarm_debug.rviz config (rviz:=true)
  - Optional browser dashboard (dashboard:=true) — chains dashboard.launch.py
    which starts rosbridge_server (port 9090) + Vite dev server (port 5173)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_world_and_swarm(context, *_args, **_kwargs):
    bringup_share = get_package_share_directory('swarmap_bringup')
    params_path = os.path.join(bringup_share, 'config', 'default_params.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'swarm_debug.rviz')

    n = int(LaunchConfiguration('num_robots').perform(context))
    sensor_range = float(LaunchConfiguration('sensor_range').perform(context))
    comm_radius = float(LaunchConfiguration('comm_radius').perform(context))
    map_res = float(LaunchConfiguration('map_resolution').perform(context))
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes')
    use_dashboard = LaunchConfiguration('dashboard').perform(context).lower() in ('1', 'true', 'yes')

    common = {
        'num_robots': n,
        'sensor_range': sensor_range,
        'comm_radius': comm_radius,
        'map_resolution': map_res,
    }

    actions = [
        Node(
            package='swarmap_core',
            executable='world_sim_node.py',
            name='world_sim_node',
            parameters=[params_path, common],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='map_aggregator_node',
            name='map_aggregator_node',
            parameters=[params_path, common],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='swarm_monitor_node.py',
            name='swarm_monitor_node',
            parameters=[params_path, common],
            output='screen',
        ),
        Node(
            package='swarmap_core',
            executable='dashboard_bridge_node.py',
            name='dashboard_bridge_node',
            parameters=[params_path, common, {'params_file': params_path}],
            output='screen',
        ),
    ]

    for i in range(n):
        actions.append(Node(
            package='swarmap_core',
            executable='robot_node',
            namespace=f'robot_{i}',
            name='robot_node',
            parameters=[
                params_path,
                {**common, 'robot_id': f'robot_{i}'},
            ],
            output='log',
        ))

    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
        ))

    if use_dashboard:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'dashboard.launch.py'))
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='5'),
        DeclareLaunchArgument('sensor_range', default_value='5.0'),
        DeclareLaunchArgument('comm_radius', default_value='8.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='false'),

        OpaqueFunction(function=launch_world_and_swarm),
    ])
