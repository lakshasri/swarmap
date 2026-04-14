"""Demo scenario 1 — Basic swarm exploration.

10 robots in the warehouse world, no failures, 4 minute mission.
Expected outcome: >=90% coverage before the auto-shutdown timer fires.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'simulation.launch.py')
    dashboard_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'dashboard.launch.py')
    results_dir = os.path.join(os.getcwd(), 'results', 'demo_basic')
    os.makedirs(results_dir, exist_ok=True)

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='240.0'),
        DeclareLaunchArgument('seed', default_value='1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_robots': '10',
                'sensor_range': '5.0',
                'comm_radius': '8.0',
                'failure_rate': '0.0',
                'noise_level': '0.05',
                'world': 'warehouse',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dashboard_launch),
        ),

        # Record a rosbag for later replay / inspection.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 os.path.join(results_dir, 'run.bag'),
                 '/dashboard/map_compressed', '/dashboard/stats',
                 '/dashboard/events', '/swarm/health'],
            output='screen',
        ),

        # Auto-stop the mission after the configured duration.
        TimerAction(
            period=float(os.environ.get('SWARMAP_DEMO_DURATION', 240.0)),
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once', '/swarm/mission',
                         'std_msgs/msg/String', 'data: stop'],
                    output='screen',
                ),
            ],
        ),
    ])
