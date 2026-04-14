"""Demo scenario 2 — Fault tolerance under fire.

20 robots in the large-office world with a 40% progressive failure rate,
5 minute mission.  Expected outcome: coverage >=85% with ~12 survivors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'simulation.launch.py')
    dashboard_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'dashboard.launch.py')
    results_dir = os.path.join(os.getcwd(), 'results', 'demo_fault_tolerance')
    os.makedirs(results_dir, exist_ok=True)

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='300.0'),
        DeclareLaunchArgument('seed', default_value='7'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_robots': '20',
                'sensor_range': '4.0',
                'comm_radius': '7.0',
                'failure_rate': '0.40',
                'noise_level': '0.05',
                'world': 'large_office',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dashboard_launch),
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 os.path.join(results_dir, 'run.bag'),
                 '/dashboard/map_compressed', '/dashboard/stats',
                 '/dashboard/events', '/swarm/health'],
            output='screen',
        ),

        TimerAction(
            period=float(os.environ.get('SWARMAP_DEMO_DURATION', 300.0)),
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once', '/swarm/mission',
                         'std_msgs/msg/String', 'data: stop'],
                    output='screen',
                ),
            ],
        ),
    ])
