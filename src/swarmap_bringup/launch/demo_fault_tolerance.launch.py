"""Demo scenario 2 — fault tolerance under fire.

20 robots in the procedural world with a 40% progressive failure rate,
~5 minute mission. Expected outcome: coverage >= 85% with the surviving
robots covering for the dead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, Shutdown, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'simulation.launch.py')
    results_dir = os.path.join(os.getcwd(), 'results', 'demo_fault_tolerance')
    os.makedirs(results_dir, exist_ok=True)

    duration = float(os.environ.get('SWARMAP_DEMO_DURATION', 300.0))

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value=str(duration)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_robots': '20',
                'sensor_range': '4.0',
                'comm_radius': '7.0',
                'failure_rate': '0.40',
                'noise_level': '0.05',
            }.items(),
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 os.path.join(results_dir, 'run.bag'),
                 '/swarm/global_map', '/swarm/health',
                 '/swarm/events', '/swarm/frontier_markers'],
            output='log',
        ),

        TimerAction(
            period=duration,
            actions=[Shutdown(reason='demo duration elapsed')],
        ),
    ])
