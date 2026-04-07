import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('swarmap_bringup')
    rosbridge_params = os.path.join(bringup_share, 'config', 'rosbridge_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('num_robots',    default_value='10'),
        DeclareLaunchArgument('dashboard_port',default_value='5173'),

        # ── rosbridge WebSocket server ────────────────────────────────────────
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[rosbridge_params, {'use_sim_time': True}],
            output='screen',
        ),

        # ── map_aggregator_node ───────────────────────────────────────────────
        Node(
            package='swarmap_core',
            executable='map_aggregator_node',
            name='map_aggregator_node',
            parameters=[
                os.path.join(bringup_share, 'config', 'default_params.yaml'),
                {'use_sim_time': True,
                 'num_robots':   LaunchConfiguration('num_robots')},
            ],
            output='screen',
        ),

        # ── Vite preview server (npm run preview) ─────────────────────────────
        ExecuteProcess(
            cmd=[
                'npm', 'run', 'preview', '--',
                '--host', '0.0.0.0',
                '--port', LaunchConfiguration('dashboard_port'),
            ],
            cwd=os.path.join(
                os.path.dirname(os.path.dirname(bringup_share)),
                '..', 'src', 'swarmap_dashboard'
            ),
            output='screen',
        ),
    ])
