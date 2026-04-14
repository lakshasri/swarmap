"""Dashboard launch — rosbridge WebSocket + Vite dev server.

Brings up:
  - rosbridge_server on ws://localhost:9090 with the YAML config
  - Vite dev server (npm run dev) on http://localhost:5173

Assumes simulation.launch.py is running in a separate terminal so the
map_aggregator + robot nodes are alive. Or just pass dashboard:=true
to simulation.launch.py to chain both.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_dashboard_src():
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        candidate = os.path.join(here, 'src', 'swarmap_dashboard')
        if os.path.isdir(candidate):
            return candidate
        here = os.path.dirname(here)
    return None


def generate_launch_description():
    bringup_share = get_package_share_directory('swarmap_bringup')
    rosbridge_params = os.path.join(bringup_share, 'config', 'rosbridge_params.yaml')
    dashboard_src = (os.environ.get('SWARMAP_DASHBOARD_DIR')
                     or _find_dashboard_src()
                     or os.path.join(os.getcwd(), 'src', 'swarmap_dashboard'))

    return LaunchDescription([
        DeclareLaunchArgument('dashboard_port', default_value='5173'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[rosbridge_params],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['npm', 'run', 'dev', '--',
                 '--host', '0.0.0.0',
                 '--port', LaunchConfiguration('dashboard_port')],
            cwd=dashboard_src,
            output='screen',
        ),
    ])
