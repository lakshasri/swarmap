import json
import math
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class RobotSpawnerNode(Node):

    def __init__(self):
        super().__init__('robot_spawner_node')
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('sensor_range', 5.0)
        self.declare_parameter('comm_radius', 8.0)
        self.declare_parameter('noise_level', 0.0)
        self.declare_parameter('failure_rate', 0.0)
        self.declare_parameter('map_resolution', 0.1)

        self._next_id = self.get_parameter('num_robots').value

        bringup_share = get_package_share_directory('swarmap_bringup')
        self._model_path = os.path.join(bringup_share, 'models', 'diff_drive_robot', 'model.sdf')
        self._params_path = os.path.join(bringup_share, 'config', 'default_params.yaml')

        self._pub = self.create_publisher(String, '/swarm/spawner_status', 10)

        self.create_subscription(String, '/swarm/spawn_robot', self._on_spawn, 10)
        self.create_subscription(String, '/swarm/kill_robot', self._on_kill, 10)

        self.get_logger().info(f'RobotSpawner ready, next id = robot_{self._next_id}')

    def _on_spawn(self, msg: String):
        robot_id = f'robot_{self._next_id}'
        self._next_id += 1

        try:
            req = json.loads(msg.data) if msg.data.strip() else {}
        except json.JSONDecodeError:
            req = {}

        x = float(req.get('x', (self._next_id % 5) * 2.5 - 5.0))
        y = float(req.get('y', (self._next_id // 5) * 2.5 - 5.0))

        self.get_logger().info(f'Spawning {robot_id} at ({x:.1f}, {y:.1f})')

        subprocess.Popen([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', robot_id,
            '-file', self._model_path,
            '-x', str(x), '-y', str(y), '-z', '0.1', '-Y', '0.0',
        ])

        bridge_args = [
            f'/model/{robot_id}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/model/{robot_id}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/model/{robot_id}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ]
        remap_args = [
            '--remap', f'/model/{robot_id}/odometry:=/{robot_id}/odom',
            '--remap', f'/model/{robot_id}/scan:=/{robot_id}/scan',
            '--remap', f'/model/{robot_id}/cmd_vel:=/{robot_id}/cmd_vel',
        ]
        subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-r', f'__ns:=/{robot_id}',
             '-p', 'use_sim_time:=true'] + remap_args + bridge_args
        )

        sensor_range = self.get_parameter('sensor_range').value
        comm_radius = self.get_parameter('comm_radius').value
        noise_level = self.get_parameter('noise_level').value
        failure_rate = self.get_parameter('failure_rate').value
        map_res = self.get_parameter('map_resolution').value

        subprocess.Popen([
            'ros2', 'run', 'swarmap_core', 'robot_node',
            '--ros-args',
            '-r', f'__ns:=/{robot_id}',
            '-r', f'__node:=robot_node',
            '--params-file', self._params_path,
            '-p', f'robot_id:={robot_id}',
            '-p', f'sensor_range:={sensor_range}',
            '-p', f'comm_radius:={comm_radius}',
            '-p', f'noise_level:={noise_level}',
            '-p', f'failure_rate:={failure_rate}',
            '-p', f'map_resolution:={map_res}',
            '-p', 'use_sim_time:=true',
        ])

        status = String()
        status.data = json.dumps({'event': 'spawned', 'robot_id': robot_id, 'x': x, 'y': y})
        self._pub.publish(status)

    def _on_kill(self, msg: String):
        robot_id = msg.data.strip()
        if not robot_id:
            return

        self.get_logger().info(f'Killing {robot_id}')

        subprocess.Popen([
            'ros2', 'lifecycle', 'set',
            f'/{robot_id}/robot_node', 'shutdown',
        ])

        status = String()
        status.data = json.dumps({'event': 'killed', 'robot_id': robot_id})
        self._pub.publish(status)


def main():
    rclpy.init()
    node = RobotSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
