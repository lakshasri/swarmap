#!/usr/bin/env python3
"""Dashboard bridge: translates dashboard command topics into ROS2 actions.

Listens to plain std_msgs/String messages from the React dashboard:

  /swarm/spawn_robot_request    data = ""          spawn next available robot_id
  /swarm/kill_robot_request     data = "robot_3"   kill specific robot
  /swarm/pause_request          data = ""          pause world_sim
  /swarm/resume_request         data = ""          resume world_sim
  /swarm/stop_request           data = ""          pause + kill all robots
  /swarm/reset_map_request      data = ""          clear all occupancy grids

Spawn launches a new robot_node OS process and notifies world_sim_node
to wire up the corresponding scan/odom/cmd plumbing. Kill triggers a
lifecycle SHUTDOWN and pkills the process.
"""
import os
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

MAX_ROBOTS = 32


class DashboardBridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_bridge_node')
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('params_file', '')
        self.n_initial = int(self.get_parameter('num_robots').value)
        self.params_file = str(self.get_parameter('params_file').value)

        self.alive: set[str] = {f'robot_{i}' for i in range(self.n_initial)}
        self.killed: set[str] = set()
        self.spawned_procs: dict[str, subprocess.Popen] = {}

        self._spawn_pub = self.create_publisher(String, '/swarm/spawn_robot', 10)
        self._kill_pub  = self.create_publisher(String, '/swarm/kill_robot', 10)

        self.create_subscription(String, '/swarm/spawn_robot_request',
                                 self._on_spawn_request, 10)
        self.create_subscription(String, '/swarm/kill_robot_request',
                                 self._on_kill_request, 10)
        self.create_subscription(String, '/swarm/pause_request',
                                 self._on_pause, 10)
        self.create_subscription(String, '/swarm/resume_request',
                                 self._on_resume, 10)
        self.create_subscription(String, '/swarm/stop_request',
                                 self._on_stop, 10)
        self.create_subscription(String, '/swarm/reset_map_request',
                                 self._on_reset_map, 10)
        self._reset_map_pub = self.create_publisher(String, '/swarm/reset_map', 10)

        self._pause_pub  = self.create_publisher(String, '/swarm/pause', 10)
        self._resume_pub = self.create_publisher(String, '/swarm/resume', 10)

        self.get_logger().info(
            f'DashboardBridge ready (initial robots: {self.n_initial})')

    def _next_robot_id(self) -> str | None:
        # FIX #17: recycle killed IDs so we don't exhaust the pool
        for i in range(MAX_ROBOTS):
            rid = f'robot_{i}'
            if rid not in self.alive:
                return rid
        return None

    def _on_spawn_request(self, _msg: String):
        rid = self._next_robot_id()
        if rid is None:
            self.get_logger().warning('Spawn limit reached')
            return

        notify = String()
        notify.data = rid
        self._spawn_pub.publish(notify)

        cmd = [
            'ros2', 'run', 'swarmap_core', 'robot_node',
            '--ros-args',
            '-r', f'__ns:=/{rid}',
            '-r', '__node:=robot_node',
            '-p', f'robot_id:={rid}',
        ]
        if self.params_file and os.path.isfile(self.params_file):
            cmd += ['--params-file', self.params_file]

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except FileNotFoundError as e:
            self.get_logger().error(f'Failed to spawn {rid}: {e}')
            return

        self.spawned_procs[rid] = proc
        self.alive.add(rid)
        self.get_logger().info(f'Spawned {rid} (pid {proc.pid})')

    def _on_kill_request(self, msg: String):
        rid = msg.data.strip()
        if not rid or rid in self.killed:
            return
        self.killed.add(rid)
        self.alive.discard(rid)

        notify = String()
        notify.data = rid
        self._kill_pub.publish(notify)

        service_name = f'/{rid}/robot_node/change_state'
        client = self.create_client(ChangeState, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVE_SHUTDOWN
            client.call_async(req)
        else:
            self.get_logger().warning(
                f'Lifecycle service not ready for {rid}')

        proc = self.spawned_procs.pop(rid, None)
        if proc is not None:
            try:
                proc.terminate()
            except ProcessLookupError:
                pass
        else:
            try:
                subprocess.run(
                    ['pkill', '-TERM', '-f', f'__ns:=/{rid} '],
                    check=False,
                )
            except FileNotFoundError:
                pass

        self.get_logger().warning(f'Killed {rid}')

    def _on_pause(self, _msg: String):
        self._pause_pub.publish(String())
        self.get_logger().info('Simulation PAUSED')

    def _on_resume(self, _msg: String):
        self._resume_pub.publish(String())
        self.get_logger().info('Simulation RESUMED')

    def _on_stop(self, _msg: String):
        self._pause_pub.publish(String())
        for rid in sorted(self.alive.copy()):
            m = String()
            m.data = rid
            self._on_kill_request(m)
        self.get_logger().warning('Simulation STOPPED — all robots killed')

    def _on_reset_map(self, _msg: String):
        self._reset_map_pub.publish(String())
        self.get_logger().info('Map reset broadcast')


def main():
    rclpy.init()
    node = DashboardBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for proc in node.spawned_procs.values():
            try:
                proc.terminate()
            except ProcessLookupError:
                pass
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
