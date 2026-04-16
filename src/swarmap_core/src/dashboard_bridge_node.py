#!/usr/bin/env python3
"""Dashboard bridge: translates dashboard command topics into ROS2 actions.

Listens to plain std_msgs/String messages from the React dashboard:

  /swarm/spawn_robot          data = ""           (spawn next available robot_id)
  /swarm/kill_robot           data = "robot_3"
  /swarm/set_param_request    data = '{"param":"failure_rate","value":0.2}'
  /swarm/inject_failure_now   data = ""

Spawn launches a new robot_node OS process and notifies world_sim_node
to wire up the corresponding scan/odom/cmd plumbing. Kill triggers a
lifecycle SHUTDOWN. set_param_request fans SetParameters out to the
nodes that own each parameter.
"""
import json
import os
import random
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


PARAM_ROUTES = {
    'failure_rate':  ['/failure_injector_node'],
    'failure_mode':  ['/failure_injector_node'],
    'noise_level':   ['/world_sim_node'],
    'sensor_range':  ['ROBOTS'],
    'comm_radius':   ['ROBOTS'],
}

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
        self.create_subscription(String, '/swarm/set_param_request',
                                 self._on_set_param, 10)
        self.create_subscription(String, '/swarm/inject_failure_now',
                                 self._on_inject_failure, 10)
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

    def _on_set_param(self, msg: String):
        try:
            payload = json.loads(msg.data)
            param = str(payload['param'])
            value = payload['value']
        except (json.JSONDecodeError, KeyError, TypeError) as e:
            self.get_logger().warning(f'Bad set_param_request: {e}')
            return

        targets = PARAM_ROUTES.get(param)
        if not targets:
            self.get_logger().warning(f'Unknown parameter {param!r}')
            return

        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, (int, float)):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        else:
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)

        # FIX #6: copy self.alive to avoid race with _on_kill_request
        nodes: list[str] = []
        alive_snapshot = set(self.alive)
        for t in targets:
            if t == 'ROBOTS':
                nodes.extend(f'/{rid}/robot_node' for rid in alive_snapshot)
            else:
                nodes.append(t)

        for node_name in nodes:
            self._call_set_param(node_name, param, pv)

    def _call_set_param(self, node_name: str, param: str, pv: ParameterValue):
        service_name = f'{node_name}/set_parameters'
        client = self.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = param
        p.value = pv
        req.parameters = [p]
        client.call_async(req)

    def _on_inject_failure(self, _msg: String):
        if not self.alive:
            return
        target = random.choice(sorted(self.alive))
        m = String()
        m.data = target
        self._on_kill_request(m)

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
