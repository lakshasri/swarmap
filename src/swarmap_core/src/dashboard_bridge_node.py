#!/usr/bin/env python3
"""Dashboard bridge: translates dashboard command topics into ROS2 actions.

The React dashboard publishes plain std_msgs/String messages on three topics:

  /swarm/kill_robot           data = "robot_3"
  /swarm/set_param_request    data = '{"param":"failure_rate","value":0.2}'
  /swarm/inject_failure_now   data = ""   (trigger a failure immediately)

This node listens to them and performs the corresponding ROS2 operation:
lifecycle shutdown, SetParameters service calls, and direct failure injection
via the existing failure_injector_node.
"""
import json
import random

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


class DashboardBridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_bridge_node')
        self.declare_parameter('num_robots', 10)
        self.n_robots = int(self.get_parameter('num_robots').value)
        self.killed: set[str] = set()

        self.create_subscription(String, '/swarm/kill_robot',
                                 self._on_kill, 10)
        self.create_subscription(String, '/swarm/set_param_request',
                                 self._on_set_param, 10)
        self.create_subscription(String, '/swarm/inject_failure_now',
                                 self._on_inject_failure, 10)

        self.get_logger().info(
            f'DashboardBridge ready (tracking {self.n_robots} robots)')

    def _on_kill(self, msg: String):
        rid = msg.data.strip()
        if not rid:
            return
        if rid in self.killed:
            self.get_logger().info(f'{rid} already killed, ignoring')
            return
        self.killed.add(rid)
        service_name = f'/{rid}/robot_node/change_state'
        client = self.create_client(ChangeState, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                f'Lifecycle service not ready for {rid}')
            return
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVE_SHUTDOWN
        client.call_async(req)
        self.get_logger().warning(f'Dashboard killed {rid}')

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
            self.get_logger().warning(f'Unknown parameter {param!r}, ignoring')
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

        nodes: list[str] = []
        for t in targets:
            if t == 'ROBOTS':
                nodes.extend(
                    f'/robot_{i}/robot_node' for i in range(self.n_robots)
                    if f'robot_{i}' not in self.killed
                )
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
        self.get_logger().info(
            f'Set {param} on {node_name}')

    def _on_inject_failure(self, _msg: String):
        active = [f'robot_{i}' for i in range(self.n_robots)
                  if f'robot_{i}' not in self.killed]
        if not active:
            self.get_logger().info('No active robots to fail')
            return
        target = random.choice(active)
        self._on_kill(String(data=target))


def main():
    rclpy.init()
    node = DashboardBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
