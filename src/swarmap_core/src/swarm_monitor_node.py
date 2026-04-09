import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarmap_msgs.msg import RobotStatus, NeighbourDiscovery


class SwarmMonitorNode(Node):
    STALE_TIMEOUT = 3.0

    def __init__(self):
        super().__init__('swarm_monitor_node')
        self.declare_parameter('num_robots', 10)
        self.declare_parameter('warn_below_fraction', 0.5)

        n = self.get_parameter('num_robots').value
        self._warn_frac = self.get_parameter('warn_below_fraction').value

        self._robots: dict[str, dict] = {}

        self._pub_health = self.create_publisher(String, '/swarm/health', 10)
        self._pub_topo = self.create_publisher(String, '/dashboard/network_topology', 10)

        for i in range(n):
            rid = f'robot_{i}'
            self._robots[rid] = {
                'last_seen': 0.0,
                'state': 'UNKNOWN',
                'battery': 1.0,
                'neighbours': [],
                'x': 0.0,
                'y': 0.0,
            }
            self.create_subscription(
                RobotStatus,
                f'/{rid}/status',
                self._make_status_cb(rid),
                10,
            )

        self.create_subscription(
            NeighbourDiscovery,
            '/swarm/discovery',
            self._on_discovery,
            10,
        )

        self.create_timer(1.0, self._publish_all)
        self.get_logger().info(f'SwarmMonitor tracking {n} robots')

    def _make_status_cb(self, robot_id: str):
        def cb(msg: RobotStatus):
            r = self._robots.setdefault(robot_id, {})
            r['last_seen'] = self.get_clock().now().nanoseconds * 1e-9
            r['state'] = msg.current_state
            r['battery'] = float(msg.battery_level)
            r['neighbours'] = list(msg.neighbour_ids)
            r['x'] = msg.pose.position.x
            r['y'] = msg.pose.position.y
        return cb

    def _on_discovery(self, msg: NeighbourDiscovery):
        r = self._robots.setdefault(msg.robot_id, {})
        r['x'] = msg.position.x
        r['y'] = msg.position.y
        r.setdefault('last_seen', 0.0)
        r.setdefault('state', 'UNKNOWN')
        r.setdefault('battery', 1.0)
        r.setdefault('neighbours', [])

    def _publish_all(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        active, stale = [], []

        for rid, r in self._robots.items():
            is_stale = (now - r.get('last_seen', 0.0)) > self.STALE_TIMEOUT
            if is_stale or r.get('state') == 'FAILED':
                stale.append(rid)
            else:
                active.append(rid)

        total = len(self._robots)
        active_frac = len(active) / max(total, 1)
        if active_frac < self._warn_frac:
            self.get_logger().warning(
                f'Active robots {len(active)}/{total} below {self._warn_frac * 100:.0f}% threshold'
            )

        health = {
            'timestamp': now,
            'active_robots': active,
            'stale_robots': stale,
            'active_count': len(active),
            'total_count': total,
        }
        msg = String()
        msg.data = json.dumps(health)
        self._pub_health.publish(msg)

        nodes = []
        edges = []
        seen_edges: set[tuple[str, str]] = set()

        for rid, r in self._robots.items():
            nodes.append({
                'id': rid,
                'x': r.get('x', 0.0),
                'y': r.get('y', 0.0),
                'failed': rid in stale,
            })
            for nb in r.get('neighbours', []):
                key = tuple(sorted([rid, nb]))
                if key not in seen_edges:
                    seen_edges.add(key)
                    edges.append({'source': rid, 'target': nb})

        topo = {'nodes': nodes, 'edges': edges}
        topo_msg = String()
        topo_msg.data = json.dumps(topo)
        self._pub_topo.publish(topo_msg)


def main():
    rclpy.init()
    node = SwarmMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
