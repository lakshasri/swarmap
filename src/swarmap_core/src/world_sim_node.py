#!/usr/bin/env python3
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

FREE = 0
OCC = 100


def build_world(width_cells: int, height_cells: int, seed: int = 7) -> np.ndarray:
    rng = random.Random(seed)
    grid = np.zeros((height_cells, width_cells), dtype=np.int8)
    grid[0, :] = OCC
    grid[-1, :] = OCC
    grid[:, 0] = OCC
    grid[:, -1] = OCC

    for _ in range(18):
        w = rng.randint(15, 45)
        h = rng.randint(2, 5)
        x = rng.randint(20, width_cells - w - 20)
        y = rng.randint(20, height_cells - h - 20)
        if rng.random() < 0.5:
            grid[y:y + h, x:x + w] = OCC
        else:
            grid[y:y + w, x:x + h] = OCC
    return grid


class WorldSim(Node):
    def __init__(self):
        super().__init__('world_sim_node')

        self.declare_parameter('num_robots', 5)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_width_m', 50.0)
        self.declare_parameter('map_height_m', 50.0)
        self.declare_parameter('sensor_range', 5.0)
        self.declare_parameter('noise_level', 0.0)
        self.declare_parameter('scan_rays', 180)
        self.declare_parameter('tick_hz', 10.0)
        self.declare_parameter('linear_speed_cap', 0.8)
        self.declare_parameter('angular_speed_cap', 2.0)

        self.n_initial = int(self.get_parameter('num_robots').value)
        self.res = float(self.get_parameter('map_resolution').value)
        self.width_m = float(self.get_parameter('map_width_m').value)
        self.height_m = float(self.get_parameter('map_height_m').value)
        self.range_max = float(self.get_parameter('sensor_range').value)
        self.noise = float(self.get_parameter('noise_level').value)
        self.rays = int(self.get_parameter('scan_rays').value)
        self.tick_hz = float(self.get_parameter('tick_hz').value)
        self.v_cap = float(self.get_parameter('linear_speed_cap').value)
        self.w_cap = float(self.get_parameter('angular_speed_cap').value)

        self.w_cells = int(self.width_m / self.res)
        self.h_cells = int(self.height_m / self.res)
        self.grid = build_world(self.w_cells, self.h_cells)

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.tf_bcast = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.robots: dict[str, dict] = {}
        self._static_tf_msgs: list[TransformStamped] = []

        spacing = self.width_m / (self.n_initial + 1)
        for i in range(self.n_initial):
            x = spacing * (i + 1)
            y = self.height_m * 0.5
            self._add_robot(f'robot_{i}', x, y)
        self._republish_static_tfs()

        self.create_subscription(String, '/swarm/spawn_robot',
                                 self._on_spawn, 10)
        self.create_subscription(String, '/swarm/kill_robot',
                                 self._on_kill, 10)
        # Init flag FIRST so subscription callbacks don't race
        self.paused = False
        self.create_subscription(String, '/swarm/pause',
                                 lambda _: self._set_paused(True), 10)
        self.create_subscription(String, '/swarm/resume',
                                 lambda _: self._set_paused(False), 10)

        self.gt_pub = self.create_publisher(OccupancyGrid, '/world/ground_truth', 1)
        self.gt_timer = self.create_timer(2.0, self._publish_ground_truth)

        self.dt = 1.0 / self.tick_hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'world_sim_node: {self.n_initial} robots in '
            f'{self.width_m:.1f}x{self.height_m:.1f} m world '
            f'({self.w_cells}x{self.h_cells} cells), {self.rays} rays')

    def _add_robot(self, rid: str, x: float, y: float):
        if rid in self.robots:
            return
        ns = f'/{rid}'
        r = {
            'pose': [x, y, 0.0],
            'cmd':  [0.0, 0.0],
            'scan_pub': self.create_publisher(LaserScan, f'{ns}/scan', self.qos),
            'odom_pub': self.create_publisher(Odometry,  f'{ns}/odom', self.qos),
        }
        r['cmd_sub'] = self.create_subscription(
            Twist, f'{ns}/cmd_vel', self._make_cmd_cb(rid), self.qos)
        self.robots[rid] = r

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{rid}/odom'
        t.transform.rotation.w = 1.0
        self._static_tf_msgs.append(t)

    def _republish_static_tfs(self):
        if self._static_tf_msgs:
            self.static_tf.sendTransform(self._static_tf_msgs)

    def _on_spawn(self, msg: String):
        rid = msg.data.strip()
        if not rid or rid in self.robots:
            return
        x, y = self._find_free_spawn()
        self._add_robot(rid, x, y)
        self._republish_static_tfs()
        self.get_logger().info(f'world: spawned {rid} at ({x:.1f}, {y:.1f})')

    def _find_free_spawn(self) -> tuple[float, float]:
        cx = self.width_m * 0.5
        cy = self.height_m * 0.5
        clearance = 0.5
        for r_step in range(0, 60):
            radius = r_step * clearance
            for k in range(max(1, r_step * 6)):
                ang = (k / max(1, r_step * 6)) * 2.0 * math.pi
                x = cx + radius * math.cos(ang)
                y = cy + radius * math.sin(ang)
                if x < 1.0 or x > self.width_m - 1.0:
                    continue
                if y < 1.0 or y > self.height_m - 1.0:
                    continue
                gx, gy = self._world_to_grid(x, y)
                if self._cell_occupied(gx, gy):
                    continue
                if self._too_close_to_other_robots(x, y, clearance):
                    continue
                return x, y
        # FIX #15: fallback scans wider to avoid spawning inside a wall
        for _ in range(200):
            x = random.uniform(2.0, self.width_m - 2.0)
            y = random.uniform(2.0, self.height_m - 2.0)
            gx, gy = self._world_to_grid(x, y)
            if not self._cell_occupied(gx, gy):
                return x, y
        self.get_logger().warning('spawn: could not find free cell, using center')
        return cx, cy

    def _too_close_to_other_robots(self, x: float, y: float, min_d: float) -> bool:
        for r in self.robots.values():
            ox, oy, _ = r['pose']
            if (ox - x) ** 2 + (oy - y) ** 2 < min_d * min_d:
                return True
        return False

    def _on_kill(self, msg: String):
        rid = msg.data.strip()
        r = self.robots.pop(rid, None)
        if r is None:
            return
        self.destroy_subscription(r['cmd_sub'])
        self.destroy_publisher(r['scan_pub'])
        self.destroy_publisher(r['odom_pub'])
        self.get_logger().info(f'world: removed {rid}')

    def _make_cmd_cb(self, rid: str):
        def cb(msg: Twist):
            r = self.robots.get(rid)
            if not r:
                return
            v = max(-self.v_cap, min(self.v_cap, msg.linear.x))
            w = max(-self.w_cap, min(self.w_cap, msg.angular.z))
            r['cmd'][0] = v
            r['cmd'][1] = w
        return cb

    def _publish_ground_truth(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.res
        msg.info.width = self.w_cells
        msg.info.height = self.h_cells
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().tolist()
        self.gt_pub.publish(msg)

    def _world_to_grid(self, x: float, y: float):
        # FIX #24: use floor instead of int() truncation
        return int(math.floor(x / self.res)), int(math.floor(y / self.res))

    def _cell_occupied(self, gx: int, gy: int) -> bool:
        if gx < 0 or gy < 0 or gx >= self.w_cells or gy >= self.h_cells:
            return True
        return self.grid[gy, gx] == OCC

    def _raycast(self, x: float, y: float, theta: float) -> float:
        step = self.res * 0.5
        max_steps = int(self.range_max / step)
        for s in range(1, max_steps + 1):
            d = s * step
            px = x + d * math.cos(theta)
            py = y + d * math.sin(theta)
            gx, gy = self._world_to_grid(px, py)
            if self._cell_occupied(gx, gy):
                return d
        return self.range_max

    def _set_paused(self, val: bool):
        self.paused = val
        self.get_logger().info(f'world: {"PAUSED" if val else "RESUMED"}')

    def tick(self):
        if self.paused:
            return
        now = self.get_clock().now().to_msg()
        for rid, r in list(self.robots.items()):
            v, w = r['cmd']
            x, y, th = r['pose']

            nx = x + v * math.cos(th) * self.dt
            ny = y + v * math.sin(th) * self.dt
            nth = th + w * self.dt

            ngx, ngy = self._world_to_grid(nx, ny)
            if not self._cell_occupied(ngx, ngy):
                x, y = nx, ny
            th = (nth + math.pi) % (2 * math.pi) - math.pi
            r['pose'] = [x, y, th]

            self._publish_odom(rid, r, x, y, th, v, w, now)
            self._publish_tf(rid, x, y, th, now)
            self._publish_scan(rid, r, x, y, th, now)

    def _publish_odom(self, rid, r, x, y, th, v, w, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{rid}/odom'
        msg.child_frame_id = f'{rid}/base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th * 0.5)
        msg.pose.pose.orientation.w = math.cos(th * 0.5)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        r['odom_pub'].publish(msg)

    def _publish_tf(self, rid, x, y, th, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = f'{rid}/odom'
        t.child_frame_id = f'{rid}/base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(th * 0.5)
        t.transform.rotation.w = math.cos(th * 0.5)
        self.tf_bcast.sendTransform(t)

    def _publish_scan(self, rid, r, x, y, th, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = f'{rid}/base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / self.rays
        scan.range_min = 0.05
        scan.range_max = self.range_max
        ranges = []
        for k in range(self.rays):
            a = th + scan.angle_min + k * scan.angle_increment
            d = self._raycast(x, y, a)
            if self.noise > 0.0:
                d += random.gauss(0.0, self.noise * self.range_max)
                d = max(scan.range_min, min(scan.range_max, d))
            ranges.append(float(d))
        scan.ranges = ranges
        r['scan_pub'].publish(scan)


def main():
    rclpy.init()
    node = WorldSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
