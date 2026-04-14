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
        self.declare_parameter('linear_speed_cap', 0.6)
        self.declare_parameter('angular_speed_cap', 1.5)

        self.n = int(self.get_parameter('num_robots').value)
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

        self.poses = []
        self.cmds = []
        spacing = self.width_m / (self.n + 1)
        for i in range(self.n):
            x = spacing * (i + 1)
            y = self.height_m * 0.5
            self.poses.append([x, y, 0.0])
            self.cmds.append([0.0, 0.0])

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_pubs = []
        self.odom_pubs = []
        self.cmd_subs = []
        for i in range(self.n):
            ns = f'/robot_{i}'
            self.scan_pubs.append(self.create_publisher(LaserScan, f'{ns}/scan', qos))
            self.odom_pubs.append(self.create_publisher(Odometry, f'{ns}/odom', qos))
            self.cmd_subs.append(self.create_subscription(
                Twist, f'{ns}/cmd_vel', self._make_cmd_cb(i), qos))

        self.tf_bcast = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        self.gt_pub = self.create_publisher(OccupancyGrid, '/world/ground_truth', 1)
        self.gt_timer = self.create_timer(2.0, self._publish_ground_truth)

        self.dt = 1.0 / self.tick_hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'world_sim_node: {self.n} robots in '
            f'{self.width_m:.1f}x{self.height_m:.1f} m world '
            f'({self.w_cells}x{self.h_cells} cells), {self.rays} rays')

    def _make_cmd_cb(self, idx: int):
        def cb(msg: Twist):
            v = max(-self.v_cap, min(self.v_cap, msg.linear.x))
            w = max(-self.w_cap, min(self.w_cap, msg.angular.z))
            self.cmds[idx][0] = v
            self.cmds[idx][1] = w
        return cb

    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()
        msgs = []
        for i in range(self.n):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = f'robot_{i}/odom'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.rotation.w = 1.0
            msgs.append(t)
        self.static_tf.sendTransform(msgs)

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
        gx = int(x / self.res)
        gy = int(y / self.res)
        return gx, gy

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

    def tick(self):
        now = self.get_clock().now().to_msg()
        for i in range(self.n):
            v, w = self.cmds[i]
            x, y, th = self.poses[i]

            nx = x + v * math.cos(th) * self.dt
            ny = y + v * math.sin(th) * self.dt
            nth = th + w * self.dt

            ngx, ngy = self._world_to_grid(nx, ny)
            if not self._cell_occupied(ngx, ngy):
                x, y = nx, ny
            th = (nth + math.pi) % (2 * math.pi) - math.pi
            self.poses[i] = [x, y, th]

            self._publish_odom(i, x, y, th, v, w, now)
            self._publish_tf(i, x, y, th, now)
            self._publish_scan(i, x, y, th, now)

    def _publish_odom(self, i, x, y, th, v, w, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = f'robot_{i}/odom'
        msg.child_frame_id = f'robot_{i}/base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th * 0.5)
        msg.pose.pose.orientation.w = math.cos(th * 0.5)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.odom_pubs[i].publish(msg)

    def _publish_tf(self, i, x, y, th, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = f'robot_{i}/odom'
        t.child_frame_id = f'robot_{i}/base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(th * 0.5)
        t.transform.rotation.w = math.cos(th * 0.5)
        self.tf_bcast.sendTransform(t)

    def _publish_scan(self, i, x, y, th, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = f'robot_{i}/base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / self.rays
        scan.range_min = 0.05
        scan.range_max = self.range_max
        ranges = []
        for k in range(self.rays):
            a = th + scan.angle_min + k * scan.angle_increment
            r = self._raycast(x, y, a)
            if self.noise > 0.0:
                r += random.gauss(0.0, self.noise * self.range_max)
                r = max(scan.range_min, min(scan.range_max, r))
            ranges.append(float(r))
        scan.ranges = ranges
        self.scan_pubs[i].publish(scan)


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
