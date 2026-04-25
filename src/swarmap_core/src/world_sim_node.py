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
    """
    Clean rectangular obstacle world. Axis-aligned rectangles on a grid,
    with a perimeter wall, some interior rooms, pillars, and guaranteed
    free neighborhoods around docks + the central spawn band.
    """
    rng = random.Random(seed)
    grid = np.zeros((height_cells, width_cells), dtype=np.int8)

    # Perimeter wall (3 cells thick)
    grid[0:3, :] = OCC
    grid[-3:, :] = OCC
    grid[:, 0:3] = OCC
    grid[:, -3:] = OCC

    def place_rect(x, y, w, h):
        x = max(3, min(x, width_cells - 3 - w))
        y = max(3, min(y, height_cells - 3 - h))
        grid[y:y + h, x:x + w] = OCC

    def clear_rect(x, y, w, h):
        x0 = max(0, x); y0 = max(0, y)
        x1 = min(width_cells, x + w); y1 = min(height_cells, y + h)
        grid[y0:y1, x0:x1] = FREE

    # Dock cell coordinates (must match robot_node.cpp docks_):
    # center + 4 quadrants at 25%/75% of world
    dock_centers = [
        (width_cells // 2, height_cells // 2),
        (width_cells // 4, height_cells // 4),
        (3 * width_cells // 4, height_cells // 4),
        (width_cells // 4, 3 * height_cells // 4),
        (3 * width_cells // 4, 3 * height_cells // 4),
    ]

    # Large rectangular obstacles
    placements = [
        (80,  80,  60, 15),
        (200, 100, 40, 80),
        (100, 220, 90, 20),
        (320, 180, 25, 100),
        (260, 340, 120, 30),
        (90,  360, 40, 70),
        (380, 80,  30, 60),
    ]

    def overlaps_dock(sx, sy, sw, sh, margin=12):
        for (dx, dy) in dock_centers:
            if (sx - margin <= dx <= sx + sw + margin and
                sy - margin <= dy <= sy + sh + margin):
                return True
        return False

    for (x, y, w, h) in placements:
        sx = int(x * width_cells / 500.0)
        sy = int(y * height_cells / 500.0)
        sw = max(4, int(w * width_cells / 500.0))
        sh = max(4, int(h * height_cells / 500.0))
        # Skip obstacles that would overlap a dock
        if overlaps_dock(sx, sy, sw, sh):
            continue
        place_rect(sx, sy, sw, sh)

    # Carve wide free neighborhoods around each dock (2m radius)
    for (dx, dy) in dock_centers:
        clear_rect(dx - 10, dy - 10, 20, 20)

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

        # Scatter robots randomly across the map, with a minimum separation of
        # ~4 m between any two to avoid immediate conflicts.
        placed: list[tuple[float, float]] = []
        min_sep_m = 4.0
        for i in range(self.n_initial):
            x, y = self._random_safe_spawn(placed, clearance_cells=10,
                                            min_sep_m=min_sep_m)
            placed.append((x, y))
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
        # Use random scatter with min separation from existing robots
        existing = [(r['pose'][0], r['pose'][1]) for r in self.robots.values()]
        x, y = self._random_safe_spawn(existing, clearance_cells=10, min_sep_m=4.0)
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

    def _robot_body_blocked(self, wx: float, wy: float, body_radius: int = 2) -> bool:
        """True if any cell within `body_radius` of the robot's world position is occupied."""
        cgx, cgy = self._world_to_grid(wx, wy)
        for dy in range(-body_radius, body_radius + 1):
            for dx in range(-body_radius, body_radius + 1):
                if dx * dx + dy * dy > body_radius * body_radius:
                    continue
                if self._cell_occupied(cgx + dx, cgy + dy):
                    return True
        return False

    def _random_safe_spawn(self, already_placed: list[tuple[float, float]],
                            clearance_cells: int = 10,
                            min_sep_m: float = 4.0) -> tuple[float, float]:
        """Pick a random cell with clearance and at least `min_sep_m` from all
        already-placed robots. Falls back to relaxed constraints if needed."""

        def has_clearance(gx, gy):
            for dy in range(-clearance_cells, clearance_cells + 1):
                for dx in range(-clearance_cells, clearance_cells + 1):
                    if self._cell_occupied(gx + dx, gy + dy):
                        return False
            return True

        margin = clearance_cells + 2
        for min_sep in (min_sep_m, min_sep_m * 0.5, 0.0):
            for _ in range(500):
                gx = random.randint(margin, self.w_cells - margin - 1)
                gy = random.randint(margin, self.h_cells - margin - 1)
                if not has_clearance(gx, gy):
                    continue
                wx = (gx + 0.5) * self.res
                wy = (gy + 0.5) * self.res
                if any(math.hypot(wx - px, wy - py) < min_sep
                       for (px, py) in already_placed):
                    continue
                return wx, wy
        self.get_logger().warning('_random_safe_spawn: falling back to centre')
        return self.width_m * 0.5, self.height_m * 0.5

    def _safe_spawn(self, x: float, y: float, clearance_cells: int = 10) -> tuple[float, float]:
        """Find a spawn position with at least `clearance_cells` of free space around it.
        Snaps to cell centers so initial positions line up cleanly on the grid."""
        cgx, cgy = self._world_to_grid(x, y)

        def has_clearance(gx, gy):
            for dy in range(-clearance_cells, clearance_cells + 1):
                for dx in range(-clearance_cells, clearance_cells + 1):
                    if self._cell_occupied(gx + dx, gy + dy):
                        return False
            return True

        if has_clearance(cgx, cgy):
            return (cgx + 0.5) * self.res, (cgy + 0.5) * self.res

        for r in range(1, 40):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    if has_clearance(cgx + dx, cgy + dy):
                        return ((cgx + dx) + 0.5) * self.res, ((cgy + dy) + 0.5) * self.res
        self.get_logger().warning(f'_safe_spawn: no clear cell near ({x:.1f},{y:.1f})')
        return self._nearest_free(x, y)

    def _nearest_free(self, x: float, y: float) -> tuple[float, float]:
        """Return (x, y) if free, else scan outward for the nearest free cell."""
        gx, gy = self._world_to_grid(x, y)
        if not self._cell_occupied(gx, gy):
            return x, y
        for r in range(1, 30):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    if not self._cell_occupied(gx + dx, gy + dy):
                        return (gx + dx + 0.5) * self.res, (gy + dy + 0.5) * self.res
        self.get_logger().warning(f'_nearest_free: no free cell near ({x:.1f}, {y:.1f})')
        return x, y

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

            # Trust A* (which plans with clearance=1) to keep robots off walls.
            # World_sim just does point-mass collision on the target cell.
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
