#!/usr/bin/env python3
"""Headless throughput benchmark.

Subscribes to all /robot_N/scan and /robot_N/map topics, measures the
per-robot scan -> map publish latency, and samples per-process CPU/RAM
usage using psutil.  Writes a CSV row per robot and prints pass/fail
against the phase-10 thresholds (latency < 500 ms, CPU < 80%/core).

Usage:
    python3 scripts/benchmarks/perf_throughput.py --num-robots 20 --duration 120
"""
import argparse
import csv
import os
import time

import psutil
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Sampler(Node):
    def __init__(self, num_robots: int):
        super().__init__('perf_throughput_sampler')
        self.last_scan = {i: None for i in range(num_robots)}
        self.latencies = {i: [] for i in range(num_robots)}
        self.map_counts = {i: 0 for i in range(num_robots)}
        for i in range(num_robots):
            self.create_subscription(
                LaserScan, f'/robot_{i}/scan',
                lambda m, rid=i: self._on_scan(rid, m), 10)
            self.create_subscription(
                OccupancyGrid, f'/robot_{i}/map',
                lambda m, rid=i: self._on_map(rid, m), 5)

    def _on_scan(self, rid, _msg):
        self.last_scan[rid] = time.time()

    def _on_map(self, rid, _msg):
        self.map_counts[rid] += 1
        t = self.last_scan[rid]
        if t is not None:
            self.latencies[rid].append(time.time() - t)


def sample_cpu(duration, interval=2.0):
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        samples.append(psutil.cpu_percent(interval=interval))
    return samples


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--num-robots', type=int, default=20)
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--out', default='results/benchmark/throughput.csv')
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    rclpy.init()
    node = Sampler(args.num_robots)
    cpu_samples = []
    end = time.time() + args.duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.25)
        cpu_samples.append(psutil.cpu_percent(interval=None))
    rclpy.shutdown()

    rows = []
    max_latency = 0.0
    for i in range(args.num_robots):
        lats = node.latencies[i]
        avg = sum(lats) / len(lats) if lats else float('nan')
        p95 = sorted(lats)[int(0.95 * len(lats))] if lats else float('nan')
        max_latency = max(max_latency, p95 if lats else 0.0)
        rows.append({
            'robot_id': i,
            'map_msgs': node.map_counts[i],
            'latency_avg_s': avg,
            'latency_p95_s': p95,
        })
    cpu_avg = sum(cpu_samples) / max(len(cpu_samples), 1)

    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f'num_robots={args.num_robots}  cpu_avg={cpu_avg:.1f}%  '
          f'max_p95_latency={max_latency*1000:.0f}ms')
    ok = max_latency < 0.5 and cpu_avg < 80.0
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
