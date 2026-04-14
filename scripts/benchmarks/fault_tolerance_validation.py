#!/usr/bin/env python3
"""Fault-tolerance validation: 10 runs × random seeds, check coverage stats.

Each run launches demo_fault_tolerance, waits for the mission to end, reads
the final coverage from /dashboard/stats, then records the result.

Pass criteria (phase 10 spec):
    mean coverage >= 0.85   minimum coverage >= 0.75
    mission success rate (coverage > 0.85) >= 0.80
"""
import argparse
import csv
import json
import os
import statistics
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CoverageWatcher(Node):
    def __init__(self):
        super().__init__('coverage_watcher')
        self.final_coverage = None
        self.create_subscription(String, '/dashboard/stats', self._cb, 5)

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'coverage' in data:
                self.final_coverage = float(data['coverage'])
        except json.JSONDecodeError:
            pass


def run_once(seed: int, duration: float) -> float:
    proc = subprocess.Popen([
        'ros2', 'launch', 'swarmap_bringup', 'demo_fault_tolerance.launch.py',
        f'seed:={seed}',
    ])
    rclpy.init()
    watcher = CoverageWatcher()
    end = time.time() + duration + 30.0
    try:
        while time.time() < end:
            rclpy.spin_once(watcher, timeout_sec=0.25)
        cov = watcher.final_coverage or 0.0
    finally:
        watcher.destroy_node()
        rclpy.shutdown()
        proc.send_signal(2)
        try:
            proc.wait(30)
        except subprocess.TimeoutExpired:
            proc.kill()
    return cov


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=10)
    ap.add_argument('--duration', type=float, default=300.0)
    ap.add_argument('--out', default='results/benchmark/fault_tolerance.csv')
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    coverages = []
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run', 'seed', 'coverage'])
        for i in range(args.runs):
            seed = 1000 + i
            cov = run_once(seed, args.duration)
            print(f'run {i} seed={seed} coverage={cov:.3f}')
            w.writerow([i, seed, cov])
            coverages.append(cov)

    mean = statistics.mean(coverages)
    mn = min(coverages)
    success = sum(1 for c in coverages if c > 0.85) / len(coverages)
    print(f'mean={mean:.3f}  min={mn:.3f}  success_rate={success:.2f}')

    ok = mean >= 0.85 and mn >= 0.75 and success >= 0.80
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
