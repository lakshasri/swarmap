#!/usr/bin/env python3
"""Read a rosbag2 recorded by the demo launches and plot coverage vs. time.

Overlays vertical markers at each failure event pulled from /dashboard/events.
"""
import argparse
import json
import os

import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='sqlite3'),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {}
    while reader.has_next():
        topic, raw, t = reader.read_next()
        cls = get_message(types[topic])
        msg = deserialize_message(raw, cls)
        msgs.setdefault(topic, []).append((t * 1e-9, msg))
    return msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    data = read_bag(args.bag)
    t0 = None
    times, covs = [], []
    for t, msg in data.get('/dashboard/stats', []):
        try:
            d = json.loads(msg.data)
        except AttributeError:
            continue
        if t0 is None:
            t0 = t
        times.append(t - t0)
        covs.append(float(d.get('coverage', 0.0)))

    failures = []
    for t, msg in data.get('/dashboard/events', []):
        try:
            d = json.loads(msg.data)
        except AttributeError:
            continue
        if d.get('type') == 'failure':
            failures.append((t - (t0 or t), d.get('robot_id')))

    plt.figure(figsize=(9, 5))
    plt.plot(times, [c * 100 for c in covs], label='coverage %')
    for t, rid in failures:
        plt.axvline(t, color='red', alpha=0.3)
        plt.text(t, 5, f'{rid}', rotation=90, fontsize=7, color='red')
    plt.xlabel('time (s)')
    plt.ylabel('coverage %')
    plt.title('Coverage over time with failure markers')
    plt.grid(True, alpha=0.3)
    plt.legend()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=120, bbox_inches='tight')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
