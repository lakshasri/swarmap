"""Integration test: neighbour takes over when a robot dies."""
import json
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from std_msgs.msg import String
from swarmap_msgs.msg import FrontierBid


@pytest.mark.launch_test
def generate_test_description():
    sim_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'simulation.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_robots': '5',
                'world': 'warehouse',
                'failure_rate': '0.2',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class FailureRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_failure_recovery')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_survivors_cover_gap(self):
        failed = []
        bids_after_failure = []
        failure_time = [None]

        def health_cb(msg: String):
            data = json.loads(msg.data)
            for r in data.get('robots', []):
                if r.get('state') == 'FAILED' and r['id'] not in failed:
                    failed.append(r['id'])
                    if failure_time[0] is None:
                        failure_time[0] = time.time()

        def bid_cb(msg: FrontierBid):
            if failure_time[0] is not None and msg.claim:
                bids_after_failure.append((time.time(), msg.robot_id))

        self.node.create_subscription(String, '/swarm/health', health_cb, 5)
        self.node.create_subscription(
            FrontierBid, '/swarm/frontier_bids', bid_cb, 20)

        end = time.time() + 120.0
        while time.time() < end and (not failed or len(bids_after_failure) < 3):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(failed, 'no robots failed during the test window')
        self.assertTrue(bids_after_failure,
                        'no survivors bid on frontiers after a failure')
        dt = bids_after_failure[0][0] - failure_time[0]
        self.assertLess(dt, 15.0,
                        f'takeover took {dt:.1f}s (> 15 s)')
