"""Integration test: only one robot claims each frontier."""
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
                'num_robots': '3',
                'world': 'warehouse',
                'failure_rate': '0.0',
                'comm_radius': '20.0',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class FrontierAuctionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_frontier_auction')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_exclusive_claim(self):
        claims = {}  # frontier_id -> set(robot_id)

        def cb(msg: FrontierBid):
            if msg.claim:
                claims.setdefault(msg.frontier_id, set()).add(msg.robot_id)

        sub = self.node.create_subscription(
            FrontierBid, '/swarm/frontier_bids', cb, 20)
        end = time.time() + 45.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)

        self.assertTrue(claims, 'no claims observed — auction never ran')
        for fid, robots in claims.items():
            self.assertEqual(len(robots), 1,
                             f'frontier {fid} claimed by multiple robots: {robots}')
