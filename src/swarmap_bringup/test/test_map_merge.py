"""Integration test: neighbour map merging covers more than either alone."""
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
from nav_msgs.msg import OccupancyGrid


@pytest.mark.launch_test
def generate_test_description():
    sim_launch = os.path.join(
        get_package_share_directory('swarmap_bringup'),
        'launch', 'simulation.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_robots': '2',
                'world': 'warehouse',
                'failure_rate': '0.0',
                'comm_radius': '25.0',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


def _known(msg: OccupancyGrid) -> int:
    return sum(1 for v in msg.data if v != -1)


class MapMergeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_map_merge')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _latest(self, topic, timeout):
        bucket = []
        sub = self.node.create_subscription(
            OccupancyGrid, topic, lambda m: bucket.append(m), 1)
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return bucket[-1] if bucket else None

    def test_merged_exceeds_individual(self):
        time.sleep(60)  # let them explore
        m0 = self._latest('/robot_0/map', 5.0)
        m1 = self._latest('/robot_1/map', 5.0)
        fused = self._latest('/dashboard/map_compressed', 5.0)
        self.assertIsNotNone(m0)
        self.assertIsNotNone(m1)
        self.assertIsNotNone(fused)
        self.assertGreater(_known(fused), _known(m0))
        self.assertGreater(_known(fused), _known(m1))
