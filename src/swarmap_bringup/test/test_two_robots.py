"""Integration test: two-robot handshake.

Launches 2 robots and the core swarm stack in a headless Gazebo world,
then asserts that:

  * each robot publishes a /robot_N/map within 10 s;
  * each robot's RobotStatus message eventually lists the other's id
    under neighbour_ids;
  * the known-cell count in /robot_0/map grows over a 30 s window.
"""
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import OccupancyGrid
from swarmap_msgs.msg import RobotStatus


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
                'comm_radius': '15.0',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TwoRobotHandshake(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_two_robots')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_msg(self, topic, msg_type, timeout):
        bucket = []
        sub = self.node.create_subscription(
            msg_type, topic, lambda m: bucket.append(m), 5)
        end = time.time() + timeout
        while time.time() < end and not bucket:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return bucket

    def test_both_maps_published(self):
        m0 = self._wait_for_msg('/robot_0/map', OccupancyGrid, 20.0)
        m1 = self._wait_for_msg('/robot_1/map', OccupancyGrid, 20.0)
        self.assertTrue(m0, 'robot_0 never published a map')
        self.assertTrue(m1, 'robot_1 never published a map')

    def test_neighbour_discovery(self):
        seen = {'0': False, '1': False}
        end = time.time() + 30.0

        def cb(msg: RobotStatus):
            if msg.robot_id.endswith('0') and 'robot_1' in msg.neighbour_ids:
                seen['0'] = True
            if msg.robot_id.endswith('1') and 'robot_0' in msg.neighbour_ids:
                seen['1'] = True

        sub0 = self.node.create_subscription(
            RobotStatus, '/robot_0/status', cb, 5)
        sub1 = self.node.create_subscription(
            RobotStatus, '/robot_1/status', cb, 5)
        while time.time() < end and not all(seen.values()):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub0)
        self.node.destroy_subscription(sub1)
        self.assertTrue(all(seen.values()),
                        f'neighbour discovery failed: {seen}')

    def test_map_grows(self):
        first = self._wait_for_msg('/robot_0/map', OccupancyGrid, 10.0)[-1]
        time.sleep(30)
        later = self._wait_for_msg('/robot_0/map', OccupancyGrid, 10.0)[-1]
        known_first = sum(1 for v in first.data if v != -1)
        known_later = sum(1 for v in later.data if v != -1)
        self.assertGreater(known_later, known_first,
                           'map did not grow over 30 s')
