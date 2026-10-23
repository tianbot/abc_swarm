#!/usr/bin/env python3
"""Controller geometry and actual odometry feedforward regressions."""
import importlib.util
import math
from pathlib import Path
import threading
import unittest
from nav_msgs.msg import Odometry
from abc_swarm.leader_follower_control import tracking_velocity

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('lf_tracker', ROOT / 'scripts/leader_follower_tracker.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LeaderFollowerTests(unittest.TestCase):
    def test_stationary_offsets_and_forward_feedforward(self):
        for offset in [(-0.5, 0.5), (-0.5, -0.5), (-0.5, 0)]:
            self.assertEqual(tracking_velocity((*offset, 0), (0, 0), offset, .1, .5, .5), (0, 0))
        v, w = tracking_velocity((-.5, 0, 0), (.06, 0), (-.5, 0), .1, .5, .5)
        self.assertAlmostEqual(v, .06)
        self.assertAlmostEqual(w, 0)

    def test_front_point_error_contracts_under_unicycle_dynamics(self):
        pose, offset, d, dt = (-.8, .1, .4), (-.5, .5), .1, 1e-5
        vl, wl = .05, .1
        v, w = tracking_velocity(pose, (vl, wl), offset, d, .5, .5)
        def error(x, y, yaw):
            return offset[0] + d - x - d*math.cos(yaw), offset[1] - y - d*math.sin(yaw)
        x, y, yaw = pose
        before = error(*pose)
        after = error(x+(v*math.cos(yaw)-vl+wl*y)*dt,
                      y+(v*math.sin(yaw)-wl*x)*dt, yaw+(w-wl)*dt)
        for a,b in zip(before, after):
            self.assertAlmostEqual((b-a)/dt, -.5*a, places=4)

    def test_actual_odometry_updates_the_velocity_input(self):
        node = module.LeaderFollower.__new__(module.LeaderFollower)
        node.lock, node.odom = threading.Lock(), None
        msg = Odometry()
        msg.header.stamp.secs = 100
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = .07, -.12
        node.odom_cb(msg)
        self.assertEqual(node.odom[2:], (.07, -.12))

    def test_three_robot_topology_and_default_disabled(self):
        import roslaunch
        for formation, leader in [('triangle','tianbot_mini_01'),('snake','tianbot_mini_02')]:
            config = roslaunch.config.ROSLaunchConfig()
            roslaunch.xmlloader.XmlLoader().load(str(ROOT / 'launch/mocap' /
                ('demo_mocap_three_mini_leader_follower_'+formation+'.launch')), config,
                verbose=False, argv=['start_driver:=false','start_vrpn:=false','start_teleop:=false'])
            prefix = '/tianbot_mini_03_leader_follower/'
            self.assertEqual(config.params[prefix+'leader_frame'].value, leader)
            self.assertEqual(config.params[prefix+'leader_odom_topic'].value, '/'+leader+'/odom')
            self.assertFalse(config.params['/abc_swarm_demo/enabled'].value)
            self.assertFalse(any(n.package=='rosserial_server' for n in config.nodes))
