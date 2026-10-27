#!/usr/bin/env python3
"""Manual relay contract, with ROS IO mocked and no master or hardware."""
import json
from pathlib import Path
import runpy
import threading
import time
import unittest
import rospy
from unittest.mock import Mock, patch


Runtime = runpy.run_path(str(Path(__file__).resolve().parents[1]/'nodes/demo_runtime.py'))['DemoRuntime']


class ManualCommandTests(unittest.TestCase):
    def test_readiness_uses_odometry_timeout_independently_of_tf_timeout(self):
        node=Runtime.__new__(Runtime)
        node.frames=[];node.topics=['/test_robot/odom'];node.mode='manual'
        node.tf_timeout=.1;node.odom_timeout=.7;node.future_tolerance=.08
        node.feedback={'/test_robot/odom':(rospy.Time.from_sec(99.5),time.monotonic(),0.,0.)}
        with patch('rospy.Time.now',return_value=rospy.Time.from_sec(100.)):
            self.assertTrue(node.readiness()[0])
            node.feedback['/test_robot/odom']=(rospy.Time.from_sec(99.2),time.monotonic(),0.,0.)
            self.assertIn('timestamp age',node.readiness()[1])

    def runtime(self):
        node=Runtime.__new__(Runtime)
        node.lock=threading.RLock();node.mode='manual';node.active=True
        node.enabled_param='/test_demo/enabled';node.command_timeout=.3
        node.publisher=Mock();node.status_pub=Mock();node.last_json=None
        node.path_status='';node.fault='';node.readiness=lambda:(True,'required feedback ready')
        return node

    def step(self,node):
        with patch('rospy.set_param'),patch('rospy.get_param',side_effect=lambda name,default=None:default):
            node.step()
        return node.publisher.publish.call_args[0][0]

    def test_live_manual_twist_is_forwarded_without_limits(self):
        node=self.runtime();node.command=(time.monotonic(),.3,4.2)
        output=self.step(node)
        self.assertEqual((output.linear.x,output.angular.z),(.3,4.2))
        status=json.loads(node.status_pub.publish.call_args[0][0].data)
        self.assertEqual(status['input_command'],[.3,4.2])
        self.assertEqual(status['output_command'],[.3,4.2])
        self.assertEqual(status['command_state'],'forwarded')

    def test_timeout_and_feedback_loss_stop_without_replaying_requests(self):
        node=self.runtime();node.command=(time.monotonic()-1.,.3,4.2)
        output=self.step(node);self.assertEqual((output.linear.x,output.angular.z),(0.,0.))
        self.assertEqual(json.loads(node.status_pub.publish.call_args[0][0].data)['command_state'],'command_timeout')
        node.command=(time.monotonic(),.3,4.2)
        node.readiness=lambda:(False,'stale pose: test_robot')
        output=self.step(node)
        self.assertEqual((output.linear.x,output.angular.z),(0.,0.));self.assertIsNone(node.command)
        node.readiness=lambda:(True,'required feedback ready')
        output=self.step(node)
        self.assertEqual((output.linear.x,output.angular.z),(0.,0.));self.assertIsNone(node.command)


if __name__=='__main__':unittest.main()
