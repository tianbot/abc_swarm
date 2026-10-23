#!/usr/bin/env python3
"""Common demo services, feedback readiness and gated manual leader commands.

This owns the experiment lifecycle, not hardware health certification. GUI and
keyboard use the same teleop input; autonomous path nodes retain their output.
"""
import json
import math
import threading
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class DemoRuntime:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = rospy.get_param('~mode', 'manual')
        if self.mode not in ('manual', 'path'):
            raise ValueError('mode must be manual or path')
        self.fixed = rospy.get_param('~fixed_frame', 'world')
        self.frames = rospy.get_param('~robot_frames')
        self.topics = rospy.get_param('~feedback_topics', [])
        self.timeout = float(rospy.get_param('~feedback_timeout', 0.5))
        self.command_timeout = float(rospy.get_param('~command_timeout', 0.3))
        self.max_linear = float(rospy.get_param('~max_linear', 0.1))
        self.max_angular = float(rospy.get_param('~max_angular', 0.6))
        if not self.frames or any(not math.isfinite(v) or v <= 0 for v in
                (self.timeout, self.command_timeout, self.max_linear, self.max_angular)):
            raise ValueError('Robot frames and positive finite limits are required')
        self.listener = tf.TransformListener()
        self.feedback = {}
        self.path_status = ''
        self.command = None
        self.active = False
        self.fault = ''
        self.last_json = None
        self.enabled_param = '/abc_swarm_demo/enabled'
        self.publisher = (rospy.Publisher(rospy.get_param('~cmd_topic'), Twist, queue_size=1)
                          if self.mode == 'manual' else None)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.subscribers = [rospy.Subscriber(topic, Odometry, self.odom_cb,
                            callback_args=topic, queue_size=1) for topic in self.topics]
        if self.mode == 'manual':
            self.subscribers.append(rospy.Subscriber('~teleop_cmd', Twist,
                                                     self.command_cb, queue_size=1))
        else:
            self.subscribers.append(rospy.Subscriber('/abc_swarm_path/status', String,
                                                     self.path_cb, queue_size=1))
        self.services = [rospy.Service('~set_enabled', SetBool, self.set_enabled),
                         rospy.Service('~reset', Trigger, self.reset)]
        rospy.on_shutdown(self.shutdown)

    def odom_cb(self, msg, topic):
        with self.lock:
            self.feedback[topic] = (msg.header.stamp, time.monotonic(),
                                   msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def command_cb(self, msg):
        with self.lock:
            if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
                self.command = (time.monotonic(), msg.linear.x, msg.angular.z)
            else:
                self.command = None

    def path_cb(self, msg):
        with self.lock:
            self.path_status = msg.data

    def readiness(self):
        now = rospy.Time.now()
        for frame in self.frames:
            try:
                stamp = self.listener.getLatestCommonTime(self.fixed, frame)
                age = (now - stamp).to_sec()
                if stamp == rospy.Time(0) or not 0 <= age <= self.timeout:
                    return False, 'stale pose: ' + frame
                trans, rot = self.listener.lookupTransform(self.fixed, frame, stamp)
                if not all(math.isfinite(v) for v in (*trans, *rot)):
                    return False, 'invalid pose: ' + frame
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return False, 'waiting for pose: ' + frame
        for topic in self.topics:
            sample = self.feedback.get(topic)
            if sample is None:
                return False, 'waiting for odometry: ' + topic
            stamp, received, v, w = sample
            if (stamp == rospy.Time(0) or not 0 <= (now - stamp).to_sec() <= self.timeout
                    or time.monotonic() - received > self.timeout
                    or not all(math.isfinite(x) for x in (v, w))):
                return False, 'stale/invalid odometry: ' + topic
        if self.mode == 'path' and not self.path_status.startswith(
                ('ready', 'tracking', 'settling', 'waiting for follower feedback')):
            return False, self.path_status or 'waiting for prepared path'
        return True, 'required feedback ready'

    def zero(self):
        if self.publisher is not None:
            self.publisher.publish(Twist())

    def set_enabled(self, request):
        with self.lock:
            if request.data:
                if self.mode == 'path' and self.fault:
                    return SetBoolResponse(success=False, message='Reset the path experiment after a fault')
                ready, reason = self.readiness()
                if not ready:
                    return SetBoolResponse(success=False, message=reason)
            self.command = None  # Never replay input received before arming.
            self.fault = ''
            self.active = bool(request.data)
            rospy.set_param(self.enabled_param, self.active)
            if not self.active:
                self.zero()
            return SetBoolResponse(success=True, message='enabled' if self.active else 'stopped')

    def reset(self, request):
        with self.lock:
            if rospy.get_param(self.enabled_param, False):
                return TriggerResponse(success=False, message='Stop the experiment before resetting')
            if self.mode != 'path':
                return TriggerResponse(success=False, message='This manual experiment has no path to reset')
            self.path_status = ''
        try:
            result = rospy.ServiceProxy('/abc_swarm_path/reset', Trigger)()
            with self.lock:
                if result.success:
                    self.fault = ''
            return result
        except rospy.ServiceException as exc:
            return TriggerResponse(success=False, message=str(exc))

    def step(self):
        with self.lock:
            ready, reason = self.readiness()
            requested = bool(rospy.get_param(self.enabled_param, False))
            if requested and self.mode == 'path' and self.fault:
                rospy.set_param(self.enabled_param, False)
                requested = False
            if requested and not ready:
                rospy.set_param(self.enabled_param, False)
                requested = False
                self.fault = reason
            if requested and not self.active:
                self.command = None
                self.fault = ''
            self.active = requested
            if self.publisher is not None:
                cmd = Twist()
                if (self.active and self.command is not None
                        and time.monotonic() - self.command[0] <= self.command_timeout):
                    cmd.linear.x = max(-self.max_linear, min(self.max_linear, self.command[1]))
                    cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.command[2]))
                self.publisher.publish(cmd)
            state = 'running' if self.active else ('ready' if ready else 'waiting')
            if self.mode == 'path' and self.path_status.startswith('completed'):
                state, reason = 'completed', self.path_status
            elif self.mode == 'path' and self.path_status.startswith('fault:'):
                state, reason = 'fault', self.path_status
            elif self.fault:
                state, reason = 'fault', self.fault
            status = json.dumps({'schema_version': 1, 'state': state, 'reason': reason,
                                 'enabled': self.active, 'ready': ready and not (self.mode == 'path' and self.fault),
                                 'mode': self.mode,
                                 'vrpn_server': rospy.get_param('/vrpn_client_node/server', ''),
                                 'vrpn_port': rospy.get_param('/vrpn_client_node/port', 0)},
                                sort_keys=True)
            if status != self.last_json:
                self.status_pub.publish(String(data=status))
                self.last_json = status

    def shutdown(self):
        self.zero()
        try:
            rospy.set_param(self.enabled_param, False)
        except Exception:
            pass  # The master may already have shut down.

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('abc_swarm_demo')
    DemoRuntime().run()
