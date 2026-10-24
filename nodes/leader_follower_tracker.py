#!/usr/bin/env python3
"""Planar leader-follower control using a point ahead of the follower axle.

Adapted from the original nodes/leader_follower.py. Offsets
describe the desired follower axle position in the leader's body frame.
"""

import math
import threading
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


from abc_swarm.feedback import odometry_error
from abc_swarm.leader_follower_control import tracking_velocity

class LeaderFollower:
    def __init__(self):
        self.leader_frame = rospy.get_param('~leader_frame')
        self.follower_frame = rospy.get_param('~follower_frame')
        self.offset = (float(rospy.get_param('~offset_x', -0.5)),
                       float(rospy.get_param('~offset_y', 0.0)))
        self.d = float(rospy.get_param('~front_distance', 0.1))
        self.k_x = float(rospy.get_param('~k_x', 0.5))
        self.k_y = float(rospy.get_param('~k_y', 0.5))
        self.max_linear = float(rospy.get_param('~max_linear', 0.1))
        self.max_angular = float(rospy.get_param('~max_angular', 0.6))
        self.tf_timeout = float(rospy.get_param('~tf_timeout', 0.5))
        self.odom_timeout = float(rospy.get_param('~odom_timeout', 0.5))
        self.future_tolerance = float(rospy.get_param('~odom_future_tolerance', 0.1))
        self.enabled_param = rospy.get_param('~enabled_param',
                                             '/abc_swarm_demo/enabled')
        positive = (self.d, self.k_x, self.k_y, self.max_linear,
                    self.max_angular, self.tf_timeout, self.odom_timeout, self.future_tolerance)
        if (not all(math.isfinite(v) and v > 0 for v in positive)
                or not all(math.isfinite(v) for v in self.offset)):
            raise ValueError('Invalid gains, distances, limits or timeouts')

        self.lock = threading.Lock()
        self.odom = None
        self.listener = tf.TransformListener()
        self.velocity_pub = rospy.Publisher(rospy.get_param('~cmd_topic'),
                                             Twist, queue_size=1)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1,
                                         latch=True)
        self.last_status = None
        self.odom_sub = rospy.Subscriber(rospy.get_param('~leader_odom_topic'),
                                         Odometry, self.odom_cb, queue_size=1)
        rospy.on_shutdown(self.stop)

    def odom_cb(self, msg):
        # Store actual feedback; the legacy callback only assigned locals.
        with self.lock:
            self.odom = (msg.header.stamp, time.monotonic(),
                         msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            rospy.loginfo('%s: %s', rospy.get_name(), value)

    def stop(self):
        self.velocity_pub.publish(Twist())

    def step(self):
        if not rospy.get_param(self.enabled_param, False):
            self.stop()
            self.status('disabled')
            return

        with self.lock:
            odom = self.odom
        now = rospy.Time.now()
        if odom is None:
            self.stop()
            self.status('waiting for leader odometry')
            return
        stamp, received, v_leader, w_leader = odom
        issue = odometry_error(stamp.to_sec(), now.to_sec(), time.monotonic() - received,
                               v_leader, w_leader, self.odom_timeout, self.future_tolerance)
        if issue:
            self.stop()
            self.status('leader odometry: ' + issue)
            return

        try:
            stamp = self.listener.getLatestCommonTime(self.leader_frame,
                                                       self.follower_frame)
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp == rospy.Time(0) or age < 0 or age > self.tf_timeout:
                raise tf.ExtrapolationException('stale TF')
            trans, rot = self.listener.lookupTransform(
                self.leader_frame, self.follower_frame, stamp)
            if not all(math.isfinite(v) for v in (*trans, *rot)):
                raise tf.ExtrapolationException('non-finite TF')
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            self.stop()
            self.status('TF unavailable or stale')
            rospy.logwarn_throttle(2.0, '%s -> %s: %s' % (
                self.leader_frame, self.follower_frame, exc))
            return

        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        linear, angular = tracking_velocity(
            (trans[0], trans[1], yaw), (v_leader, w_leader), self.offset,
            self.d, self.k_x, self.k_y)
        if not all(math.isfinite(v) for v in (linear, angular)):
            self.stop()
            self.status('invalid velocity')
            return
        cmd = Twist()
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, linear))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, angular))
        self.velocity_pub.publish(cmd)
        self.status('tracking')

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('leader_follower_tracker')
    LeaderFollower().run()
