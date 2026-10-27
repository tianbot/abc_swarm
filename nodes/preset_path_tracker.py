#!/usr/bin/env python3
"""TF/ROS lifecycle for the independent preset-path Pure Pursuit algorithm."""
import json
import math
import time

import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from abc_swarm.preset_paths import preset_path
from abc_swarm.pure_pursuit import PurePursuit


class PresetPathNode:
    def __init__(self):
        self.fixed = rospy.get_param('~fixed_frame', 'world')
        self.robot = rospy.get_param('~robot_frame')
        self.enabled_param = rospy.get_param('~enabled_param', '/abc_swarm_demo/enabled')
        self.timeout = float(rospy.get_param('~tf_timeout', 0.5))
        self.settle_timeout = float(rospy.get_param('~settle_timeout', 30.0))
        self.settle_tolerance = float(rospy.get_param('~settle_tolerance', 0.1))
        self.start_tolerance = float(rospy.get_param('~start_tolerance', 0.15))
        self.edges = rospy.get_param('~edges', [])
        self.front_distance = float(rospy.get_param('~follow/front_distance', 0.1))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.timeout, self.settle_timeout, self.settle_tolerance,
                    self.start_tolerance, self.front_distance)):
            raise ValueError('Timeouts and tolerances must be positive and finite')
        self.listener = tf.TransformListener()
        self.cmd = rospy.Publisher(rospy.get_param('~cmd_topic'), Twist, queue_size=1)
        self.raw_pub=rospy.Publisher('~raw_cmd',Twist,queue_size=1)
        self.config_pub=rospy.Publisher('~effective_config',String,queue_size=1,latch=True)
        self.path_pub = rospy.Publisher('~path', Path, queue_size=1, latch=True)
        self.actual_pub = rospy.Publisher('~actual_path', Path, queue_size=1)
        self.target_pub = rospy.Publisher('~target', PointStamped, queue_size=1)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.last_status = None
        self.follower_topics = rospy.get_param('~follower_status_topics', [])
        self.follower_states = {}
        self.follower_subscribers = [rospy.Subscriber(
            topic, String, self.follower_status, callback_args=topic, queue_size=1)
            for topic in self.follower_topics]
        self.controller = None
        self.fault = False
        self.settle_started = None
        self.wait_started = None
        self.followers_started = False
        self.actual = Path()
        self.reset_requested = False
        self.reset_service = rospy.Service('~reset', Trigger, self.reset)
        rospy.on_shutdown(self.stop)

    def follower_status(self, msg, topic):
        self.follower_states[topic] = msg.data

    def status(self, value):
        if value != self.last_status:
            self.status_pub.publish(String(data=value))
            self.last_status = value
            rospy.loginfo('%s: %s', rospy.get_name(), value)

    def stop(self):
        self.cmd.publish(Twist())

    def pose(self, parent, child):
        stamp = self.listener.getLatestCommonTime(parent, child)
        age = (rospy.Time.now() - stamp).to_sec()
        if stamp == rospy.Time(0) or age < 0 or age > self.timeout:
            raise ValueError('TF unavailable or stale: ' + child)
        trans, rot = self.listener.lookupTransform(parent, child, stamp)
        if not all(math.isfinite(v) for v in (*trans, *rot)):
            raise ValueError('Nonfinite TF: ' + child)
        return trans[0], trans[1], tf.transformations.euler_from_quaternion(rot)[2]

    def reset(self, request):
        # The service thread requests work; only the control loop mutates progress.
        if rospy.get_param(self.enabled_param, False):
            return TriggerResponse(success=False, message='Disable the experiment before resetting')
        self.reset_requested = True
        return TriggerResponse(success=True, message='Reset requested; wait for ready status and path preview')

    def prepare(self, pose):
        points = preset_path(
            shape=rospy.get_param('~shape', 'circle'),
            radius=float(rospy.get_param('~radius', 0.8)),
            width=float(rospy.get_param('~width', 2.5)),
            height=float(rospy.get_param('~height', 2.0)), origin=pose,
            clockwise=rospy.get_param('~clockwise', False))
        self.controller = PurePursuit(
            points, laps=rospy.get_param('~laps', 1),
            speed=float(rospy.get_param('~speed', 0.15)),
            lookahead=float(rospy.get_param('~lookahead', 0.25)),
            max_angular=float(rospy.get_param('~max_angular', 0.5)),
            goal_tolerance=float(rospy.get_param('~goal_tolerance', 0.05)),
            max_error=float(rospy.get_param('~max_error', 0.5)))
        self.config_pub.publish(String(data=json.dumps(dict(node_type='path_tracker',algorithm='pure_pursuit',
            frame=self.fixed,robot=self.robot,start_pose=list(pose),speed=self.controller.speed,
            lookahead=self.controller.lookahead,max_angular=self.controller.max_angular,
            goal_tolerance=self.controller.goal_tolerance,max_error=self.controller.max_error,
            front_distance=self.front_distance,tf_timeout=self.timeout),sort_keys=True)))
        self.start_yaw = pose[2]
        self.settle_started = None
        self.wait_started = None
        self.followers_started = False
        self.actual = Path()
        self.actual.header.frame_id = self.fixed
        self.fault = False
        path = Path()
        path.header.frame_id, path.header.stamp = self.fixed, rospy.Time.now()
        for i, (x, y) in enumerate(points):
            msg = PoseStamped()
            msg.header = path.header
            msg.pose.position.x, msg.pose.position.y = x, y
            nxt = points[(i + 1) % (len(points) - 1)]
            yaw = math.atan2(nxt[1] - y, nxt[0] - x)
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = q
            path.poses.append(msg)
        self.path_pub.publish(path)
        self.actual_pub.publish(self.actual)

    def abort(self, reason):
        self.stop()
        rospy.set_param(self.enabled_param, False)
        self.fault = True
        self.status('fault: ' + reason + '; restart launch or reset before restarting')

    def step(self):
        enabled = rospy.get_param(self.enabled_param, False)
        if self.reset_requested:
            self.reset_requested = False
            if enabled:
                self.abort('reset requested while experiment became enabled')
                return
            self.controller = None
            self.fault = False
            self.status('resetting')
            self.path_pub.publish(Path())
        if self.fault:
            self.stop()
            if enabled:
                rospy.set_param(self.enabled_param, False)
            return
        if self.controller is None and enabled:
            self.abort('path not prepared')
            return
        if not enabled and self.controller is not None:
            self.stop()
            if self.controller.completed:
                # Retain completion status; a new lap requires explicit reset.
                return
            self.settle_started = None
            self.wait_started = None
            self.followers_started = False
            self.status('ready; waiting for feedback (path frozen)')
            return
        try:
            pose = self.pose(self.fixed, self.robot)
            if self.controller is None:
                self.prepare(pose)
                self.stop()
                self.status('ready; waiting for feedback (path frozen)')
                return
            if not all(self.follower_states.get(topic) == 'tracking' for topic in self.follower_topics):
                self.stop()
                if self.wait_started is None:
                    self.wait_started = time.monotonic()
                if self.followers_started or time.monotonic() - self.wait_started > 2.0:
                    self.abort('follower feedback not ready')
                else:
                    self.status('waiting for follower feedback')
                return
            self.followers_started = True
            if self.controller.progress == 0:
                first = self.controller.points[0]
                angle = math.atan2(math.sin(pose[2] - self.start_yaw),
                                   math.cos(pose[2] - self.start_yaw))
                if math.hypot(pose[0] - first[0], pose[1] - first[1]) > self.start_tolerance or abs(angle) > 0.35:
                    raise ValueError('Robot moved from prepared start pose')
            linear, angular = self.controller.step(pose)
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = linear, angular
            self.raw_pub.publish(cmd)
            self.cmd.publish(cmd)
            if self.controller.completed:
                if self.settle_started is None:
                    self.settle_started = time.monotonic()
                # Match the same configurable front point used by both followers.
                settled = all(math.hypot(p[0] + self.front_distance * math.cos(p[2]) - float(e['x']) - self.front_distance,
                                         p[1] + self.front_distance * math.sin(p[2]) - float(e['y']))
                              <= self.settle_tolerance
                              for e in self.edges
                              for p in [self.pose(e['leader'], e['follower'])])
                if settled or time.monotonic() - self.settle_started >= self.settle_timeout:
                    rospy.set_param(self.enabled_param, False)
                    self.status('completed' if settled else 'completed; formation settling timed out')
                else:
                    self.status('settling followers; leader stopped')
            else:
                self.status('tracking')
            stamp = rospy.Time.now()
            target = PointStamped()
            target.header.frame_id, target.header.stamp = self.fixed, stamp
            target.point.x, target.point.y = self.controller.target
            self.target_pub.publish(target)
            actual = PoseStamped()
            actual.header.frame_id, actual.header.stamp = self.fixed, stamp
            actual.pose.position.x, actual.pose.position.y = pose[:2]
            q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
            (actual.pose.orientation.x, actual.pose.orientation.y,
             actual.pose.orientation.z, actual.pose.orientation.w) = q
            self.actual.poses.append(actual)
            self.actual.poses = self.actual.poses[-4000:]
            self.actual.header.stamp = stamp
            self.actual_pub.publish(self.actual)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, ValueError) as exc:
            if enabled:
                self.abort(str(exc))
            else:
                self.stop()
                self.status('waiting for valid pose/config: ' + str(exc))

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('preset_path_tracker')
    PresetPathNode().run()
