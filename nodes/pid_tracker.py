#!/usr/bin/env python3

import rospy

import math
import tf
from tf import transformations
import geometry_msgs.msg
from tf import broadcaster
from dynamic_reconfigure.server import Server
from abc_swarm.cfg import tf_pidConfig
import numpy as np
from simple_pid import PID

pid_linear = PID(2, 0.5, 0.0)
pid_linear.output_limits = (-0.15, 0.15)
pid_angular = PID(2, 1.0, 0.0)
pid_angular.output_limits = (-1.57, 1.57)

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {linear_kp}, {linear_ki}, {linear_kd}, {angular_kp}, {angular_ki}, {angular_kd}""".format(**config))
    pid_linear.tunings = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    pid_angular.tunings = [float(config.get(key)) for key in ['angular_kp','angular_ki','angular_kd']]
    return config


if __name__ == '__main__':
    rospy.init_node('pid_tracker')
    target_frame = rospy.get_param('~target_frame')
    tracker_frame = rospy.get_param('~tracker_frame')
    tracker_motion_topic = rospy.get_param('~tracker_motion_topic')
    set_distance = rospy.get_param('~set_distance')
    max_linear = float(rospy.get_param('~max_linear', 0.15))
    max_angular = float(rospy.get_param('~max_angular', 1.57))
    tf_timeout = float(rospy.get_param('~tf_timeout', 0.5))
    enable_param = rospy.get_param('~enable_param', '~enabled')
    if not all(math.isfinite(v) and v > 0 for v in
               (max_linear, max_angular, tf_timeout)):
        raise ValueError('Speed limits and tf_timeout must be finite and positive')
    pid_linear.output_limits = (-max_linear, max_linear)
    pid_angular.output_limits = (-max_angular, max_angular)

    listener = tf.TransformListener()

    follower_vel = rospy.Publisher(tracker_motion_topic, geometry_msgs.msg.Twist, queue_size=1)

    def stop():
        follower_vel.publish(geometry_msgs.msg.Twist())
        pid_linear.reset()
        pid_angular.reset()

    rospy.on_shutdown(stop)

    rate = rospy.Rate(10.0)

    #pid_linear.sample_time = pid_angular.sample_time = 0.1
    srv = Server(tf_pidConfig, callback)

    while not rospy.is_shutdown():
        if not rospy.get_param(enable_param, enable_param == '~enabled'):
            stop()
            rate.sleep()
            continue
        try:
            stamp = listener.getLatestCommonTime(tracker_frame, target_frame)
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp == rospy.Time(0) or age < 0 or age > tf_timeout:
                raise tf.ExtrapolationException('TF is stale or has an invalid timestamp')
            (trans, rot) = listener.lookupTransform(tracker_frame, target_frame, rospy.Time())
            if not all(math.isfinite(v) for v in (*trans, *rot)):
                raise tf.ExtrapolationException('TF contains non-finite values')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            stop()
            rospy.logwarn_throttle(2.0, 'TF unavailable or stale: {} -> {}'.format(tracker_frame, target_frame))
            rate.sleep()
            continue

        dis = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        angular = pid_angular(-math.atan2(trans[1],trans[0]))
        angular_z_gap = tf.transformations.euler_from_quaternion(rot)[2]
        rospy.logdebug('L2_dis: {}, Yaw_gap: {}'.format(dis, angular_z_gap))
        linear =  pid_linear(set_distance - dis)

        if abs(linear) < 0.02:
            linear = 0

        # bias tuning
        if abs(linear) < 0.02 and abs(angular_z_gap) != 0:
            linear = 0
            angular = angular_z_gap * 0.5

        if abs(angular) * 180 / math.pi <= 3 and abs(dis) != 0:
            linear = linear * 0.5
            angular = 0

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = max(-max_linear, min(max_linear, linear))
        msg.angular.z = max(-max_angular, min(max_angular, angular))
        follower_vel.publish(msg)
        rospy.logdebug('{}, linear.x: {}, angular.z: {}'.format(tracker_frame, linear, angular))

        rate.sleep()
