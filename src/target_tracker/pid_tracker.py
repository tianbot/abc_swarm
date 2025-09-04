#!/usr/bin/env python

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
pid_linear.output_limits = (-0.25, 0.25)
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

    listener = tf.TransformListener()

    follower_vel = rospy.Publisher(tracker_motion_topic, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    #pid_linear.sample_time = pid_angular.sample_time = 0.1
    srv = Server(tf_pidConfig, callback)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(tracker_frame, target_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('tf from {} to {} lookup failed'.format(tracker_frame, target_frame))
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
        msg.linear.x = linear
        msg.angular.z = angular
        follower_vel.publish(msg)
        rospy.logdebug('{}, linear.x: {}, angular.z: {}'.format(tracker_frame, linear, angular))

        rate.sleep()
