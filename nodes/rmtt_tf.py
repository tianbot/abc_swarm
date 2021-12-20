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


if __name__ == '__main__':
    rospy.init_node('rmtt_tf_listener')
    target_frame = rospy.get_param('~target_frame')
    follower_robot_name = rospy.get_param('~follower_robot_name')
    set_distance = rospy.get_param('~set_distance')

    listener = tf.TransformListener()

    follower_vel = rospy.Publisher(follower_robot_name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    pid_linear_x = PID(1, 0.0, 0.0)
    pid_linear_x.output_limits = (-0.5, 0.5)
    pid_linear_y = PID(1, 0.0, 0.0)
    pid_linear_y.output_limits = (-0.5, 0.5)
    pid_angular = PID(2, 0.0, 0.0)
    pid_angular.output_limits = (-1.5, 1.5)
    #pid_linear.sample_time = pid_angular.sample_time = 0.1

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(follower_robot_name+'/base_link', target_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular=pid_angular(transformations.euler_from_quaternion(rot)[2])
        linear_x=pid_linear_x(trans[0])
        linear_y=pid_linear_y(trans[1])

        if abs(linear_x) < 0.02:
            linear_x = 0

        if abs(linear_y) < 0.02:
            linear_y = 0


        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular
        follower_vel.publish(msg)

        rate.sleep()