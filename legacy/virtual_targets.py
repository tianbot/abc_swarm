#!/usr/bin/env python
import math
import rospy
import tf
from tf import transformations
from tf import broadcaster
import numpy as np

def broadcaster():
    rospy.init_node('virtual_target', anonymous=True)
    
    # read params
    center_frame = rospy.get_param('~center_frame', 'tbmn_1')
    radius = rospy.get_param('~radius', 0.5)
    target_frame = rospy.get_param('~target_frame', 'virtual_leader')
    linear_speed = rospy.get_param('~speed', 0.1)

    listener = tf.TransformListener()
    listener.waitForTransform(center_frame, "world", rospy.Time(0), rospy.Duration(3.0));
    try:
        (trans, rot) = listener.lookupTransform("world", center_frame, rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    br = tf.TransformBroadcaster()
    x = trans[0]
    y = trans[1]
    theta = 0

    r = radius
    v = linear_speed
    a = v / r

    aligned_center_frame = center_frame + '_aligned'

    br.sendTransform((x, y, 0),
              tf.transformations.quaternion_from_euler(0, 0, 0),
              rospy.Time.now(),
              aligned_center_frame,
              "world")
    rospy.sleep(5)
    
    start = rospy.Time.now()
    rate = rospy.Rate(20) 
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform("world", center_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        x = trans[0]
        y = trans[1]
        theta = 0
        br.sendTransform((x, y, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                aligned_center_frame,
                "world")        
        
        t = (rospy.Time.now() - start).to_sec()
        theta = a * t
        x = r * math.sin(theta)
        y = - r * math.cos(theta)
        br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     "rmtt_0_target",
                     aligned_center_frame)
        
        theta = a * t + np.pi
        x = r * math.sin(theta)
        y = - r * math.cos(theta)
        br.sendTransform((x, y, 0),
                tf.transformations.quaternion_from_euler(0, 0, theta),
                rospy.Time.now(),
                "rmtt_2_target",
                aligned_center_frame)

        rate.sleep()


if __name__ == '__main__':
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass