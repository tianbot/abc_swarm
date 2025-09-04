#!/usr/bin/env python
import math
import rospy
import tf
from tf import transformations
from tf import broadcaster
import numpy as np

def broadcaster():

    listener = tf.TransformListener()
    listener.waitForTransform(center_frame_id, global_frame_id, rospy.Time(0), rospy.Duration(3.0));
    try:
        (trans, rot) = listener.lookupTransform(global_frame_id, center_frame_id, rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    br = tf.TransformBroadcaster()
    x = trans[0]
    y = trans[1]
    theta = 0

    r = radius
    v = linear_speed
    a = v / r

    aligned_center_frame_id = center_frame_id + '_aligned'

    br.sendTransform((x, y, 0),
              tf.transformations.quaternion_from_euler(0, 0, 0),
              rospy.Time.now(),
              aligned_center_frame_id,
              global_frame_id)
    rospy.sleep(2)
    
    start = rospy.Time.now()
    rate = rospy.Rate(20) 
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(global_frame_id, center_frame_id, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        """ 
        \   |
         \  |
          \ |
           \| 
            o-center_frame_id
            |\ 
            | \
            |  \
            |   \
            |    \
            |theta \
        """

        x = trans[0]
        y = trans[1]
        theta = 0
        br.sendTransform((x, y, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                aligned_center_frame_id,
                global_frame_id)        
        
        t = (rospy.Time.now() - start).to_sec()
        theta = a * t
        x = r * math.sin(theta)
        y = - r * math.cos(theta)
        br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     f"{target_frame_id_prefix}_0_target",
                     aligned_center_frame_id)
        theta = a * t + np.pi
        x = r * math.sin(theta)
        y = - r * math.cos(theta)
        br.sendTransform((x, y, 0),
                tf.transformations.quaternion_from_euler(0, 0, theta),
                rospy.Time.now(),
                f"{target_frame_id_prefix}_2_target",
                aligned_center_frame_id)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('virtual_target', anonymous=True)
    
    # read params
    center_frame_id = rospy.get_param('~center_frame_id', 'leader')
    target_frame_id_prefix = rospy.get_param('~target_frame_id_prefix', 'rmtt')
    radius = rospy.get_param('~radius', 0.5)
    global_frame_id = rospy.get_param('~global_frame_id', 'world')
    linear_speed = rospy.get_param('~speed', 0.1)
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass