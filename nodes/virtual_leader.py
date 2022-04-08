#!/usr/bin/env python
import math
import rospy
import tf

def broadcaster():
    br = tf.TransformBroadcaster()
    x = 0
    y = 0
    theta = 0
    r = 0.5
    v = 0.1
    a = v / r
    rospy.init_node('virtual_leader', anonymous=True)

    br.sendTransform((x, y, 0),
              tf.transformations.quaternion_from_euler(0, 0, theta),
              rospy.Time.now(),
              "virtual_leader",
              "world")
    rospy.sleep(5)
    
    start = rospy.Time.now()
    rate = rospy.Rate(20) 
    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start).to_sec()
        theta = a * t
        x = r * math.sin(theta)
        y = 0.5 - r * math.cos(theta)
        br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     "virtual_leader",
                     "world")
        rate.sleep()


if __name__ == '__main__':
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass