#!/usr/bin/env python
import math
import rospy
import tf

def broadcaster():
    br = tf.TransformBroadcaster()
    x = 0
    y = 0
    theta = 0
    r = radius    # radius
    v = linear_speed  # linear velocity
    a = v / r  # angular velocity

    br.sendTransform((x, y, 0),
              tf.transformations.quaternion_from_euler(0, 0, theta),
              rospy.Time.now(),
              leader_frame_id,
              global_frame_id)
    rospy.sleep(1)
    
    start = rospy.Time.now()
    rate = rospy.Rate(20) 
    while not rospy.is_shutdown():
        #  here is the code to calculate the circular motion trajectory
        #  the coordinates of the center of the circle are (0，r/2)
        #  linear: v, angular: a, delta: t
        #  the coordinates of the current moment are calculated by the trajectory equation and then broadcast out
        t = (rospy.Time.now() - start).to_sec()         # time step
        theta = a * t                                   # angle
        x = r * math.sin(theta)
        y = r - r * math.cos(theta)
        br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     leader_frame_id,
                     global_frame_id)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('virtual_leader', anonymous=True)
    global_frame_id = rospy.get_param('~global_frame_id', 'world')
    leader_frame_id = rospy.get_param('~leader_frame_id', 'leader')
    radius = rospy.get_param('~radius', 0.5)
    linear_speed = rospy.get_param('~speed', 0.1)
    try:

        broadcaster()
    except rospy.ROSInterruptException:
        pass