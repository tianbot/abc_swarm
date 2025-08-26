#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import math

class TurtleFollow:
    def __init__(self):
        # initialize the ros node
        rospy.init_node('turtle_follow_node', anonymous=True)
        
        # wait for turtlesim service
        rospy.wait_for_service('/spawn')
        
        # create a service client to spawn turtle2
        try:
            spawn_service = rospy.ServiceProxy('/spawn', Spawn)
            spawn_service(5.0, 5.0, 0.0, 'turtle2')
            rospy.loginfo('Spawned turtle2 successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to spawn turtle2: %s' % e)
            return
        
        # initialize the turtles position variable
        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()
        
        # subscribe to the location of the two tortoises
        self.turtle1_pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_pose_callback)
        self.turtle2_pose_sub = rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_pose_callback)
        
        # create a publisher to control the turtle2
        self.turtle2_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        
        # set the loop frequency
        self.rate = rospy.Rate(10)  # 10 Hz
        
    def turtle1_pose_callback(self, msg):
        """callback function to get the position of the first turtle"""
        self.turtle1_pose = msg
        
    def turtle2_pose_callback(self, msg):
        """callback function to get the position of the second turtle"""
        self.turtle2_pose = msg
        
    def run(self):
        """run follow logic"""
        while not rospy.is_shutdown():
            # create a speed message
            vel_msg = Twist()
            
            # calculate the distance and angle between the second turtle and the first turtle
            dx = self.turtle1_pose.x - self.turtle2_pose.x
            dy = self.turtle1_pose.y - self.turtle2_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            angle_to_target = math.atan2(dy, dx)
            
            # calculate the difference between the current orientation of the second turtle and the angle of the target
            angle_diff = angle_to_target - self.turtle2_pose.theta
            
            # make sure the angle difference is within the range of [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # follow the logic move towards the first turtle and move at a certain speed
            vel_msg.linear.x = 1.5 * distance     # the linear velocity is proportional to the distance
            vel_msg.angular.z = 6.0 * angle_diff  # the angular velocity is directly proportional to the angular difference
            
            # publish a speed command
            self.turtle2_vel_pub.publish(vel_msg)
            
            # sleep at a set frequency
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # create and run turtle follow nodes
        follower = TurtleFollow()
        follower.run()
    except rospy.ROSInterruptException:
        pass