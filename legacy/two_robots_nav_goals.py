#!/usr/bin/env python  
# -*- coding: UTF-8 -*-
from time import sleep
import roslib
import rospy  
import actionlib
from actionlib_msgs.msg import *  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from geometry_msgs.msg import Pose, Twist


class MultiRobotsNav():  

    def done_cb_1(self, state, result):
        if state == GoalStatus.SUCCEEDED:  
                self.goal_success += 1
                self.goal_1_active = False
                rospy.loginfo("Goal 1 succeeded!{}".format(self.goal_success))


    def done_cb_2(self, state, result):
        if state == GoalStatus.SUCCEEDED:  
                self.goal_success += 1
                self.goal_2_active = False
                rospy.loginfo("Goal 2 succeeded!{}".format(self.goal_success))

    def active_cb_1(self):
        self.goal_1_active = True
        rospy.loginfo_once("Goal 1 accepted")

    def active_cb_2(self):
        self.goal_2_active = True
        rospy.loginfo_once("Goal 2 accepted")

    def feedback_cb_1(self, feedback):  
        rospy.loginfo_once("Feedback received")

    def feedback_cb_2(self, feedback):  
        rospy.loginfo_once("Feedback received")

    def __init__(self):  

        rospy.init_node('multi_robots_nav', anonymous=False)  
        rospy.on_shutdown(self.shutdown)  
        self.r = rospy.Rate(1)
        self.goal_1_active = self.goal_2_active = False
        self.goal_success = 0

        self.goal_1 = MoveBaseGoal()
        self.goal_1.target_pose.header.frame_id = 'world'
        self.goal_1.target_pose.header.stamp = rospy.Time.now()

        self.goal_1.target_pose.pose.orientation.x = 0
        self.goal_1.target_pose.pose.orientation.y = 0
        self.goal_1.target_pose.pose.orientation.z = 0
        self.goal_1.target_pose.pose.orientation.w = 1
        self.goal_1.target_pose.pose.position.x = 0
        self.goal_1.target_pose.pose.position.y = 4
        self.goal_1.target_pose.pose.position.z = 0

        self.goal_2 = MoveBaseGoal()
        self.goal_2.target_pose.header.frame_id = 'world'
        self.goal_2.target_pose.header.stamp = rospy.Time.now()

        self.goal_2.target_pose.pose.orientation.x = 0
        self.goal_2.target_pose.pose.orientation.y = 0
        self.goal_2.target_pose.pose.orientation.z = 0
        self.goal_2.target_pose.pose.orientation.w = 1
        self.goal_2.target_pose.pose.position.x = 0
        self.goal_2.target_pose.pose.position.y = 0
        self.goal_2.target_pose.pose.position.z = 0

        # Subscribe to the move_base action server  
        self.move_base_1 = actionlib.SimpleActionClient("tbmn_01/move_base", MoveBaseAction)
        self.move_base_2 = actionlib.SimpleActionClient("tbmn_02/move_base", MoveBaseAction)

        self.move_base_1.wait_for_server(rospy.Duration(6))  
        self.move_base_2.wait_for_server(rospy.Duration(6))  
        rospy.loginfo("Connected to move base server")          
        rospy.loginfo("Starting navigation test")  

    def execute(self):

        while not rospy.is_shutdown():
            self.r.sleep()
            self.goal_1.target_pose.header.stamp = self.goal_2.target_pose.header.stamp = rospy.Time.now()
            if self.goal_success % 4 == 0 and self.goal_1_active == False and self.goal_2_active == False:
                self.move_base_1.send_goal(self.goal_1,self.done_cb_1, self.active_cb_1, self.feedback_cb_1)      
                self.move_base_2.send_goal(self.goal_2,self.done_cb_2, self.active_cb_2, self.feedback_cb_2) 
            elif self.goal_success % 4 == 2 and self.goal_1_active == False and self.goal_2_active == False:
                rospy.loginfo("Navigation test succeeded")
                self.move_base_1.send_goal(self.goal_2,self.done_cb_1, self.active_cb_1, self.feedback_cb_1)
                self.move_base_2.send_goal(self.goal_1,self.done_cb_2, self.active_cb_2, self.feedback_cb_2) 
                rospy.loginfo("return") 

        rospy.spin()

    def shutdown(self):  

        rospy.loginfo("Stopping the robot...")  

        # Cancel any active goals  

        self.move_base_1.cancel_goal()
        rospy.sleep(2)
        self.move_base_2.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':  

    try:  
        task = MultiRobotsNav() 
        task.execute()

    except rospy.ROSInterruptException:  

        rospy.loginfo("Navigation test finished.")