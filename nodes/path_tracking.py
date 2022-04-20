#!/usr/bin/env python
from importlib.resources import path
import math
import copy
import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist



class PathTracking(): 
    def path_cb(self, path):
        self.updated_path = path

    def pure_pursuit(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.path = copy.deepcopy(self.updated_path)
            self.d = []

            try:
                (trans, rot) = self.listener.lookupTransform("world", self.robot_name+'/base_link', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rospy.loginfo_once('TF is ready')

            current_x = trans[0]
            current_y = trans[1]
            current_theta = tf.transformations.euler_from_quaternion(rot)[2]

            # find nearest points
            for i in range(len(self.path.poses)):
                dx = current_x - self.path.poses[i].pose.position.x
                dy = current_y - self.path.poses[i].pose.position.y
                self.d.append(np.hypot(dx, dy))
            
            if len(self.d):

                ind = np.argmin(self.d)
                lf_distance = 0
                while self.Lf > lf_distance and ind < len(self.path.poses) - 2:
                    delta_x = self.path.poses[ind+1].pose.position.x - self.path.poses[ind].pose.position.x
                    delta_y = self.path.poses[ind+1].pose.position.y - self.path.poses[ind].pose.position.y 
                    lf_distance = lf_distance + np.hypot(delta_x, delta_y)
                    if ind > len(self.path.poses):
                        break
                    ind = ind + 1
            
                target_x = self.path.poses[ind].pose.position.x
                target_y = self.path.poses[ind].pose.position.y
                
                # Calculate twist
                alpha = math.atan2(target_y - current_y, target_x - current_x) - current_theta
                alpha = np.mod(alpha + math.pi, 2*math.pi) - math.pi

                twist = Twist()
                angular_z = 0.5 * alpha
                
                # publish twist
                if lf_distance > self.Lf / 2:
                    twist.linear.x = 0.2
                    if angular_z > 1.0:
                        angular_z = 1.0
                    elif angular_z < -1.0:
                        angular_z = -1.0
                    twist.angular.z = angular_z
                elif lf_distance < self.Lf / 4:
                    twist.linear.x = 0
                    twist.angular.z = 0

                self.cmd_vel_pub.publish(twist)
        
        rate.sleep()
        rospy.spin()

    def __init__(self):  

        rospy.init_node('path_tracking', anonymous=False)  
        # member var
        self.path = Path()
        self.updated_path = Path()
        self.d = []
        self.Lf = 0.3
        # param
        self.robot_name = rospy.get_param('~robot_name', 'tianbot_mini')
        self.plan_topic_name = rospy.get_param('~plan_topic', 'global_plan')

        # subs and pubs
        self.path_sub = rospy.Subscriber(self.plan_topic_name, Path, self.path_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.listener = tf.TransformListener()

if __name__ == '__main__':  

    try:  
        tracker = PathTracking() 
        tracker.pure_pursuit()

    except rospy.ROSInterruptException:  

        rospy.loginfo("Path Tracking finished.")