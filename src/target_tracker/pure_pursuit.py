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
            # Deep copy the updated path to avoid modifying the original
            self.path = copy.deepcopy(self.updated_path)
            self.d = []  # Array to store distances to path points

            try:
                # Get current robot pose from TF (world to robot_name transform)
                (trans, rot) = self.listener.lookupTransform("world", self.robot_name + "/base_link", rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue  # Skip this iteration if TF lookup fails
            rospy.loginfo_once('TF is ready')  # Log once when TF becomes available

            # Extract current position and orientation
            current_x = trans[0]  # Current X coordinate in world frame
            current_y = trans[1]  # Current Y coordinate in world frame
            current_theta = tf.transformations.euler_from_quaternion(rot)[2]  # Yaw angle from quaternion

            # Calculate distances to all points in the path
            for i in range(len(self.path.poses)):
                dx = current_x - self.path.poses[i].pose.position.x
                dy = current_y - self.path.poses[i].pose.position.y
                self.d.append(np.hypot(dx, dy))  # Euclidean distance to path point
                # print("dx: {}, dy: {}".format(dx, dy))
            
            
            if len(self.d):  # If we have valid distances
                # Find nearest point index
                ind = np.argmin(self.d)
                lf_distance = 0  # Accumulated look-ahead distance
                
                # Find look-ahead point (point ahead of nearest point by Lf distance)
                while self.Lf > lf_distance and ind < len(self.path.poses) - 2:
                    # Calculate segment vector between consecutive path points
                    delta_x = self.path.poses[ind+1].pose.position.x - self.path.poses[ind].pose.position.x
                    delta_y = self.path.poses[ind+1].pose.position.y - self.path.poses[ind].pose.position.y 
                    lf_distance += np.hypot(delta_x, delta_y)  # Add segment length
                    if ind > len(self.path.poses):  # Safety check
                        break
                    ind += 1  # Move to next point
            
                # Get target point coordinates
                target_x = self.path.poses[ind].pose.position.x
                target_y = self.path.poses[ind].pose.position.y
                
                # Calculate steering angle (alpha)
                alpha = math.atan2(target_y - current_y, target_x - current_x) - current_theta
                # Normalize angle to [-pi, pi]
                alpha = np.mod(alpha + math.pi, 2*math.pi) - math.pi

                # Create Twist message for velocity commands
                twist = Twist()
                angular_z = 0.5 * alpha  # Simple P controller for steering angle
                
                # publish twist, if the distance is too far, move forward, if too close, stop
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
                rospy.loginfo('linear.x: {}, angular.z: {}'.format(twist.linear.x, twist.angular.z))
        
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
        self.robot_name = rospy.get_param('~robot_name', 'tianbot')
        self.plan_topic_name = rospy.get_param('~plan_topic', 'path')

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