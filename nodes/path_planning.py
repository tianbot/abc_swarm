#! /usr/bin/env python
#coding:utf-8						# 中文注释
from hashlib import new
from typing_extensions import Self
import rospy
import numpy as np
import tf
import copy
from scipy.interpolate import interp1d
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, Path

class Turtlebot_core():
    def __init__(self):
        rospy.init_node('Turtlebot_core',anonymous=False)
        self.wps_buf = []
        self.count = 0
        self.i = 1
        self.pub_globalpath = rospy.Publisher('/path', Path, queue_size=1)

        self.pub_globalpath = rospy.Publisher("/global_path", Path, queue_size=1)
        self.point_list = MarkerArray()
        rospy.Subscriber("/clicked_point", PointStamped, self.wpsCallback, queue_size=1)
        self.marker_pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.loop()          

    def wpsCallback(self, data):
        p1 = np.zeros(2)
        p1[0] = data.point.x
        p1[1] = data.point.y
        self.wps_buf.append(p1)
        p1 = Marker()
        p1.header.frame_id = "/world"
        p1.header.stamp = rospy.Time.now()
        p1.ns = "tag"
        p1.id = self.i
        p1.action = Marker.ADD
        p1.type = Marker.CYLINDER
        p1.pose.position.x = data.point.x
        p1.pose.position.y = data.point.y
        p1.pose.position.z = 0.1
        p1.pose.orientation.w = 1.0
        p1.scale.x = 0.05
        p1.scale.y = 0.05
        p1.scale.z = 0.3
        p1.color.r = 1.0
        p1.color.g = 0.0
        p1.color.b = 0.0
        p1.color.a = 1.0

        self.i = self.i + 1
        self.point_list.markers.append(p1)
        self.marker_pub.publish(self.point_list)
        x = self.wps_buf[:,0]
        y = self.wps_buf[:,1]
        t = np.linspace(0, 1, num=len(x))
        f1 = interp1d(t, x, kind='cubic')
        f2 = interp1d(t, y, kind='cubic')
        newt = np.linspace(0, 1, num=100)
        newx = f1(newt)
        newy = f2(newt)
        self.wps_global = np.column_stack((newx, newy))			# 路径点
        self.msg_globalpath = Path()
        self.msg_globalpath.header.stamp = rospy.Time.now()
        self.msg_globalpath.header.frame_id = "map"

        self.msg_globalpath.header.frame_id = "map"

        for i in range(len(self.wps_global)):
            pose = PoseStamped()
            pose.pose.position.x = self.wps_global[i,0]
            pose.pose.position.y = self.wps_global[i,1]
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"			

            self.msg_globalpath.poses.append(copy.deepcopy(pose))

    def listener(self):
        rospy.spin()


    def loop(self):
        rate = rospy.Rate(5)
      
        while not rospy.is_shutdown():
            self.pub_globalpath.publish(self.msg_globalpath)

            rate.sleep()

        
if __name__ == "__main__":	
	try:									# 程序正常运行
		Turtlebot_core()					# 类初始化
	except rospy.ROSInterruptException:		# 程序异常
		rospy.loginfo('Could not start motion control node.')	# 错误日志信息输出



