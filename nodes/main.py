#!/usr/bin/env python
#coding:utf-8						# 中文注释

import rospy						# ros Python 库
import copy
import math						# Python 数学模块
import tf
import numpy as np
from scipy.interpolate import interp1d			# 差值模块， 对离散的数据点进行差值
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist			# ros 中定义的消息格式
from scipy.spatial import KDTree			# 使用 Python KDTtree
import matplotlib.pyplot as plt
import robot

LOOKAHEAD_WPS = 10					# 预瞄点



# 节点定义
class Pathtrace:
	def __init__(self):
		rospy.init_node('path_trace', log_level=rospy.DEBUG)
				
		self.pub_globalpath = rospy.Publisher("/global_path", Path, queue_size=1)
		self.pub_localpath = rospy.Publisher("/local_path", Path, queue_size=1)		
		self.pub_velcmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		rospy.Subscriber('/odom', Odometry, self.pose_cb)		# 获取机器人位置

		self.count = 0
		self.wps = None

		self.msg_globalpath = None					# 全局路径
		self.wps_global = None 						# 全局路径点						
		
		self.msg_localpath = None					# 局部路径
		self.wps_local = None

		self.currentpose = None						# 机器人当前位置

		self.robot = None						# 机器人当前状态
		
		self.waypoint_tree = None					# KDTree 

		self.point2path()
		self.loop()							# 循环函数

	def point2path(self):
		self.wps = np.loadtxt("/home/tianbot/catkin_air/src/pathtracking/scripts/wps.txt")
		# print(len(self.wps)) # 查看路径点个数
		# self.wps_global = np.array([1.0, 1.0])		
		
		x = self.wps[:,0]
		y = self.wps[:,1]
		t = np.linspace(0, 1, num=len(x))
		f1 = interp1d(t,x,kind='cubic')
		f2 = interp1d(t,y,kind='cubic')
		newt = np.linspace(0,1,100)
		newx = f1(newt)
		newy = f2(newt)				

		self.wps_global = np.column_stack((newx, newy))			# 路径点
		self.waypoint_tree = KDTree(self.wps_global)
		# plt.plot(newx, newy)
		# plt.show()
		# print(newx)
		# print(self.wps_global)
		# 路径显示
		# plt.scatter(x, y)
		# plt.plot(newx, newy)
		# plt.show()				

		self.msg_globalpath = Path()
		self.msg_globalpath.header.stamp = rospy.Time.now()
		self.msg_globalpath.header.frame_id = "map"

		for i in range(len(self.wps_global)):
			pose = PoseStamped()
			pose.pose.position.x = self.wps_global[i,0]
			pose.pose.position.y = self.wps_global[i,1]
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "map"			

			self.msg_globalpath.poses.append(copy.deepcopy(pose))
		# self.pub_globalpath.publish(self.msg_globalpath)
			# pass			
			# print(i) 	# 查看路径点
			# print('ma')

	def pose_cb(self, msg):
		self.currentpose = msg
		vel = robot.Velocity()
		vel.update(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
		quanternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)								# 四元数
		euler = tf.transformations.euler_from_quaternion(quanternion)	# 四元数到欧拉角变换
		yaw = euler[2]
				
		pose = robot.Pose()
		pose.update(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
		
		self.robot = robot.Robot()
		self.robot.update(vel, pose)

	def get_closest_waypoint_idx(self):		# 获取路径上最近点
		x = self.currentpose.pose.pose.position.x
		y = self.currentpose.pose.pose.position.y

		closest_idx = self.waypoint_tree.query([x,y],1)[1]
		# print(closest_idx)	

		return closest_idx

	def publish_localpath(self, closest_idx):
		self.wps_local = self.wps_global[closest_idx:closest_idx + LOOKAHEAD_WPS]
		self.msg_localpath = Path()
		self.msg_localpath.header.stamp = rospy.Time.now()		
		self.msg_localpath.header.frame_id = 'map'

		for i in range(len(self.wps_local)):
			pose = PoseStamped()
			pose.pose.position.x = self.wps_local[i,0]
			pose.pose.position.y = self.wps_local[i,1]
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "map"			
			
			self.msg_localpath.poses.append(copy.deepcopy(pose))
			# print(i)
		self.pub_localpath.publish(self.msg_localpath)
		# rospy.loginfo(self.msg_localpath)

	# 坐标系变换
	def trans(self):
		o_x = 0
		o_y = 0
		o_z = 0
		o_w = 1

		x = 0
		y = 0
		z = 0

		br = tf.TransformBroadcaster()
		br.sendTransform((x, y, z), (o_x, o_y, o_z, o_w), rospy.Time.now(), 'odom', 'world')

	def loop(self):								# 循环函数
		rate = rospy.Rate(10)						# 设置频率
		# print(self.msg_globalpath)		
		while not rospy.is_shutdown():
			# rospy.loginfo('-----')
			self.count = self.count + 1
			self.pub_globalpath.publish(self.msg_globalpath)
			self.trans()
			if self.currentpose and self.waypoint_tree and self.msg_globalpath:
				# rospy.loginfo('-----')
				closest_waypoint_idx = self.get_closest_waypoint_idx() 
				# rospy.loginfo(closest_waypoint_idx)
				self.publish_localpath(closest_waypoint_idx)
				# print(closest_waypoint_idx)
				# 纯跟踪
				# pid = robot.Pid(self.robot, self.wps_global, self.wps_local)
				# twistcmd = Twist()
				# twistcmd.linear.x, twistcmd.angular.z = pid.calculateTwistCommand()
				# LQR				
				lqr = robot.lqr(self.robot, self.wps_global, self.wps_local)
				twistcmd = Twist()
				twistcmd.linear.x, twistcmd.angular.z = lqr.v, lqr.w
				self.pub_velcmd.publish(twistcmd)
				self.pub_velcmd.publish(twistcmd)
			#if (self.count > 15):
			# 	rospy.on_shutdown()
			rate.sleep()


if __name__ == '__main__':				# 主函数
	try:						# 程序正常运行
		Pathtrace()				# 类初始化
	except rospy.ROSInterruptException:		# 程序异常
		rospy.loginfo('Could not start motion control node.')	# 错误日志信息输出
