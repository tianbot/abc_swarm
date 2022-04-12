#! /usr/bin/env python
#coding:utf-8						# 中文注释
import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class Turtlebot_core():
	def __init__(self):
		rospy.init_node("Turtlebot_core", anonymous=True)
		self.wps_buf = []
		self.count = 0
		self.i = 1

		rospy.Subscriber("/clicked_point", PointStamped, self.wpsCallback, queue_size=1)
		self.map_pub = rospy.Publisher("local_map", OccupancyGrid, queue_size=1)
		self.marker_pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)	

		self.local_map = OccupancyGrid()						# 定义地图
		self.point_list = MarkerArray()

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

		np.savetxt("/home/tianbot/catkin_air/src/pathtracking/scripts/wps.txt", np.array(self.wps_buf))
		print("save way point success!!!")
		print("p:"+str(p1))

	def DataUpdating(self):
		# 时间戳		
		current_time = rospy.Time.now()
		
		self.local_map.info.map_load_time = current_time
		self.local_map.header.frame_id = "map"
		self.local_map.info.resolution = 0.01					# 分辨率 1cm
		self.local_map.info.height = 500					# 长和宽
		self.local_map.info.width = 500
		self.local_map.info.origin.position.x = 0				# 原点位置
		self.local_map.info.origin.position.y = 0
		self.local_map.info.origin.position.z = 0
		self.local_map.info.origin.orientation.x = 0			# 原点姿态
		self.local_map.info.origin.orientation.y = 0
		self.local_map.info.origin.orientation.z = 0
		self.local_map.info.origin.orientation.w = 1

		# 默认全部未知区域
		self.local_map.data = [-1] * self.local_map.info.width * self.local_map.info.height

		# 设置障碍区域
		boundary = 20
		for x in [boundary, self.local_map.info.height - boundary]:
			for y in range(boundary, self.local_map.info.width - boundary + 1):
				self.local_map.data[x * self.local_map.info.width + y] = 100

		for y in [boundary, self.local_map.info.height - boundary]:
			for y in range(boundary, self.local_map.info.width - boundary + 1):
				self.local_map.data[x * self.local_map.info.width + y] = 100

		# 设置可行区域
		for x in range(boundary + 1, self.local_map.info.height - boundary):
			for y in range(boundary + 1, self.local_map.info.width - boundary):
				self.local_map.data[x * self.local_map.info.width + y] = 0

		# 发布地图
		self.map_pub.publish(self.local_map)

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
		br.sendTransform((x, y, z), (o_x, o_y, o_z, o_w), rospy.Time.now(), 'map', 'world')


	def loop(self):
		rate = rospy.Rate(5)
		
		
		while not rospy.is_shutdown():
			# 数据更新函数
			self.DataUpdating()
			self.trans()
			self.count = self.count + 1
			#while (self.count > 10):
			#	rospy.on_shutdown()
			rate.sleep()

        
if __name__ == "__main__":	
	try:									# 程序正常运行
		Turtlebot_core()					# 类初始化
	except rospy.ROSInterruptException:		# 程序异常
		rospy.loginfo('Could not start motion control node.')	# 错误日志信息输出



