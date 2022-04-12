#!/usr/bin/env python
#coding:utf-8						# 中文注释

import rospy						# ros Python 库
import copy
import math 					# Python 数学模块
import tf
import numpy as np
from scipy.interpolate import interp1d			# 差值模块， 对离散的数据点进行差值
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist			# ros 中定义的消息格式
from scipy.spatial import KDTree			# 使用 Python KDTtree
import scipy.linalg as la				# 用于求解逆矩阵
import matplotlib.pyplot as plt


LOOKAHEAD_WPS = 8					# 预瞄点

# 速度
class Velocity:
	def __init__(self):
		self.v = 0
		self.w = 0

	def update(self, v, w):
		self.v = w
		self.w = w


# 位置
class Pose:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.yaw = 0

	def update(self, x, y, yaw):
		self.x = x
		self.y = y
		self.yaw = yaw

# 移动机器人状态(速度、位置)
class Robot:
	def __init__(self):
		self.vel = Velocity()
		self.pose = Pose()
	
	def update(self, vel, pose):
		self.vel = vel
		self.pose = pose


# Pid 跟踪
class Pid:
	def __init__(self, robot, global_path, local_path):				# 机器人状态，全局路径
		self.robot = robot
		self.global_path = global_path
		self.local_path = local_path

	def calculateTwistCommand(self):
		targetX = self.local_path[-1, 0]			# 预瞄点横坐标
		targetY = self.local_path[-1, 1]			# 预瞄点纵坐标
		
		currentX = self.robot.pose.x				# 当前点横坐标
		currentY = self.robot.pose.y				# 当前点纵坐标
		
		goal_x = self.global_path[-1, 0]			# 目标点横坐标
		goal_y = self.global_path[-1, 1]			# 目标点纵坐标

		alpha = math.atan2(targetY - currentY, targetX - currentX) - self.robot.pose.yaw	# 计算航向误差

		lad = math.hypot(goal_x - currentX, goal_y - currentY)		

		if lad > 0.3:
			self.robot.vel.v = 0.2
			self.robot.vel.w = alpha
			if alpha > 1.0:
				alpha = 1.0
			else:
				alpha = alpha		
		else:
			self.robot.vel.v = 0
			self.robot.vel.w = 0 		

		return self.robot.vel.v, self.robot.vel.w 

# LQR 跟踪
class lqr:
	def __init__(self, robot, global_path, local_path):				# 机器人状态，全局路径
		self.robot = robot
		self.global_path = global_path
		self.local_path = local_path
		self.ref_v = 0.2			# 参考速度
		self.lqr_steering_control(self.robot, 0.05)
	
	def solve_DARE(self, A, B, Q, R):
    		# """
    		# solve a discrete time_Algebraic Riccati equation (DARE)
    		# """
    		X = Q
    		maxiter = 500
    		eps = 0.01
    		Test = A.T * X * A - (A.T * X * B) * la.pinv(R + B.T * X * B) * (B.T * X * A) + Q

    		for i in range(maxiter):
        		Xn = A.T * X * A - (A.T * X * B) * la.pinv(R + B.T * X * B) * (B.T * X * A) + Q
        		if (abs(Xn - X)).max() < eps:
            			X = Xn
            			break
        		else:
            			X = Xn

    			X = Xn

    		return Xn

	def dlqr(self, A, B, Q, R):
		# """
		# Solve the discrete time lqr controller.
		# x[k+1] = A x[k] + B u[k]
		# cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
		# ref Bertsekas, p.151
		# """
		# first, try to solve the ricatti equation
		X = self.solve_DARE(A, B, Q, R)
		# compute the LQR gain
		K = la.pinv(B.T * X * B + R) * (B.T * X * A)
    		return K

	def lqr_steering_control(self, robot, dt):
		x_error = robot.pose.x - self.local_path[0, 0]
		y_error = robot.pose.y - self.local_path[0, 1]

		ref_yaw = math.atan((self.local_path[1, 1] - self.local_path[0, 1]) / (self.local_path[1, 0] - self.local_path[0, 0]))
		yaw_error = robot.pose.yaw - ref_yaw

		x = np.mat(np.zeros((3, 1)))

		x[0, 0] = x_error
		x[1, 0] = y_error
		x[2, 0] = yaw_error
		v = 0.4
		# 由状态方程计算矩阵系数，计算k
		A = np.mat(np.zeros((3, 3)))
		A[0, 0] = 1.0
		A[0, 1] = 0
		A[0, 2] = -robot.vel.v * dt * math.sin(robot.pose.yaw)
		A[1, 0] = 0
		A[1, 1] = 1.0
		A[1, 2] = robot.vel.v * dt * math.cos(robot.pose.yaw)
		A[2, 0] = 0
		A[2, 1] = 0
		A[2, 2] = 1

		B = np.mat(np.zeros((3, 2)))
		B[0, 0] = dt * math.cos(robot.pose.yaw)
		B[0, 1] = 0
		B[1, 0] = dt * math.sin(robot.pose.yaw)
		B[1, 1] = 0
		B[2, 0] = 0
		B[2, 1] = dt

		Q = np.mat(np.zeros((3, 3)))
		Q[0, 0] = 10.0
		Q[0, 1] = 0
		Q[0, 2] = 0
		Q[1, 0] = 0
		Q[1, 1] = 10.0
		Q[1, 2] = 0
		Q[2, 0] = 0
		Q[2, 1] = 0
		Q[2, 2] = 10.0

		R = np.mat(np.zeros((2, 2)))
		R[0, 0] = 5.0
		R[0, 1] = 0.0
		R[1, 0] = 0.0
		R[1, 1] = 5.0
	
		K = self.dlqr(A, B, Q, R)
	
		fb = (-K * x)
		# self.v = 0.1
		# self.w = -yaw_error

		
		targetX = self.local_path[-1, 0]			# 局部路径
		targetY = self.local_path[-1, 1]			# 
		
		currentX = self.robot.pose.x				# 当前点横坐标
		currentY = self.robot.pose.y				# 当前点纵坐标
		
		goal_x = self.global_path[-1, 0]			# 目标点横坐标
		goal_y = self.global_path[-1, 1]			# 目标点纵坐标

		alpha = math.atan2(targetY - currentY, targetX - currentX) - self.robot.pose.yaw	# 计算航向误差

		lad = math.hypot(goal_x - currentX, goal_y - currentY)
			
		# 控制量约束
		if lad > 0.3:
			self.v = self.ref_v + fb[0, 0]
			self.w = fb[1, 0]
		
			if self.v > 0.5:
				self.v = 0.5
			else:
				self.v = self.v	
			if self.w > 0.5:
				self.w = 0.5
			else:
				self.w = alpha		
		else:
			self.v = 0
			self.w = 0 






