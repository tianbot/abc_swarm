#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
把 /cmd_vel (odom 坐标系) 转换到 /cmd_vel_body (base_link 坐标系)
依赖: rospy, tf
"""
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
import tf2_geometry_msgs  # 必须显式 import，否则 transform 不会注册

class VelGlobalToBody:
    def __init__(self):

        # TF 监听
        self.tf_buf  = tf2_ros.Buffer()
        self.tf_lsn  = tf2_ros.TransformListener(self.tf_buf)

        # 发布 /cmd_vel_body
        self.pub_body = rospy.Publisher('cmd_vel_body', Twist, queue_size=10)

        # 订阅 /cmd_vel (全局)
        rospy.Subscriber('cmd_vel_global', Twist, self.cb_cmd_vel)

        rospy.loginfo("vel_global_to_body 节点已启动")

    def cb_cmd_vel(self, msg):
        """
        收到全局速度后，转成机体坐标系再发出去
        """
        try:
            # 查找 base_link 相对于 odom 的变换
            trans = self.tf_buf.lookup_transform(
                target_frame=body_frame,
                source_frame=global_frame,
                time=rospy.Time(0),   # 用最新可用变换
                timeout=rospy.Duration(0.2))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "TF 异常: %s" % e)
            return

        # 把全局线速度向量做旋转
        # 这里只旋转线速度，角速度在 2D 下与坐标系无关
        v_global = tf2_geometry_msgs.Vector3Stamped()
        v_global.vector = msg.linear
        v_global.header.frame_id = global_frame
        v_global.header.stamp = trans.header.stamp

        v_body = tf2_geometry_msgs.do_transform_vector3(v_global, trans)

        # 组装新的 Twist
        twist_body = Twist()
        threshold = 0.5
        twist_body.linear.x  = min(max(-threshold, v_body.vector.x), threshold)
        twist_body.linear.y  = min(max(-threshold, v_body.vector.y), threshold)
        twist_body.linear.z  = min(max(-threshold, 1 - trans.transform.translation.z), threshold)

        twist_body.angular   = msg.angular  # 角速度直接复用

        rospy.loginfo("linear.x: {}, linear.y: {}, linear.z: {} angular.z: {}".format(twist_body.linear.x, twist_body.linear.y, twist_body.linear.z, twist_body.angular.z))

        self.pub_body.publish(twist_body)

if __name__ == '__main__':
    rospy.init_node('vel_global_to_body', anonymous=True)
    global_frame = rospy.get_param('~global_frame', "follower_2/base_link")
    body_frame = rospy.get_param('~body_frame', "world")
    try:
        VelGlobalToBody()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
