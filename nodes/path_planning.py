# ! /usr/bin/env python
# coding:utf-8 
import rospy
import copy
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import interp1d			# 差值模块， 对离散的数据点进行差值
from nav_msgs.msg import Odometry, Path
import numpy as np

class PathPlanning():
    def __init__(self):
        rospy.init_node('path_planning', anonymous=False)

        self.marker_pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.pub_globalpath = rospy.Publisher("/global_path", Path, queue_size=1)
        rospy.Subscriber("/clicked_point", PointStamped, self.wpts_cb, queue_size=1)

        self.wpts = np.empty((1,2))
        self.pts_marker = MarkerArray()



        self.i = 1

        self.loop()

    def wpts_cb(self, msg):
        self.wpt = np.zeros((1,2))
        self.wpt[0,0] = msg.point.x
        self.wpt[0,1] = msg.point.y
        self.wpts = np.append(self.wpts, self.wpt, axis = 0)
        self.pt = Marker()
        self.pt.header.frame_id = 'tianbot_mini/map'
        self.pt.header.stamp = rospy.Time.now()
        self.pt.ns = "tag"
        self.pt.id = self.i
        self.pt.action = Marker.ADD
        self.pt.type = Marker.CYLINDER

        self.pt.pose.position.x = msg.point.x
        self.pt.pose.position.y = msg.point.y
        self.pt.pose.position.z = msg.point.z
        
        self.pt.pose.orientation.w = 1.0
        self.pt.scale.x = 0.05
        self.pt.scale.y = 0.05
        self.pt.scale.z = 0.3
		
        self.pt.color.r = 1.0
        self.pt.color.g = 0.0
        self.pt.color.b = 0.0
        self.pt.color.a = 1.0
		
        self.i = self.i + 1

        self.pts_marker.markers.append(self.pt)
        
        rospy.loginfo("Save waypoint ssuccess!!!")

    def loop(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pts_marker:
                self.marker_pub.publish(self.pts_marker)       
            if len(self.wpts) > 4:
                x = self.wpts[:,0]
                y = self.wpts[:,1]
                t = np.linspace(0, 1, num=len(x))
                f1 = interp1d(t,x,kind='cubic')
                f2 = interp1d(t,y,kind='cubic')
                newt = np.linspace(0,1,20*len(self.wpts))
                newx = f1(newt)
                newy = f2(newt)	
                self.wps_global = np.column_stack((newx, newy))
                self.global_path = Path()
                self.global_path = Path()
                self.global_path.header.stamp = rospy.Time.now()
                self.global_path.header.frame_id = 'tianbot_mini/map'

                for i in range(len(self.wps_global)):
                    pose = PoseStamped()
                    pose.pose.position.x = self.wps_global[i,0]
                    pose.pose.position.y = self.wps_global[i,1]
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = 'tianbot_mini/map'	

                    self.global_path.poses.append(copy.deepcopy(pose))
                self.pub_globalpath.publish(self.global_path)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        PathPlanning()
    except rospy.ROSInterruptException:
        rospy.loginfo('Path planning is finished')

