#!/usr/bin/env python

import rospy
import tf
import numpy as np
from tf import transformations
from tf import broadcaster

def long_snake_target_tf(target_num, window_size, target_frame='world', follower_frame_prefix='follower', hz=50):
    # tf listeners and broadcasters
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    
    # the sliding window stores the trajectory of leader
    leader_trajectory = []
    
    # calculate the amount of target indexs
    num_targets = round(1 * target_num)
    target_indexs = [round(i / target_num * window_size) for i in range(1, num_targets + 1)]
    
    # queue of target poses
    target_poses = []
    last_target_pose = [0, 0, 0]
    
    def update_leader_trajectory():
        try:
            # get the current tf of the leader
            (trans, rot) = listener.lookupTransform('world', target_frame, rospy.Time(0))
            nonlocal last_target_pose
            # update the sliding window if the leader pose is updated, not equal to the last one, 
            # 1e-01 is the tolerance of the distance between two poses
            if not np.allclose(last_target_pose, trans, atol=1e-01):
                # rospy.loginfo("Leader pose updated: {}".format(trans))
                leader_trajectory.append((trans, rot))
                last_target_pose = trans
            if len(leader_trajectory) > window_size:
                leader_trajectory.pop(0)  # remove the oldest track point
            else:
                rospy.loginfo("Leader trajectory len: {}".format(len(leader_trajectory)))

            # calc the target points queue
            target_poses.clear()
            for index in target_indexs:
                if index < len(leader_trajectory):
                    target_poses.append(leader_trajectory[index])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))
    
    def publish_target_poses(event):
        if not target_poses:
            return
        
        for i, (target_trans, target_rot) in enumerate(target_poses):
            # publish tf of the each target
            broadcaster.sendTransform(
                target_trans,
                target_rot,
                rospy.Time.now(),
                follower_frame_prefix + '_{}/target'.format(target_num - (i+1)),
                'world'
            )
            # rospy.loginfo("target_trans: {}, target_rot: {}".format(target_trans, target_rot))
        
        rospy.loginfo("Published {} targets TFs for followers: {}".format(target_num, follower_frame_prefix))
    
    # update leader trajectory points regularly
    rospy.Timer(rospy.Duration(1 / hz), lambda event: update_leader_trajectory())
    
    # target point tf are published regularly
    rospy.Timer(rospy.Duration(1 / hz), publish_target_poses)
    
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('long_snake_target_tf_publisher')
    target_num = rospy.get_param('~target_num', default=3)
    window_size = rospy.get_param('~window_size', default=100)
    target_frame = rospy.get_param('~target_frame')
    follower_frame = rospy.get_param('~follower_frame')
    hz = rospy.get_param('~hz', default=50)

    rospy.loginfo("target_num: {}, window_size: {}, target_frame: {}, follower_frame: {}, hz: {}".format(target_num, window_size, target_frame, follower_frame, hz))
    
    # publish the long snake formation tf
    long_snake_target_tf(
        target_num=target_num,
        window_size=window_size,
        target_frame=target_frame,
        follower_frame_prefix=follower_frame,
        hz=hz
    )
