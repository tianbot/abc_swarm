#!/usr/bin/env python

import rospy
import tf
from tf import transformations
from tf import broadcaster

def herringbone_formation_target_tf(target_num, window_size, target_frame='world', follower_frame_prefix='follower', hz=50, target_offset=[5, 5 ,0]):
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

    # queue of target poses, control the offset between formations
    target_pose_bias = []
    median_index = (target_num - 1) // 2  # calculate the median index

    for i in range(target_num):
        if i == median_index:
            # the offset for the median index is (0, 0)
            target_pose_bias.append([0, 0, 0])
        else:
            # calculate the offset: (i - median_index) * 1.0
            offset_x = (i - median_index) * target_offset[0]
            offset_y = (i - median_index) * target_offset[1]
            offset_z = (i - median_index) * target_offset[2]
            target_pose_bias.append([offset_x, offset_y, offset_z])

    def update_leader_trajectory():
        try:
            # get the current tf of the leader
            (trans, rot) = listener.lookupTransform('world', target_frame, rospy.Time(0))
            
            # update the sliding window
            leader_trajectory.append((trans, rot))
            if len(leader_trajectory) > window_size:
                leader_trajectory.pop(0)  # remove the oldest track point
            
            rospy.loginfo("Leader trajectory len: {}".format(len(leader_trajectory)))

            # calc the target points queue
            target_poses.clear()
            for index in target_indexs:
                if index < len(leader_trajectory):
                    target_poses.append(leader_trajectory[index])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))

    def publish_formation_target_poses(event):
        if target_poses.__len__() != 0:
            for i in range(target_num):
                (target_trans, target_rot) = target_poses[0]
                # add the target pose bias
                rospy.loginfo("target_pose_bias: {}".format(target_pose_bias[i]))
                broadcaster.sendTransform(
                    [target_trans[0] + target_pose_bias[i][0], target_trans[1] + target_pose_bias[i][1], target_trans[2]],
                    target_rot,
                    rospy.Time.now(),
                    follower_frame_prefix + '_{}/target'.format(target_num - (i+1)),
                    'world'
                )
                rospy.loginfo("Published {} herringbone targets: {}".format(target_num, follower_frame_prefix))

    
    # update leader trajectory points regularly
    rospy.Timer(rospy.Duration(1 / hz), lambda event: update_leader_trajectory())

    # target point tf are published regularly
    rospy.Timer(rospy.Duration(1 / hz), publish_formation_target_poses)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('herringbone_target_tf_publisher')
    target_num = rospy.get_param('~target_num', default=3)
    window_size = rospy.get_param('~window_size', default=100)
    target_frame = rospy.get_param('~target_frame')
    follower_frame = rospy.get_param('~follower_frame')
    hz = rospy.get_param('~hz', default=50)

    rospy.loginfo("target_num: {}, window_size: {}, target_frame: {}, follower_frame: {}, hz: {}".format(target_num, window_size, target_frame, follower_frame, hz))
    
    # publish the herringbone formation tf
    herringbone_formation_target_tf(
        target_num=3,
        window_size=300,
        target_frame=target_frame,
        follower_frame_prefix=follower_frame,
        target_offset=[0.5, 0.5, 0],
        hz=50
    )
