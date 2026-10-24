#!/usr/bin/env python3
"""Mocap/odom synthetic ROS contract test on private loopback master 11435.

No hardware/VRPN/UI processes or real names. Synthetic physics is accelerated;
this checks ROS composition/stop behavior, not measured hardware performance.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    folder=Path(tempfile.mkdtemp(prefix='abc_swarm_orca_ros_'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11435',ROS_IP='127.0.0.1',
                      ROS_HOME=str(folder/'home'),ROS_LOG_DIR=str(folder/'logs'))
    os.environ.pop('ROS_HOSTNAME',None)
    import rospy,rosgraph,rospkg,tf,yaml
    from geometry_msgs.msg import Twist,PoseArray
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from abc_swarm.orca import front_point
    from abc_swarm.point_control import angle
    processes=[];streams=[]

    def start(args,name):
        stream=(folder/(name+'.log')).open('w');streams.append(stream)
        p=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

    try:
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1',11435))==0:raise RuntimeError('Private test port occupied')
        start(['roscore','-p','11435'],'master')
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('Private master unavailable')
            time.sleep(.1)
        rospy.init_node('orca_mocap_test',disable_signals=True)
        package=Path(rospkg.RosPack().get_path('abc_swarm'))
        default_goals=yaml.safe_load((package/'config/point_stabilization.yaml').read_text())
        default_orca=yaml.safe_load((package/'config/orca.yaml').read_text())
        launch=package/'launch/mocap/demo_mocap_three_mini_orca_point_stabilization.launch'
        rows={i:[] for i in range(3)};states=[];goal_msgs=[]
        def command(msg,i):rows[i].append((time.monotonic(),msg.linear.x,msg.angular.z))
        subs=[rospy.Subscriber('/orca_test_%02d/cmd_vel'%i,Twist,command,callback_args=i) for i in rows]
        subs += [rospy.Subscriber('/abc_swarm_demo/status',String,lambda m:states.append(json.loads(m.data))),
                 rospy.Subscriber('/abc_swarm_demo/goals',PoseArray,lambda m:goal_msgs.append(m))]
        odom={i:rospy.Publisher('/orca_test_%02d/odom'%i,Odometry,queue_size=1) for i in rows}
        broadcaster=tf.TransformBroadcaster();poses={};minimum=100.0

        def pump(duration,move=True,missing_tf=None,missing_odom=None):
            nonlocal minimum
            end=time.monotonic()+duration
            while time.monotonic()<end:
                stamp=rospy.Time.now()
                for i in rows:
                    v=w=0.0
                    if move and rows[i] and time.monotonic()-rows[i][-1][0]<.2:
                        _,v,w=rows[i][-1]
                    x,y,yaw=poses[i]
                    if abs(w)<1e-9:poses[i]=(x+v*math.cos(yaw)*.05,y+v*math.sin(yaw)*.05,yaw)
                    else:poses[i]=(x+v/w*(math.sin(yaw+w*.05)-math.sin(yaw)),y-v/w*(math.cos(yaw+w*.05)-math.cos(yaw)),angle(yaw+w*.05))
                    x,y,yaw=poses[i]
                    if i!=missing_tf:broadcaster.sendTransform((x,y,0),tf.transformations.quaternion_from_euler(0,0,yaw),stamp,'orca_test_%02d'%i,'world')
                    msg=Odometry();msg.header.stamp=stamp+rospy.Duration(.03)
                    msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w
                    if i!=missing_odom:odom[i].publish(msg)
                for i,p in poses.items():
                    for j,q in poses.items():
                        if j>=i:continue
                        distance=math.hypot(p[0]-q[0],p[1]-q[1]);minimum=min(minimum,distance)
                        if move:
                            assert distance>.30,(poses,distance)
                time.sleep(.025)

        def zeros():
            for i in rows:
                recent=[r for r in rows[i] if r[0]>time.monotonic()-.25]
                assert recent and all(v==0 and w==0 for _,v,w in recent),(i,recent)

        def launch_case(name,initial,targets,orca_override=None):
            nonlocal poses,minimum
            poses=dict(enumerate(initial));minimum=100
            states.clear();goal_msgs.clear()
            for rr in rows.values():rr.clear()
            config=dict(default_goals,target_reference='world',target_poses=targets)
            goal_file=folder/(name+'_goals.yaml');goal_file.write_text(yaml.safe_dump(config))
            settings=dict(default_orca,**(orca_override or {}))
            orca_file=folder/(name+'_orca.yaml');orca_file.write_text(yaml.safe_dump(settings))
            args=['roslaunch',str(launch),'start_driver:=false','start_vrpn:=false','use_rviz:=false',
                  'goal_config:='+str(goal_file),'orca_config:='+str(orca_file)]
            args += ['robot_name_%d:=orca_test_%02d'%(i+1,i) for i in rows]
            p=start(args,name)
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                pump(.1,move=False)
                assert p.poll() is None,'node exited; inspect '+str(folder)
                if states and goal_msgs and states[-1]['state'] in ('running','fault'):return p
            raise AssertionError('Controller not ready: '+str(states[-1:] ))

        head=[(-1,0,0),(1,0,math.pi),(0,1.3,0)]
        head_goals=[(1,0,0),(-1,0,math.pi),(0,1.3,0)]
        cases=[('head_on',head,head_goals),
               ('cross',[(-1,0,0),(0,-1,math.pi/2),(1.5,1.5,0)],[(1,0,0),(0,1,math.pi/2),(1.5,1.5,0)]),
               ('swap_three',[(-1,0,0),(1,0,math.pi),(0,1,-math.pi/2)],[(1,0,0),(0,1,math.pi),(-1,0,-math.pi/2)])]
        for name,initial,targets in cases:
            p=launch_case(name,initial,targets)
            assert states[-1]['avoidance']=='orca' and states[-1]['state']=='running',states[-1]
            frozen=[(p.position.x,p.position.y) for p in goal_msgs[-1].poses]
            if name=='head_on':
                pump(.9,move=False,missing_odom=2);zeros()
                assert states[-1]['state']=='waiting' and 'odom' in states[-1]['reason'],states[-1]
                pump(.4,move=False)
                assert states[-1]['state']=='running',states[-1]
                pump(.9,move=False,missing_tf=1);zeros()
                assert states[-1]['state']=='waiting' and 'pose' in states[-1]['reason'],states[-1]
                pump(.4,move=False)
            deadline=time.monotonic()+40
            while time.monotonic()<deadline:
                pump(.1)
                if states[-1]['state'] in ('completed','fault'):break
            assert states[-1]['state']=='completed',(name,states[-1],poses)
            assert frozen==[(p.position.x,p.position.y) for p in goal_msgs[-1].poses]
            for i,g in enumerate(targets):
                assert math.hypot(poses[i][0]-g[0],poses[i][1]-g[1])<=.045
                assert abs(angle(poses[i][2]-g[2]))<=.085
            assert all(abs(v)<=.15+1e-8 and abs(w)<=.5+1e-8 for rr in rows.values() for _,v,w in rr)
            pump(.4);zeros()
            print(name,'completed; minimum body-center distance',round(minimum,4),flush=True)
            stop(p);time.sleep(.1)
            for rr in rows.values():assert rr[-1][1:]==(0,0)

        p=launch_case('conflicting_targets',head,[(0,0,0),(0,0,0),(0,1.3,0)])
        pump(.3,move=False);zeros()
        assert states[-1]['state']=='fault' and 'target' in states[-1]['reason'],states[-1]
        pump(.4,move=False);zeros();stop(p)
        p=launch_case('overlap',[(0,0,0),(.1,0,0),(0,1.3,0)],head_goals)
        pump(.3,move=False);zeros()
        assert states[-1]['state']=='fault' and 'overlap' in states[-1]['reason'],states[-1]
        stop(p)
        p=launch_case('stall',head,head_goals,{'stall_timeout':.6})
        pump(1,move=False);zeros()
        assert states[-1]['state']=='fault' and 'timeout' in states[-1]['reason'],states[-1]
        stop(p)
        print('PASS: mocap-only point ORCA, measured odom, three encounter tasks, shared TF/odom stop/recovery, frozen goals, pose completion, fault latches and shutdown zeros')
        print('Logs: '+str(folder))
    finally:
        for p in reversed(processes):stop(p)
        for stream in streams:stream.close()


if __name__=='__main__':main()
