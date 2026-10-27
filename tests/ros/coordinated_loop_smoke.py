#!/usr/bin/env python3
"""Private mocap three-loop test on 11436; synthetic names, driver/VRPN/UI off.

Physics is accelerated, with one intentionally slower robot. This is interface
and feedback testing, not a new user simulation demo or hardware measurement.
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
    folder=Path(tempfile.mkdtemp(prefix='abc_swarm_loops_ros_'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11436',ROS_IP='127.0.0.1',
                      ROS_HOME=str(folder/'home'),ROS_LOG_DIR=str(folder/'logs'))
    os.environ.pop('ROS_HOSTNAME',None)
    import rospy,rosgraph,rospkg,tf,yaml
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry,Path as RosPath
    from std_msgs.msg import String
    from abc_swarm.coordinated_loop import loop_geometry
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
            if probe.connect_ex(('127.0.0.1',11436))==0:raise RuntimeError('Private port occupied')
        start(['roscore','-p','11436'],'master')
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('Private master unavailable')
            time.sleep(.1)
        rospy.init_node('coordinated_mocap_test',disable_signals=True)
        package=Path(rospkg.RosPack().get_path('abc_swarm'))
        rows={i:[] for i in range(3)};states=[];paths={};poses={};visited=[];violations=[];direct_outputs=[];minimum=100.;max_spread=0
        def command(msg,i):rows[i].append((time.monotonic(),msg.linear.x,msg.angular.z))
        def status(msg):
            nonlocal max_spread
            s=json.loads(msg.data);states.append(s);max_spread=max(max_spread,s.get('progress_spread',0))
            if not visited or visited[-1]!=s['stage']:visited.append(s['stage'])
            if s['stage']=='turning' and not all(r['command'][0]==0 for r in s['robots']):violations.append(s)
            for robot in s['robots']:
                if robot['phase']=='tracking_circle':
                    direct_outputs.append(robot['command']==robot['raw_command'])
        subscribers=[rospy.Subscriber('/loops_test_%02d/cmd_vel'%i,Twist,command,callback_args=i) for i in rows]
        subscribers += [rospy.Subscriber('/abc_swarm_demo/status',String,status)]
        for i in rows:
            subscribers.append(rospy.Subscriber('/abc_swarm_demo/paths/loops_test_%02d'%i,RosPath,
                                               lambda msg,j=i:paths.__setitem__(j,msg)))
        odom={i:rospy.Publisher('/loops_test_%02d/odom'%i,Odometry,queue_size=1) for i in rows}
        broadcaster=tf.TransformBroadcaster()

        def pump(duration,move=True,missing_tf=None,missing_odom=None):
            nonlocal minimum
            end=time.monotonic()+duration
            while time.monotonic()<end:
                stamp=rospy.Time.now()
                for i in rows:
                    v=w=0.
                    if move and rows[i] and time.monotonic()-rows[i][-1][0]<.2:
                        _,v,w=rows[i][-1]
                        if i==2:v*=.8;w*=.8
                    x,y,yaw=poses[i]
                    if abs(w)<1e-9:poses[i]=(x+v*math.cos(yaw)*.075,y+v*math.sin(yaw)*.075,yaw)
                    else:poses[i]=(x+v/w*(math.sin(yaw+w*.075)-math.sin(yaw)),y-v/w*(math.cos(yaw+w*.075)-math.cos(yaw)),angle(yaw+w*.075))
                    x,y,yaw=poses[i]
                    marker_x,marker_y,marker_yaw=x,y,yaw
                    if index==4:
                        marker_x=x+.03*math.cos(yaw)-.01*math.sin(yaw)
                        marker_y=y+.03*math.sin(yaw)+.01*math.cos(yaw)
                        marker_yaw=yaw-.6
                    if i!=missing_tf:broadcaster.sendTransform((marker_x,marker_y,0),tf.transformations.quaternion_from_euler(0,0,marker_yaw),stamp,'loops_test_%02d'%i,'world')
                    msg=Odometry();msg.header.stamp=stamp+rospy.Duration(.03)
                    msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w
                    if index!=4 and i!=missing_odom:odom[i].publish(msg)
                for i,p in poses.items():
                    for j,q in poses.items():
                        if j<i:minimum=min(minimum,math.hypot(p[0]-q[0],p[1]-q[1]))
                time.sleep(.025)

        def zeros():
            for i in rows:
                recent=[r for r in rows[i] if r[0]>time.monotonic()-.25]
                assert recent and all(v==0 and w==0 for _,v,w in recent),(i,recent)

        for index,(shape,size,cw) in enumerate([('circle',1.5,False),('triangle',2.,False),('circle',2.,True),('triangle',1.5,True),('triangle',1.5,False)]):
            origin=(1,2,.3)
            expected,_=loop_geometry(shape,size,size,cw,origin)
            poses=dict(enumerate(expected));p=poses[2];poses[2]=(p[0]+.12,p[1]-.1,p[2]+.25)
            states.clear();paths.clear();visited.clear();violations.clear();direct_outputs.clear();minimum=100;max_spread=0
            for rr in rows.values():rr.clear()
            path=package/('launch/mocap/demo_mocap_three_mini_'+shape+'_tracking.launch')
            args=['roslaunch',str(path),'start_driver:=false','start_vrpn:=false','use_rviz:=false',
                  ('diameter' if shape=='circle' else 'side_length')+':='+str(size),
                  'clockwise:='+str(cw).lower()]
            if shape=='triangle':
                # Triangle origin is now its centroid, explicitly place this test at (1,2).
                args+=['reference:=world','origin_x:=1','origin_y:=2','origin_yaw:=0.3']
            args += ['robot_name_%d:=loops_test_%02d'%(i+1,i) for i in rows]
            if index==0:args+=['avoidance:=orca']
            # Keep this regression's existing 5.5cm completion assertion even
            # though the teaching launch now defaults to a relaxed 10cm capture.
            control=yaml.safe_load((package/'config/point_control.yaml').read_text())
            control.update(position_tolerance=.04,position_hysteresis=.01)
            if index==4:
                control['mocap_calibration']={'loops_test_%02d'%i:{'yaw_offset':.6,'marker_offset':[.03,.01]} for i in rows}
                args+=['avoidance:=orca','velocity_source:=mocap']
            else:
                # These cases inject odometry loss; select that source explicitly.
                # The final calibrated case covers the mocap-only default.
                args+=['velocity_source:=odom']
            profile=folder/('control_%d.yaml'%index);profile.write_text(yaml.safe_dump(control))
            args+=['control_config:='+str(profile)]
            process=start(args,shape+'_'+str(size))
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                pump(.1,move=False)
                assert process.poll() is None,'node exited; inspect '+str(folder)
                if states and len(paths)==3 and states[-1]['stage']=='assembling':break
            assert states[-1]['stage']=='assembling',states[-1]
            assert states[-1]['mode']=='coordinated_tracking'
            frozen=[[(p.pose.position.x,p.pose.position.y) for p in paths[i].poses] for i in rows]
            assert all(paths[i].header.frame_id=='world' for i in rows)
            if index==0:
                poses[0]=(poses[0][0]+.1,poses[0][1],poses[0][2])
            deadline=time.monotonic()+55;injected=False
            while time.monotonic()<deadline:
                pump(.1)
                assert process.poll() is None,'node exited; inspect '+str(folder)
                if index==0 and not injected and states[-1]['stage']=='tracking_circle':
                    pump(.85,move=False,missing_tf=1);zeros()
                    assert states[-1]['state']=='waiting',states[-1]
                    pump(.4,move=False)
                    assert states[-1]['state']=='running',states[-1]
                    pump(.85,move=False,missing_odom=2);zeros()
                    assert states[-1]['state']=='waiting',states[-1]
                    pump(.4,move=False);injected=True
                if states[-1]['state'] in ('completed','fault'):break
            assert states[-1]['state']=='completed',(shape,size,states[-1],poses)
            assert frozen==[[(p.pose.position.x,p.pose.position.y) for p in paths[i].poses] for i in rows]
            assert 'assembling' in visited
            assert not violations,violations
            if shape=='triangle':
                assert visited.count('turning')==3,visited
                assert states[-1]['edge_rounds_completed']==3,states[-1]
            else:
                assert 'tracking_circle' in visited and 'finishing' in visited,visited
                # Each tracker is published directly: a slower robot may drift
                # in phase, rather than silently scaling the other outputs.
                assert direct_outputs and all(direct_outputs),direct_outputs
                assert max_spread>.15,max_spread
            for i,start_pose in enumerate(expected):
                assert math.hypot(poses[i][0]-start_pose[0],poses[i][1]-start_pose[1])<.055,(i,poses[i],start_pose)
                assert abs(angle(poses[i][2]-start_pose[2]))<.09,(i,poses[i],start_pose)
            assert minimum>.55,minimum
            assert all(abs(v)<=.15+1e-8 and abs(w)<=.5+1e-8 for rr in rows.values() for _,v,w in rr)
            pump(.4);zeros()
            print(shape,size,'clockwise',cw,'completed; min distance',round(minimum,3),'max progress spread',round(max_spread,3),flush=True)
            stop(process);time.sleep(.1)
            for rr in rows.values():assert rr[-1][1:]==(0,0)
        print('PASS: mocap coordinated circles/triangles, assembly barriers, slow-robot coordination, vertex turns, frozen geometry, TF/odom stop/recovery, completion and shutdown')
        print('Logs: '+str(folder))
    except Exception:
        for log in folder.glob('*.log'):
            print('\n--- '+log.name+' ---\n'+'\n'.join(log.read_text(errors='replace').splitlines()[-100:]),flush=True)
        raise
    finally:
        for p in reversed(processes):stop(p)
        for stream in streams:stream.close()


if __name__=='__main__':main()
