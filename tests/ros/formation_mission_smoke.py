#!/usr/bin/env python3
"""Private 11438 ROS-clock models: no hardware, no VRPN, no odom publishers."""
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
    folder=Path(tempfile.mkdtemp(prefix='abc_swarm_mission_ros_'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11438',ROS_IP='127.0.0.1',
                      ROS_HOME=str(folder/'home'),ROS_LOG_DIR=str(folder/'logs'))
    os.environ.pop('ROS_HOSTNAME',None)
    import rospy,rosgraph,rospkg,tf,yaml
    from geometry_msgs.msg import Twist,PoseArray
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from abc_swarm.point_control import angle,world_goals
    from abc_swarm.preset_paths import preset_path
    processes=[];streams=[]

    def start(cmd,name):
        out=(folder/(name+'.log')).open('w');streams.append(out)
        p=subprocess.Popen(cmd,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

    try:
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1',11438))==0:raise RuntimeError('Private port occupied')
        start(['roscore','-p','11438'],'master');end=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>end:raise RuntimeError('Private master not ready')
            time.sleep(.1)
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('mission_smoke',disable_signals=True)
        names=['mission_test_%02d'%i for i in range(1,4)]
        frames=['mission_body_%02d'%i for i in range(1,4)]
        commands={n:[] for n in names};states=[];starts=[];targets=[];visited=[]
        def status(msg):
            s=json.loads(msg.data);states.append(s)
            if not visited or visited[-1]!=s['stage']:visited.append(s['stage'])
        subs=[rospy.Subscriber('/'+n+'/cmd_vel',Twist,lambda m,key=n:commands[key].append((time.monotonic(),m.linear.x,m.angular.z))) for n in names]
        subs += [rospy.Subscriber('/abc_swarm_demo/status',String,status),
                 rospy.Subscriber('/abc_swarm_demo/startup_goals',PoseArray,lambda m:starts.append(m)),
                 rospy.Subscriber('/abc_swarm_demo/goals',PoseArray,lambda m:targets.append(m))]
        clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
        teleop=rospy.Publisher('/abc_swarm_demo/teleop_cmd',Twist,queue_size=1)
        broadcaster=tf.TransformBroadcaster();sim=100.;poses=[];minimum=100.;peak=0.
        package=Path(rospkg.RosPack().get_path('abc_swarm'))
        profile=yaml.safe_load((package/'config/point_stabilization.yaml').read_text())
        profile['mocap_calibration']={n:{'yaw_offset':.6 if i==1 else 0.,'marker_offset':[.03,.01]} for i,n in enumerate(names)}
        control=folder/'control.yaml';control.write_text(yaml.safe_dump(profile))

        def pump(duration,missing=None,request=None,move=True):
            nonlocal sim,minimum,peak
            until=sim+duration
            while sim<until:
                dt=.025
                for i,n in enumerate(names):
                    v=w=0.
                    if move and commands[n] and time.monotonic()-commands[n][-1][0]<.2:
                        _,v,w=commands[n][-1]
                    x,y,yaw=poses[i]
                    if abs(w)<1e-8:poses[i]=(x+v*dt*math.cos(yaw),y+v*dt*math.sin(yaw),yaw)
                    else:poses[i]=(x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),
                                   y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),angle(yaw+w*dt))
                    peak=max(peak,abs(v))
                sim+=dt;clock.publish(Clock(clock=rospy.Time.from_sec(sim)))
                stamp=rospy.Time.from_sec(sim-.02)
                for i,(x,y,yaw) in enumerate(poses):
                    if i==missing:continue
                    raw_yaw=yaw-(.6 if i==1 else 0.)
                    marker=(x+.03*math.cos(yaw)-.01*math.sin(yaw),y+.03*math.sin(yaw)+.01*math.cos(yaw),0)
                    broadcaster.sendTransform(marker,tf.transformations.quaternion_from_euler(math.pi if i==1 else 0,0,raw_yaw),stamp,frames[i],'world')
                if request is not None:
                    msg=Twist();msg.linear.x,msg.angular.z=request;teleop.publish(msg)
                for i,p in enumerate(poses):
                    for q in poses[:i]:minimum=min(minimum,math.hypot(p[0]-q[0],p[1]-q[1]))
                time.sleep(.005)

        def zeros():
            for n in names:
                recent=[r for r in commands[n] if r[0]>time.monotonic()-.18]
                assert recent and all(v==w==0 for _,v,w in recent),(n,recent)

        for shape in ['triangle','snake']:
            for mode in ['manual','auto']:
                poses=[(0,0,0),(-.8,.6,.5),(-.7,-.7,-.3)] if shape=='triangle' else [(0,0,0),(-.8,.1,.4),(-1.4,-.05,-.3)]
                states.clear();starts.clear();targets.clear();visited.clear();minimum=100;peak=0
                for rows in commands.values():rows.clear()
                pump(.5,move=False)
                args=['roslaunch',str(package/('launch/mocap/demo_mocap_three_mini_leader_follower_'+shape+'.launch')),
                      'mode:='+mode,'radius:=0.6','control_config:='+str(control),'use_sim_time:=true',
                      'start_driver:=false','start_vrpn:=false','start_teleop:=false','use_rviz:=false']
                for i,n in enumerate(names):args+=['robot_name_%d:=%s'%(i+1,n),'rigid_body_%d:=%s'%(i+1,frames[i])]
                p=start(args,shape+'_'+mode);wall=time.monotonic()+12
                while not starts and time.monotonic()<wall:
                    pump(.1,request=(.15,.3))  # Requests during admission must not be replayed.
                    assert p.poll() is None,'node exited; inspect '+str(folder)
                assert starts,'no startup targets; inspect '+str(folder)
                frozen=[(x.position.x,x.position.y) for x in starts[-1].poses]
                assert states[-1]['stage']!='following',states[-1]
                wall=time.monotonic()+45
                while states[-1]['stage']!='following' and time.monotonic()<wall:
                    pump(.2)
                    assert states[-1]['state']!='fault',(states[-1],poses)
                assert states[-1]['stage']=='following',states[-1]
                assert 'fine_positioning' in visited and 'ready' in visited,visited
                assert [(x.position.x,x.position.y) for x in starts[-1].poses]==frozen
                publishers=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
                for n in names:assert publishers['/'+n+'/cmd_vel']==['/abc_swarm_demo'],publishers
                assert not any(topic.endswith('/odom') for topic in publishers),publishers
                if mode=='manual':
                    pump(1.)
                    assert abs(commands[names[0]][-1][1])<1e-8,commands[names[0]][-1]
                    pump(2.,request=(.08,.08));assert commands[names[0]][-1][1]>0
                    pump(3.);assert abs(commands[names[0]][-1][1])<1e-8
                    pump(1.,missing=2);assert states[-1]['state']=='waiting',states[-1]
                    pump(1.,missing=2);zeros()  # Observe a full zero-output interval after the timeout.
                    pump(2.);assert states[-1]['stage']=='following' and states[-1]['state']=='running',states[-1]
                    assert abs(commands[names[0]][-1][1])<1e-8
                else:
                    wall=time.monotonic()+70
                    while states[-1]['state'] not in ('completed','fault') and time.monotonic()<wall:
                        pump(.5)
                    assert states[-1]['state']=='completed',states[-1]
                    pump(1.);zeros()
                    assert 'settling' in visited,visited
                    assert all(r['position_error']<=.04 and abs(r['yaw_error'])<=.08 for r in states[-1]['robots']),states[-1]
                assert minimum>.3,minimum
                assert peak<=.15+1e-8,peak
                stop(p);pump(.2,move=False)
                assert all(commands[n][-1][1:]==(0.,0.) for n in names),commands
                print(shape,mode,'PASS stages',visited,'minimum',round(minimum,3),flush=True)
        print('PASS: default admission/refinement/ready/follow handoffs, calibrated mocap-only feedback, unique outputs, early teleop discard, pause/recovery, completion and exit zeros')
        print('Logs:',folder)
    finally:
        for p in reversed(processes):stop(p)
        for stream in streams:stream.close()


if __name__=='__main__':main()
