#!/usr/bin/env python3
"""Private loopback ROS closed-loop test. No drivers, VRPN, GUI or real names."""
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
    folder=Path(tempfile.mkdtemp(prefix='abc_swarm_point_ros_'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11434',ROS_IP='127.0.0.1',
                      ROS_HOME=str(folder/'home'),ROS_LOG_DIR=str(folder/'logs'))
    os.environ.pop('ROS_HOSTNAME',None)
    import rospy,rosgraph,tf,yaml
    from geometry_msgs.msg import Twist,PoseArray
    from std_msgs.msg import String
    from abc_swarm.point_control import angle,world_goals
    from dynamic_reconfigure.client import Client
    processes=[];streams=[]

    def start(command,name):
        stream=(folder/(name+'.log')).open('w');streams.append(stream)
        p=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait()

    try:
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1',11434))==0:raise RuntimeError('Private port already in use')
        start(['roscore','-p','11434'],'master')
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('Private master not ready')
            time.sleep(0.1)
        rospy.init_node('point_smoke',disable_signals=True)
        rows={i:[] for i in range(3)};states=[];goals=[]
        def command(msg,i):rows[i].append((time.monotonic(),msg.linear.x,msg.angular.z))
        subs=[rospy.Subscriber('/point_test_%02d/cmd_vel'%i,Twist,command,callback_args=i) for i in rows]
        subs += [rospy.Subscriber('/abc_swarm_demo/status',String,lambda msg:states.append(json.loads(msg.data))),
                 rospy.Subscriber('/abc_swarm_demo/goals',PoseArray,lambda msg:goals.append(msg))]
        broadcaster=tf.TransformBroadcaster()
        anchor=(1,2,0.4)
        expected=world_goals([[0,0,0],[-0.5,0.5,0],[-0.5,-0.5,0]],'initial_leader',anchor)
        poses={0:anchor,1:(expected[1][0]-0.2,expected[1][1]+0.1,-1),
               2:(expected[2][0]+0.2,expected[2][1]-0.1,2)}

        def pump(duration,missing=None,move=True):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                now=rospy.Time.now()
                for i in rows:
                    x,y,yaw=poses[i]
                    # Accelerated synthetic physics, with real ROS clock/freshness checks.
                    if move and rows[i] and time.monotonic()-rows[i][-1][0]<0.2:
                        _,v,w=rows[i][-1]
                        poses[i]=(x+v*math.cos(yaw)*0.075,y+v*math.sin(yaw)*0.075,angle(yaw+w*0.075))
                    x,y,yaw=poses[i]
                    if i!=missing:
                        broadcaster.sendTransform((x,y,0),tf.transformations.quaternion_from_euler(0,0,yaw),
                                                  now,'point_test_%02d'%i,'world')
                time.sleep(0.025)

        def zeros():
            for i in rows:
                recent=[r for r in rows[i] if r[0]>time.monotonic()-0.25]
                assert recent and all(v==0 and w==0 for _,v,w in recent),(i,recent)

        root=Path(__file__).resolve().parents[2]
        # Prefer the installed package entrypoint when this test is run against an install.
        import rospkg
        package=Path(rospkg.RosPack().get_path('abc_swarm'))
        launch=package/'launch/mocap/demo_mocap_three_mini_point_stabilization.launch'
        args=['roslaunch',str(launch),'start_driver:=false','start_vrpn:=false','use_rviz:=false']
        for i in rows:args += ['robot_name_%d:=point_test_%02d'%(i+1,i)]
        p=start(args,'initial_leader')
        pump(3,move=False)
        assert p.poll() is None,'required node exited; see '+str(folder)
        assert states[-1]['state']=='running',states[-1]
        assert states[-1]['mode']=='point_stabilization'
        client=Client('/abc_swarm_demo',timeout=3)
        config=client.get_configuration(timeout=3)
        assert config['k_distance']==0.9 and config['k_heading']==2.0,config
        config=client.update_configuration({'k_distance':0.8,'k_heading':1.9})
        assert config['k_distance']==0.8 and rospy.get_param('/abc_swarm_demo/k_distance')==0.8
        client.update_configuration({'k_distance':0.9,'k_heading':2.0})

        assert '/abc_swarm_demo/set_enabled' not in dict(rosgraph.Master('/smoke').getSystemState()[2])
        assert goals and len(goals[-1].poses)==3
        for pose,goal in zip(goals[-1].poses,expected):
            assert abs(pose.position.x-goal[0])<1e-6 and abs(pose.position.y-goal[1])<1e-6
        pump(0.9,missing=2)
        assert states[-1]['state']=='waiting',states[-1]
        assert 'point_test_02' in states[-1]['reason']
        zeros()
        frozen=[(p.position.x,p.position.y) for p in goals[-1].poses]
        # Head moves after capture; goals must stay at their original world coordinates.
        poses[0]=(anchor[0]+0.15,anchor[1],anchor[2]+0.2)
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            pump(0.1)
            if states[-1]['state']=='completed':break
        assert states[-1]['state']=='completed',states[-1]
        assert frozen==[(p.position.x,p.position.y) for p in goals[-1].poses]
        for i,goal in enumerate(expected):
            assert math.hypot(poses[i][0]-goal[0],poses[i][1]-goal[1])<=0.045,(i,poses[i],goal)
            assert abs(angle(poses[i][2]-goal[2]))<=0.085,(i,poses[i],goal)
        pump(0.4);zeros()
        assert all(0<=v<=0.15 and abs(w)<=0.5 for rr in rows.values() for _,v,w in rr)
        poses[0]=(anchor[0]+0.2,anchor[1],anchor[2])
        client.update_configuration({'k_distance':1.0,'k_heading':2.2})
        pump(0.4);zeros()  # Dynamic tuning after completion never restarts motion.
        assert states[-1]['state']=='completed'
        client.close()
        stop(p)

        # The explicit snake task freezes a line behind the current first robot.
        snake_expected=world_goals([[0,0,0],[-0.4,0,0],[-0.8,0,0]],'initial_leader',poses[0])
        states.clear();goals.clear()
        p=start(args+['formation:=snake','spacing:=0.4'],'snake_preset')
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            pump(.1,move=False)
            assert p.poll() is None,'snake node exited; see '+str(folder)
            if states and goals and states[-1]['state']=='running':break
        assert p.poll() is None,'snake node exited; see '+str(folder)
        assert states and goals,'snake startup feedback not ready; see '+str(folder)
        assert states[-1]['formation']=='snake' and states[-1]['state']=='running',states[-1]
        for pose,goal in zip(goals[-1].poses,snake_expected):
            assert abs(pose.position.x-goal[0])<1e-6 and abs(pose.position.y-goal[1])<1e-6
        stop(p)

        data=yaml.safe_load((package/'config/point_stabilization.yaml').read_text())
        data.update(target_reference='world',target_poses=[[p[0]+0.2,p[1],p[2]] for p in poses.values()])
        config=folder/'world.yaml';config.write_text(yaml.safe_dump(data))
        states.clear();goals.clear()
        p=start(args+['goal_config:='+str(config)],'world')
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            pump(.1,move=False)
            assert p.poll() is None,'world node exited; see '+str(folder)
            if states and goals and states[-1]['state']=='running':break
        assert p.poll() is None,'required node exited; see '+str(folder)
        assert states and goals,'world startup feedback not ready; see '+str(folder)
        assert states[-1]['state']=='running',states[-1]
        assert abs(goals[-1].poses[0].position.x-data['target_poses'][0][0])<1e-6
        assert any(v>0 or abs(w)>0 for rr in rows.values() for t,v,w in rr if t>time.monotonic()-0.3)
        stop(p)
        time.sleep(0.15)
        for i in rows:assert rows[i][-1][1:]==(0,0),rows[i][-1]

        # A duplicate mapping must fail during initialization, before any motion publisher.
        before=time.monotonic()
        p=start(args+['robot_name_3:=point_test_00'],'invalid_duplicate')
        deadline=time.monotonic()+10
        while p.poll() is None and time.monotonic()<deadline:pump(0.1,move=False)
        assert p.poll() is not None,'invalid duplicate mapping was accepted'
        assert not any(v!=0 or w!=0 for rr in rows.values() for t,v,w in rr if t>before)
        print('PASS: point goals frozen, automatic closed-loop pose convergence, all-stop on stale TF, recovery, world targets, completion hold, shutdown zeros, snake task selection, dynamic tuning without completion restart and invalid mapping rejection')
        print('Logs: '+str(folder))
    finally:
        for p in reversed(processes):stop(p)
        for stream in streams:stream.close()


if __name__=='__main__':main()
