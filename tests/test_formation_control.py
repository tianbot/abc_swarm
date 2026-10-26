#!/usr/bin/env python3
import math
import unittest
from unittest.mock import patch

from abc_swarm.admission import AdmissionController, DEFAULT_ADMISSION
from abc_swarm.formation_control import LeaderFollowerMission
from abc_swarm.orca import OrcaBlocked, OrcaInfeasible
from abc_swarm.point_control import angle,world_goals

CONTROL=dict(max_linear=.15,max_angular=.6,position_tolerance=.04,yaw_tolerance=.08,k_distance=.9,k_heading=2.)
ORCA=dict(robot_radius=.13,safety_margin=.02,front_distance=.08,time_horizon=3.,right_bias=.01,velocity_buffer=.002)


def options(formation='triangle',mode='manual',count=3):
    names=['r%d'%i for i in range(count)]
    edges=([dict(leader=names[0],follower=names[i],x=-.5,y=.5 if i==1 else -.5) for i in range(1,count)]
           if formation=='triangle' else [dict(leader=names[i-1],follower=names[i],x=-.5,y=0.) for i in range(1,count)])
    return names,dict(mode=mode,edges=edges,shape='circle',radius=.75,width=2.,height=2.,laps=1,clockwise=False,
                      front_distance=.1,k_x=.5,k_y=.5,lookahead=.25,max_error=.5,tracking_window=.15)


def integrate(pose,command,dt=.05):
    x,y,yaw=pose;v,w=command[:2]
    if abs(w)<1e-10:return (x+v*dt*math.cos(yaw),y+v*dt*math.sin(yaw),yaw)
    return (x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),
            y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),angle(yaw+w*dt))


class AdmissionTests(unittest.TestCase):
    def test_temporary_infeasibility_stops_and_retries_without_relaxing_planes(self):
        a=AdmissionController();goals=[(0,0,0),(.7,0,0)];poses=[(-.2,0,0),(.9,0,0)]
        with patch('abc_swarm.admission.orca_velocities',side_effect=OrcaInfeasible('blocked')):
            commands=a.step(poses,goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertTrue(a.blocked);self.assertTrue(all(v==w==0 for v,w,_ in commands))
        a.step(poses,goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertFalse(a.blocked)
    def test_distance_alone_cannot_disable_orca_and_capture_guard_can_restore_it(self):
        a=AdmissionController();goals=[(0,0,0),(.5,0,0),(1.,0,0)]
        poses=[(x+.14,y,yaw) for x,y,yaw in goals]
        for _ in range(12):a.step(poses,goals,CONTROL,velocities=[(0,0)]*3,orca=ORCA)
        self.assertEqual(a.stage,'assembling_orca')  # Close, but capture disks intersect.
        poses=[(x+.01,y,yaw) for x,y,yaw in goals]
        for _ in range(8):a.step(poses,goals,CONTROL,velocities=[(0,0)]*3,orca=ORCA)
        self.assertEqual(a.stage,'fine_positioning')
        shifted=[(x+.12,y,yaw) for x,y,yaw in goals]
        a.step(shifted,goals,CONTROL,velocities=[(0,0)]*3,orca=ORCA)
        self.assertEqual(a.stage,'assembling_orca')
        self.assertFalse(a.ready)

    def test_final_pose_and_measured_stop_are_required_before_ready(self):
        a=AdmissionController();goals=[(0,0,0),(.7,0,0)]
        for _ in range(20):a.step(goals,goals,CONTROL,velocities=[(.03,0)]*2,orca=ORCA)
        self.assertEqual(a.stage,'fine_positioning');self.assertFalse(a.ready)
        for _ in range(12):commands=a.step(goals,goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertTrue(a.ready);self.assertTrue(all(v==w==0 for v,w,_ in commands))

    def test_invalid_feedback_settings_and_changed_goals_fail(self):
        for key in DEFAULT_ADMISSION:
            cfg=dict(DEFAULT_ADMISSION);cfg[key]=float('nan')
            with self.assertRaises(ValueError):AdmissionController(cfg)
        a=AdmissionController()
        with self.assertRaises(ValueError):a.step([(0,0,0)],[(1,0,0)],CONTROL,orca=ORCA)
        a=AdmissionController();a.step([(0,0,0)],[(1,0,0)],CONTROL)
        with self.assertRaises(ValueError):a.step([(0,0,0)],[(2,0,0)],CONTROL)
        with self.assertRaises(OrcaBlocked):
            AdmissionController().step([(0,0,0),(.2,0,0)],[(0,0,0),(.2,0,0)],CONTROL,velocities=[(0,0)]*2,orca=ORCA)

    def test_feedback_pause_requires_a_new_continuous_settle_window(self):
        a=AdmissionController();goals=[(0,0,0),(.7,0,0)]
        for _ in range(30):a.step(goals,goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertTrue(a.ready)
        a.pause();self.assertFalse(a.ready)
        for _ in range(5):a.step(goals,goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertFalse(a.ready)
        for _ in range(7):a.step(goals,goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertTrue(a.ready)


class FormationMissionTests(unittest.TestCase):
    def test_manual_and_auto_admit_precisely_then_follow_and_auto_completes(self):
        for shape in ['triangle','snake']:
            for mode in ['manual','auto']:
                with self.subTest(shape=shape,mode=mode):
                    names,opt=options(shape,mode);m=LeaderFollowerMission(names,opt)
                    poses=[(0,0,0),(-.8,.6,.5),(-.7,-.7,-.3)] if shape=='triangle' else [(0,0,0),(-.8,.1,.4),(-1.4,-.05,-.3)]
                    velocities=[(0,0)]*3;visited=[];frozen=None;following_steps=0
                    for _ in range(4000):
                        commands=m.step(poses,velocities,CONTROL,ORCA,teleop=(.1,.1))
                        if frozen is None:frozen=list(m.startup_goals)
                        self.assertEqual(frozen,m.startup_goals)
                        if commands[0][2].startswith('leader_'):
                            for leader,follower,x,y in m.edges:
                                self.assertEqual(m.goals[follower],world_goals([(x,y,0)],'initial_leader',poses[leader])[0])
                        if not visited or visited[-1]!=m.stage:visited.append(m.stage)
                        if m.stage=='ready':
                            self.assertIsNone(m.admission.planner.active)
                            for p,g in zip(poses,m.goals):
                                self.assertLessEqual(math.hypot(p[0]-g[0],p[1]-g[1]),CONTROL['position_tolerance'])
                                self.assertLessEqual(abs(angle(p[2]-g[2])),CONTROL['yaw_tolerance'])
                        self.assertTrue(all(abs(v)<=.15+1e-8 and abs(w)<=.6+1e-8 for v,w,_ in commands))
                        distance=min(math.hypot(p[0]-q[0],p[1]-q[1]) for i,p in enumerate(poses) for q in poses[:i])
                        self.assertGreater(distance,.3)
                        poses=[integrate(p,c) for p,c in zip(poses,commands)];velocities=[c[:2] for c in commands]
                        if m.stage=='following':following_steps+=1
                        if mode=='manual' and following_steps>250:break
                        if m.completed:break
                    self.assertIn('assembling_orca',visited);self.assertIn('fine_positioning',visited)
                    self.assertIn('ready',visited);self.assertIn('following',visited)
                    self.assertEqual(m.completed,mode=='auto')
                    if mode=='auto':
                        self.assertIn('settling',visited)
                        self.assertTrue(all(v==w==0 for v,w,_ in m.step(poses,velocities,CONTROL,ORCA)))

    def test_graph_handles_longer_chains_and_rejects_cycles_and_duplicate_roles(self):
        names,opt=options('snake',count=5);opt['edges'].reverse()
        m=LeaderFollowerMission(names,opt)
        self.assertEqual([p[0] for p in m.local_goals],[0.,-.5,-1.,-1.5,-2.])
        names,opt=options('snake')
        for edges in [[dict(leader='r2',follower='r1',x=-.5,y=0),dict(leader='r1',follower='r2',x=-.5,y=0)],
                      [dict(leader='r0',follower='r1',x=-.5,y=0)]*2]:
            bad=dict(opt,edges=edges)
            with self.assertRaises(ValueError):LeaderFollowerMission(names,bad)

    def test_manual_request_is_ignored_before_handoff_and_progress_timeout_is_fault(self):
        names,opt=options();a=LeaderFollowerMission(names,opt);b=LeaderFollowerMission(names,opt)
        poses=[(0,0,0),(-.8,.6,.5),(-.7,-.7,-.3)];velocities=[(0,0)]*3
        self.assertEqual(a.step(poses,velocities,CONTROL,ORCA,teleop=(.15,.3)),
                         b.step(poses,velocities,CONTROL,ORCA,teleop=(0,0)))
        with self.assertRaises(ValueError):a.step(poses,[(float('nan'),0)]*3,CONTROL,ORCA)
        cfg=dict(DEFAULT_ADMISSION,progress_timeout=.3)
        a=LeaderFollowerMission(names,opt,cfg)
        with self.assertRaisesRegex(OrcaBlocked,'progress'):
            for _ in range(20):a.step(poses,velocities,CONTROL,ORCA)


if __name__=='__main__':unittest.main()
