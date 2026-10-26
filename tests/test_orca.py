#!/usr/bin/env python3
import math
import random
import unittest
from abc_swarm.orca import (OrcaBlocked,orca_velocities,project_velocity,front_point,front_velocity,dot)
from abc_swarm.point_control import angle

CONTROL=dict(max_linear=.15,max_angular=.5,position_tolerance=.04,yaw_tolerance=.08,k_distance=.9,k_heading=2.)


def integrate(pose,command,dt):
    x,y,yaw=pose;v,w=command
    if abs(w)<1e-9:return x+v*math.cos(yaw)*dt,y+v*math.sin(yaw)*dt,yaw
    return x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),angle(yaw+w*dt)


class OrcaTests(unittest.TestCase):
    def test_small_convex_solver_projects_without_relaxing_constraints(self):
        v=project_velocity((1,1),[((1,0),0),((0,1),0),((-1,-1),-1)],2)
        self.assertAlmostEqual(v[0],.5);self.assertAlmostEqual(v[1],.5)
        self.assertIsNone(project_velocity((0,0),[((1,0),1),((-1,0),.5)],2))
        rng=random.Random(13)
        for _ in range(100):
            feasible=(rng.uniform(-.05,.05),rng.uniform(-.05,.05))
            constraints=[]
            for _ in range(6):
                a=rng.uniform(-math.pi,math.pi);n=(math.cos(a),math.sin(a))
                constraints.append((n,dot(n,feasible)-rng.uniform(0,.1)))
            v=project_velocity((rng.uniform(-1,1),rng.uniform(-1,1)),constraints,.2)
            self.assertIsNotNone(v)
            self.assertLessEqual(math.hypot(*v),.2+1e-8)
            for n,b in constraints:self.assertGreaterEqual(dot(n,v),b-1e-8)

    def test_differential_drive_velocity_mapping_is_exact(self):
        for yaw in [0,.3,-1.4,math.pi]:
            u=front_velocity(yaw,(.12,-.4),.08)
            e=(math.cos(yaw),math.sin(yaw));side=(-e[1],e[0])
            self.assertAlmostEqual(dot(u,e),.12)
            self.assertAlmostEqual(dot(u,side)/.08,-.4)

    def test_head_on_crossing_and_three_robot_swap_converge_with_clearance(self):
        cases=[([(-1,0,0),(1,0,math.pi)],[(1,0,0),(-1,0,math.pi)]),
               ([(-1,0,0),(0,-1,math.pi/2)],[(1,0,0),(0,1,math.pi/2)]),
               ([(-1,0,0),(1,0,math.pi),(0,1,-math.pi/2)],[(1,0,0),(0,1,math.pi),(-1,0,-math.pi/2)])]
        for dt in [.02,.05]:
            for initial,goals in cases:
                with self.subTest(dt=dt,initial=initial):
                    poses=list(initial);velocities=[(0,0)]*len(poses)
                    for _ in range(round(60/dt)):
                        result=orca_velocities(poses,goals,velocities,CONTROL,robot_radius=.13,safety_margin=.02,front_distance=.08)
                        for v,w,_ in result:
                            self.assertLessEqual(abs(v),.15+1e-8)
                            self.assertLessEqual(abs(w),.5+1e-8)
                        poses=[integrate(p,r[:2],dt) for p,r in zip(poses,result)]
                        velocities=[r[:2] for r in result]
                        for i,p in enumerate(poses):
                            for q in poses[:i]:
                                self.assertGreater(math.hypot(p[0]-q[0],p[1]-q[1]),.30)
                                a,b=front_point(p,.08),front_point(q,.08)
                                self.assertGreater(math.hypot(a[0]-b[0],a[1]-b[1]),.46-1e-6)
                        if all(r[2]=='arrived' for r in result):break
                    else:self.fail('ORCA did not converge')
                    for p,g in zip(poses,goals):
                        self.assertLessEqual(math.hypot(p[0]-g[0],p[1]-g[1]),.04)
                        self.assertLessEqual(abs(angle(p[2]-g[2])),.08)

    def test_compact_mini_admits_actual_close_field_layout_without_losing_clearance(self):
        from abc_swarm.formation_control import LeaderFollowerMission
        # Actual stationary field layout formerly rejected by the 0.46 m envelope.
        initial=[(.8667,.4520,.0206),(.4615,.2166,.0964),(.4822,.6676,.0868)]
        names=['r0','r1','r2']
        options=dict(mode='manual',edges=[dict(leader='r0',follower='r1',x=-.5,y=.5),
                                        dict(leader='r0',follower='r2',x=-.5,y=-.5)],
                     shape='circle',radius=1.,width=2.,height=2.,laps=1,clockwise=False,
                     front_distance=.1,k_x=.5,k_y=.5,lookahead=.25,max_error=.5,tracking_window=.15)
        geometry=dict(robot_radius=.075,safety_margin=.01,front_distance=.04,
                      time_horizon=3.,right_bias=.01,velocity_buffer=.002)
        for dt in (.02,.05):
            mission=LeaderFollowerMission(names,options)
            poses=list(initial);velocities=[(0,0)]*3;minimum=10.
            for _ in range(round(60/dt)):
                commands=mission.step(poses,velocities,CONTROL,geometry,dt)
                for v,w,_ in commands:
                    self.assertLessEqual(abs(v),CONTROL['max_linear']+1e-8)
                    self.assertLessEqual(abs(w),CONTROL['max_angular']+1e-8)
                poses=[integrate(p,c[:2],dt) for p,c in zip(poses,commands)]
                velocities=[c[:2] for c in commands]
                for i,p in enumerate(poses):
                    for q in poses[:i]:
                        minimum=min(minimum,math.hypot(p[0]-q[0],p[1]-q[1]))
                        if mission.admission.stage=='assembling_orca':
                            self.assertGreater(math.dist(front_point(p,.04),front_point(q,.04)),.25-1e-6)
                if mission.stage=='following':break
            else:self.fail('compact Mini field layout did not reach following')
            self.assertGreater(minimum,.17)  # 15cm body + 1cm margin per vehicle.
        # Function defaults must match the Mini geometry too, including rejecting overlap.
        goals=[(0,0,0),(.5,0,0)]
        orca_velocities([(0,0,0),(.26,0,0)],goals,[(0,0)]*2,CONTROL)
        with self.assertRaises(OrcaBlocked):
            orca_velocities([(0,0,0),(.24,0,0)],goals,[(0,0)]*2,CONTROL)

    def test_invalid_geometry_feedback_and_conflicting_targets_stop(self):
        poses=[(-1,0,0),(1,0,0)];goals=[(1,0,0),(-1,0,0)];velocities=[(0,0)]*2
        for kw in [dict(robot_radius=0),dict(front_distance=-1),dict(time_horizon=float('nan')),dict(safety_margin=-1),dict(right_bias=True)]:
            with self.assertRaises(ValueError):orca_velocities(poses,goals,velocities,CONTROL,**kw)
        with self.assertRaises(OrcaBlocked):orca_velocities([(0,0,0),(.1,0,0)],goals,velocities,CONTROL)
        with self.assertRaises(OrcaBlocked):orca_velocities(poses,[(0,0,0),(0,0,1)],velocities,CONTROL)
        with self.assertRaises(ValueError):orca_velocities(poses,goals,[(math.nan,0),(0,0)],CONTROL)


if __name__=='__main__':unittest.main()
