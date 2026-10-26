#!/usr/bin/env python3
import math
import unittest

from abc_swarm.admission_planner import AdmissionPlanner, visibility_route, segment_clear, distance
from abc_swarm.admission import AdmissionController
from abc_swarm.point_control import angle


CONTROL=dict(max_linear=.15,max_angular=.6,position_tolerance=.04,yaw_tolerance=.08,k_distance=.9,k_heading=2.)
ORCA=dict(robot_radius=.075,safety_margin=.01,front_distance=.04,time_horizon=3.,right_bias=.01,velocity_buffer=.002)


def integrate(p,c,dt=.05):
    x,y,yaw=p;v,w=c[:2]
    if abs(w)<1e-9:return x+v*dt*math.cos(yaw),y+v*dt*math.sin(yaw),yaw
    return (x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),
            y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),angle(yaw+w*dt))


class AdmissionPlannerTests(unittest.TestCase):
    def test_route_goes_around_disk_and_occupied_goal_is_rejected(self):
        obstacles=[(0.,0.)];radius=.3
        route=visibility_route((-1.,0.),(1.,0.),obstacles,radius)
        self.assertGreater(len(route),2)
        self.assertTrue(all(segment_clear(a,b,obstacles,radius) for a,b in zip(route,route[1:])))
        self.assertIsNone(visibility_route((-1,0),(0,0),obstacles,radius))
        self.assertIsNotNone(visibility_route((.28,0),(1,0),obstacles,radius))
        self.assertFalse(segment_clear((.28,0),(-1,0),obstacles,radius))

    def test_only_one_preferred_mover_and_frozen_feedback_cannot_fake_progress(self):
        planner=AdmissionPlanner();poses=[(0,0,0),(-.8,.6,.5),(-.7,-.7,-.3)]
        goals=[(0,0,0),(-.5,.5,0),(-.5,-.5,0)]
        commands=planner.commands(poses,goals,CONTROL,ORCA,.02)
        self.assertEqual(sum(c[2]=='routing' for c in commands),1)
        self.assertTrue(all(c[:2]==(0.,0.) for i,c in enumerate(commands) if i!=planner.active))
        for _ in range(50):planner.commands(poses,goals,CONTROL,ORCA,.02)
        self.assertEqual(planner.progress,0.)
        planner.pause();planner.commands(poses,goals,CONTROL,ORCA,.02)
        self.assertEqual(planner.progress,0.)

    def test_exact_swap_uses_temporary_parking_and_still_checks_final_yaw(self):
        poses=[(-.5,0,0),(.5,0,math.pi),(0,1,0)]
        goals=[(.5,0,0),(-.5,0,0),(0,1,0)]
        a=AdmissionController();velocities=[(0,0)]*3;saw_parking=False
        for _ in range(2400):
            commands=a.step(poses,goals,CONTROL,.05,velocities,ORCA)
            saw_parking=saw_parking or a.planner.temporary
            self.assertTrue(all(abs(v)<=.15+1e-8 and abs(w)<=.6+1e-8 for v,w,_ in commands))
            poses=[integrate(p,c) for p,c in zip(poses,commands)];velocities=[c[:2] for c in commands]
            self.assertTrue(all(distance(p,q)>.17 for i,p in enumerate(poses) for q in poses[:i]))
            if a.ready:break
        else:self.fail('sequential swap did not complete')
        self.assertTrue(saw_parking)
        for p,g in zip(poses,goals):
            self.assertLessEqual(distance(p,g),.04)
            self.assertLessEqual(abs(angle(p[2]-g[2])),.08)

    def test_opt_out_keeps_progress_and_wrong_final_yaw_cannot_report_ready(self):
        a=AdmissionController(planning=False);goals=[(0,0,0),(.7,0,0)]
        a.step([(-.3,0,0),(1.,0,0)],goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        a.step([(-.2,0,0),(.9,0,0)],goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertIsNone(a.planner);self.assertGreater(a.progress,0.)
        a=AdmissionController()
        for _ in range(30):
            commands=a.step([(0,0,math.pi/2),(.7,0,0)],goals,CONTROL,velocities=[(0,0)]*2,orca=ORCA)
        self.assertEqual(a.stage,'fine_positioning');self.assertFalse(a.ready)
        self.assertEqual(commands[0][2],'orienting')


if __name__=='__main__':unittest.main()
