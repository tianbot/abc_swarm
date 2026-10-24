#!/usr/bin/env python3
import math
import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from abc_swarm.coordinated_loop import CoordinatedLoop,LoopFault,loop_geometry
from abc_swarm.point_control import angle
from _kinematics import integrate

CONTROL=dict(max_linear=.15,max_angular=.5,position_tolerance=.04,yaw_tolerance=.08,k_distance=.9,k_heading=2.)


class CoordinatedLoopTests(unittest.TestCase):
    def test_geometry_size_direction_origin_and_three_equal_starts(self):
        for shape in ('circle','triangle'):
            for size in (1.5,2.):
                for cw in (False,True):
                    starts,paths=loop_geometry(shape,size,size,cw,(1,2,.3))
                    self.assertEqual(starts[0],(1,2,.3))
                    for p in paths:self.assertEqual(p[0],p[-1])
                    distances=[math.hypot(starts[i][0]-starts[j][0],starts[i][1]-starts[j][1]) for i,j in [(0,1),(1,2),(2,0)]]
                    expected=size if shape=='triangle' else math.sqrt(3)*size/2
                    for d in distances:self.assertAlmostEqual(d,expected)

    def test_assembly_waits_for_last_robot_and_holds_before_moving(self):
        c=CoordinatedLoop('circle');poses=list(c.starts)
        poses[2]=(poses[2][0]+.2,poses[2][1],poses[2][2])
        for _ in range(10):c.step(poses,CONTROL)
        self.assertEqual(c.stage,'assembling')
        for _ in range(9):c.step(c.starts,CONTROL)
        self.assertEqual(c.stage,'assembling')
        for _ in range(3):c.step(c.starts,CONTROL)
        self.assertEqual(c.stage,'tracking_circle')

    def test_all_sizes_and_directions_complete_with_corner_barriers(self):
        for shape in ('circle','triangle'):
            for size in (1.5,2.):
                for cw in (False,True):
                    with self.subTest(shape=shape,size=size,cw=cw):
                        c=CoordinatedLoop(shape,size,size,cw)
                        poses=list(c.starts);turn_entries=0;previous=c.stage
                        for _ in range(4000):
                            old=c.stage
                            commands=c.step(poses,CONTROL)
                            if c.stage=='turning' and previous!='turning':turn_entries+=1
                            previous=c.stage
                            if old=='turning':self.assertTrue(all(v==0 for v,w,p in commands))
                            self.assertTrue(all(abs(v)<=.15+1e-8 and abs(w)<=.5+1e-8 for v,w,p in commands))
                            poses=[integrate(p,cmd[:2],.05) for p,cmd in zip(poses,commands)]
                            if c.completed:break
                        self.assertTrue(c.completed,c.stage)
                        self.assertEqual(turn_entries,3 if shape=='triangle' else 0)
                        for pose,start in zip(poses,c.starts):
                            self.assertLessEqual(math.hypot(pose[0]-start[0],pose[1]-start[1]),.05)
                            self.assertLessEqual(abs(angle(pose[2]-start[2])),.08)
                        self.assertTrue(all(v==w==0 for v,w,p in c.step(poses,CONTROL)))

    def test_circle_coordinates_slow_robot_and_pauses_when_it_stops(self):
        c=CoordinatedLoop('circle',progress_timeout=5)
        poses=list(c.starts);maximum_gap=0
        for _ in range(3000):
            commands=c.step(poses,CONTROL)
            poses=[integrate(p,(cmd[0]*(.75 if i==2 else 1),cmd[1]*(.75 if i==2 else 1)),.05) for i,(p,cmd) in enumerate(zip(poses,commands))]
            maximum_gap=max(maximum_gap,c.progress_spread)
            if c.completed:break
        self.assertTrue(c.completed)
        self.assertLess(maximum_gap,.12)
        c=CoordinatedLoop('circle',progress_timeout=1)
        poses=list(c.starts)
        for _ in range(12):c.step(poses,CONTROL)
        with self.assertRaises(LoopFault):
            for _ in range(30):c.step(poses,CONTROL)

    def test_infinite_loops_and_two_triangle_laps(self):
        for shape,laps,steps in [('circle',0,1200),('triangle',2,3000)]:
            c=CoordinatedLoop(shape,laps=laps)
            poses=list(c.starts)
            for _ in range(steps):
                commands=c.step(poses,CONTROL)
                poses=[integrate(p,cmd[:2],.05) for p,cmd in zip(poses,commands)]
                if c.completed:break
            if laps:self.assertTrue(c.completed);self.assertEqual(c.rounds_completed,6)
            else:self.assertFalse(c.completed);self.assertGreater(c.common_progress,2*math.pi)

    def test_invalid_input_and_off_path_fail(self):
        for kwargs in [dict(shape='unknown'),dict(shape='circle',diameter=1),dict(shape='triangle',side_length=0),
                       dict(shape='circle',speed=.16),dict(shape='circle',laps=True),dict(shape='circle',phase_window=2),
                       dict(shape='circle',clockwise='false')]:
            with self.assertRaises(ValueError):CoordinatedLoop(**kwargs)
        c=CoordinatedLoop('circle')
        for _ in range(12):c.step(c.starts,CONTROL)
        poses=list(c.starts);poses[0]=(100,100,0)
        with self.assertRaises(LoopFault):c.step(poses,CONTROL)


if __name__=='__main__':unittest.main()
