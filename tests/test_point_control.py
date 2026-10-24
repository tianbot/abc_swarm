#!/usr/bin/env python3
import math
import unittest
from abc_swarm.point_control import angle, formation_targets, point_velocity, world_goals


class PointControlTests(unittest.TestCase):
    def test_unicycle_reaches_position_and_final_yaw_from_different_sides(self):
        starts = [(0, 0, 0), (0, 0, math.pi), (1, 0, math.pi/2),
                  (-1, 1, -2), (0.3, -0.4, 2.9), (0.5, 0.5, 0),
                  (0.5, 0.5, -3.1)]
        goal = (0.5, 0.5, math.pi/2)
        for start in starts:
            pose = start
            with self.subTest(start=start):
                for _ in range(6000):
                    v, w, phase = point_velocity(pose, goal)
                    self.assertTrue(0 <= v <= 0.15)
                    self.assertLessEqual(abs(w), 0.5)
                    if phase == 'arrived':
                        break
                    x, y, yaw = pose
                    pose = (x + 0.05*v*math.cos(yaw), y + 0.05*v*math.sin(yaw), angle(yaw + 0.05*w))
                self.assertEqual(phase, 'arrived')
                self.assertLessEqual(math.hypot(pose[0]-goal[0],pose[1]-goal[1]),0.04)
                self.assertLessEqual(abs(angle(pose[2]-goal[2])),0.08)
                self.assertEqual((v,w),(0,0))

    def test_heading_only_and_target_behind_do_not_translate(self):
        self.assertEqual(point_velocity((0,0,0),(0,0,math.pi))[0],0)
        self.assertEqual(point_velocity((0,0,0),(-1,0,0))[0],0)
        self.assertEqual(point_velocity((0,0,math.pi),(0,0,-math.pi)),(0,0,'arrived'))

    def test_reference_transform_and_explicit_world_targets(self):
        local = [[0,0,0],[-0.5,0.5,0],[-0.5,-0.5,0]]
        goals = world_goals(local,'initial_leader',(1,2,math.pi/2))
        for actual, expected in zip(goals,[(1,2,math.pi/2),(0.5,1.5,math.pi/2),(1.5,1.5,math.pi/2)]):
            for a,b in zip(actual,expected):self.assertAlmostEqual(a,b)
        self.assertEqual(world_goals(local,'world',(10,20,0)),[tuple(p) for p in local])

    def test_triangle_and_snake_presets_hold_the_first_robot(self):
        self.assertEqual(formation_targets('triangle',0.5),[(0,0,0),(-0.5,0.5,0),(-0.5,-0.5,0)])
        self.assertEqual(formation_targets('snake',0.6),[(0,0,0),(-0.6,0,0),(-1.2,0,0)])
        for formation,spacing in [('unknown',0.5),('triangle',0),('snake',-1),('snake',True),('snake',math.nan)]:
            with self.assertRaises(ValueError):formation_targets(formation,spacing)

    def test_invalid_inputs_fail_before_motion(self):
        for key in ['max_linear','max_angular','position_tolerance','yaw_tolerance','k_distance','k_heading']:
            for bad in [0,-1,float('nan'),float('inf'),True]:
                with self.subTest(key=key,bad=bad),self.assertRaises(ValueError):
                    point_velocity((0,0,0),(1,0,0),**{key:bad})
        for pose in [[0,0], [0,0,float('nan')],['0',0,0],None]:
            with self.assertRaises(ValueError):point_velocity(pose,(0,0,0))
        with self.assertRaises(ValueError):world_goals([[0,0,0]],'map',(0,0,0))
        with self.assertRaises(ValueError):world_goals([],'world',(0,0,0))


if __name__=='__main__':unittest.main()
