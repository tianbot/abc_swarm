#!/usr/bin/env python3
"""Geometric, physical and lifecycle checks for the new path experiment."""
import math
import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from abc_swarm.preset_paths import preset_path
from abc_swarm.pure_pursuit import PurePursuit
from _kinematics import integrate, run


class PresetPathTests(unittest.TestCase):
    def test_shapes_are_closed_and_use_requested_geometry(self):
        circle = preset_path(radius=0.8)
        self.assertEqual(circle[0], circle[-1])
        self.assertTrue(all(abs(math.hypot(x, y - 0.8) - 0.8) < 1e-9 for x, y in circle))
        rect = preset_path('rounded_rectangle', radius=0.4, width=2.5, height=2.0)
        self.assertEqual(rect[0], rect[-1])
        self.assertAlmostEqual(max(x for x, y in rect) - min(x for x, y in rect), 2.5)
        self.assertAlmostEqual(max(y for x, y in rect) - min(y for x, y in rect), 2.0)
        moved = preset_path(radius=0.8, clockwise=True, origin=(2, 3, math.pi / 2))
        self.assertEqual(moved[0], (2, 3))
        self.assertGreater(moved[len(moved) // 2][0], 3.5)

    def test_closed_start_does_not_complete_and_progress_does_not_go_backwards(self):
        tracker = PurePursuit(preset_path(radius=0.8))
        v, w = tracker.step((0, 0, 0))
        self.assertGreater(v, 0)
        self.assertGreater(w, 0)
        self.assertFalse(tracker.completed)
        tracker.step((0.1, 0, 0))
        previous = tracker.progress
        tracker.step((0, 0, 0))
        self.assertGreaterEqual(tracker.progress, previous)

    def test_multiple_laps_and_continuous_mode(self):
        for laps in (0, 2):
            tracker = PurePursuit(preset_path(radius=0.4), laps=laps, speed=0.08)
            pose = (0, 0, 0)
            for i in range(2000):
                velocity = tracker.step(pose)
                pose = integrate(pose, velocity, 0.05)
                if tracker.completed:
                    break
            self.assertGreater(tracker.progress, 1.9 * tracker.length)
            self.assertEqual(tracker.completed, laps == 2)
            if tracker.completed:
                self.assertEqual(tracker.step(pose), (0, 0))

    def test_physical_tracking_with_initial_error_and_limits(self):
        for shape in ('circle', 'rounded_rectangle'):
            tracker = PurePursuit(preset_path(shape, radius=0.6), speed=0.08, max_angular=0.2)
            pose = (0, -0.08, 0.1)
            for i in range(5000):
                velocity = tracker.step(pose)
                self.assertLessEqual(abs(velocity[1]), 0.2 + 1e-9)
                self.assertLessEqual(velocity[0], 0.08)
                pose = integrate(pose, velocity, 0.05)
                if tracker.completed:
                    break
            self.assertTrue(tracker.completed, shape)
            self.assertLess(math.hypot(pose[0], pose[1]), 0.06)

    def test_all_shapes_and_formations_finish(self):
        for shape in ('circle', 'rounded_rectangle'):
            for formation in ('single', 'triangle', 'snake'):
                with self.subTest(shape=shape, formation=formation):
                    tracker = PurePursuit(preset_path(shape, radius=0.8))
                    rows = run(tracker, formation)
                    self.assertTrue(tracker.completed)
                    self.assertLess(max(r[3] for r in rows), 0.02)
                    self.assertTrue(all(math.isfinite(v) for row in rows
                                        for pose in row[1] for v in pose))
                    if formation != 'single':
                        self.assertLess(max(rows[-1][6]), 0.01)

    def test_invalid_dimensions_and_lost_path_are_rejected(self):
        for kwargs in ({'radius': 0}, {'width': 1, 'shape': 'rounded_rectangle'},
                       {'shape': 'unknown'}, {'origin': (math.nan, 0, 0)}):
            with self.assertRaises(ValueError):
                preset_path(**kwargs)
        for laps in (-1, 0.5, True):
            with self.assertRaises(ValueError):
                PurePursuit(preset_path(), laps=laps)
        tracker = PurePursuit(preset_path())
        with self.assertRaises(ValueError):
            tracker.step((100, 100, 0))



if __name__ == '__main__':
    unittest.main()
