#!/usr/bin/env python3
import math
import unittest
from abc_swarm.feedback import MocapVelocityEstimator, odometry_error


class FeedbackTests(unittest.TestCase):
    def test_mocap_velocity_measures_forward_reverse_and_wrapped_turn(self):
        e = MocapVelocityEstimator()
        with self.assertRaisesRegex(ValueError, 'window'):
            e.update([(0, 0, 0), (0, 0, 3.13)], [10., 10.])
        result = e.update([(.03, 0, 0), (.03, 0, -3.13)], [10.2, 10.2])
        self.assertAlmostEqual(result[0][0], .15)
        self.assertAlmostEqual(result[0][1], 0.)
        self.assertAlmostEqual(result[1][0], -.15)
        self.assertAlmostEqual(result[1][1], (2*math.pi-6.26)/.2)
        self.assertEqual(e.update([(.03, 0, 0), (.03, 0, -3.13)], [10.2, 10.2]), result)

    def test_mocap_velocity_requires_new_window_after_gap_or_clock_reset(self):
        e = MocapVelocityEstimator()
        for stamp in [10., 10.02, 11., 9.]:
            with self.assertRaisesRegex(ValueError, 'window'):
                e.update([(0, 0, 0)], [stamp])
        self.assertEqual(e.update([(0, 0, 0)], [9.2]), [(0., 0.)])
        for poses, stamps in [([], []), ([(math.nan, 0, 0)], [1]), ([(0, 0, 0)], [0])]:
            with self.assertRaises(ValueError):e.update(poses, stamps)

    def test_observed_rosserial_clock_jitter_is_accepted(self):
        for age in [-0.09, -0.05, -0.001, 0, 0.05, 0.49]:
            self.assertEqual(odometry_error(100 - age, 100, 0.06, 0, 0), '')

    def test_future_stale_and_missing_packets_are_still_rejected(self):
        self.assertIn('ahead', odometry_error(100.11, 100, 0.01, 0, 0))
        self.assertIn('timestamp age', odometry_error(99.49, 100, 0.01, 0, 0))
        self.assertIn('no received', odometry_error(100, 100, 0.51, 0, 0))
        self.assertIn('zero', odometry_error(0, 100, 0.01, 0, 0))
        self.assertIn('nonfinite', odometry_error(100, 100, 0.01, math.nan, 0))


if __name__ == '__main__':
    unittest.main()
