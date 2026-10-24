#!/usr/bin/env python3
import math
import unittest
from abc_swarm.feedback import odometry_error


class FeedbackTests(unittest.TestCase):
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
