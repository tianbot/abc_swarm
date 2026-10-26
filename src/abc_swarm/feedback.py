"""Feedback freshness checks and stamped mocap velocity measurements."""
import math
from collections import deque


class MocapVelocityEstimator:
    """Measured planar body velocities from stamped control-center poses."""
    def __init__(self, window=.2, min_interval=.08, max_gap=.5):
        if not 0 < min_interval <= window <= max_gap:
            raise ValueError('Invalid mocap velocity time window')
        self.window, self.min_interval, self.max_gap = window, min_interval, max_gap
        self.samples = []

    def update(self, poses, stamps):
        if not poses or len(poses) != len(stamps):
            raise ValueError('One mocap timestamp is required per pose')
        if any(len(p) != 3 or not all(math.isfinite(x) for x in (*p, t)) or t <= 0
               for p, t in zip(poses, stamps)):
            raise ValueError('Invalid mocap velocity input')
        if len(self.samples) != len(poses):
            self.samples = [deque() for _ in poses]
        result = []
        for history, pose, stamp in zip(self.samples, poses, stamps):
            if history and (stamp < history[-1][0] or stamp-history[-1][0] > self.max_gap):
                history.clear()
            if not history or stamp > history[-1][0]:
                history.append((stamp, tuple(pose)))
            while len(history) > 2 and stamp-history[1][0] >= self.window:
                history.popleft()
            first, last = history[0], history[-1]
            dt = last[0]-first[0]
            if dt < self.min_interval:
                result.append(None)
                continue
            dyaw = math.atan2(math.sin(last[1][2]-first[1][2]), math.cos(last[1][2]-first[1][2]))
            heading = first[1][2]+dyaw/2
            dx, dy = last[1][0]-first[1][0], last[1][1]-first[1][1]
            measured=((dx*math.cos(heading)+dy*math.sin(heading))/dt, dyaw/dt)
            if not all(math.isfinite(x) for x in measured):
                raise ValueError('Invalid measured mocap velocity')
            result.append(measured)
        if any(v is None for v in result):
            raise ValueError('waiting for mocap velocity window')
        return result


def odometry_error(stamp, now, received_age, linear, angular, timeout=0.5,
                   future_tolerance=0.1):
    if not all(math.isfinite(x) for x in (stamp, now, received_age, linear, angular)):
        return 'nonfinite timestamp or velocity'
    if stamp <= 0:
        return 'zero/invalid timestamp'
    if received_age > timeout:
        return 'no received odometry for {:.3f}s'.format(received_age)
    age = now - stamp
    if age < -future_tolerance:
        return 'timestamp ahead by {:.3f}s (limit {:.3f}s)'.format(-age, future_tolerance)
    if age > timeout:
        return 'timestamp age {:.3f}s (limit {:.3f}s)'.format(age, timeout)
    return ''
