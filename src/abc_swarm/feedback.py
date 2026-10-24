"""Odometry freshness checks shared by runtime and differential-drive followers."""
import math


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
