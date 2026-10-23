"""Planar front-point leader-follower control (metres, radians, body twists)."""

import math


def tracking_velocity(relative_pose, leader_velocity, offset, front_distance,
                      k_x, k_y):
    """Return follower body velocity before saturation, using the old law."""
    x, y, yaw = relative_pose
    v_leader, w_leader = leader_velocity
    c, s = math.cos(yaw), math.sin(yaw)
    # The controlled point sits d metres in front of the follower axle.
    front_x = x + front_distance * c
    front_y = y + front_distance * s
    error_x = offset[0] + front_distance - front_x
    error_y = offset[1] - front_y
    u_x = k_x * error_x + v_leader - w_leader * front_y
    u_y = k_y * error_y + w_leader * front_x
    return (u_x * c + u_y * s,
            (u_y * c - u_x * s) / front_distance)
