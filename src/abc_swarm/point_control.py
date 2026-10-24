"""Forward-only unicycle pose stabilization; coordinates in metres/radians.

Rotate towards the target, approach with heading feedback, then rotate to its
final yaw. This is a local controller, not a collision-free path planner.
"""
import math


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def validate_pose(pose):
    if (not isinstance(pose, (list, tuple)) or len(pose) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in pose)):
        raise ValueError('Expected finite [x_m, y_m, yaw_rad]')


def world_goals(poses, reference, anchor):
    if reference not in ('world', 'initial_leader'):
        raise ValueError('target_reference must be world or initial_leader')
    validate_pose(anchor)
    if not isinstance(poses, (list, tuple)) or not poses:
        raise ValueError('At least one target pose is required')
    result = []
    for pose in poses:
        validate_pose(pose)
        x, y, yaw = pose
        if reference == 'initial_leader':
            ax, ay, heading = anchor
            x, y = ax + math.cos(heading)*x - math.sin(heading)*y, ay + math.sin(heading)*x + math.cos(heading)*y
            yaw += heading
        result.append((x, y, angle(yaw)))
    return result


def formation_targets(formation, spacing):
    if formation not in ('triangle', 'snake'):
        raise ValueError('formation must be triangle or snake')
    if type(spacing) not in (int, float) or not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('spacing must be positive and finite')
    if formation == 'triangle':
        return [(0.0, 0.0, 0.0), (-spacing, spacing, 0.0), (-spacing, -spacing, 0.0)]
    return [(0.0, 0.0, 0.0), (-spacing, 0.0, 0.0), (-2*spacing, 0.0, 0.0)]


def point_velocity(pose, goal, max_linear=0.15, max_angular=0.5,
                   position_tolerance=0.04, yaw_tolerance=0.08,
                   k_distance=0.9, k_heading=2.0):
    validate_pose(pose)
    validate_pose(goal)
    limits = (max_linear, max_angular, position_tolerance, yaw_tolerance,
              k_distance, k_heading)
    if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits):
        raise ValueError('Limits, tolerances and gains must be finite and positive')
    x, y, yaw = pose
    gx, gy, gyaw = goal
    distance = math.hypot(gx - x, gy - y)
    heading = angle(gyaw - yaw)
    if distance <= position_tolerance:
        if abs(heading) <= yaw_tolerance:
            return 0.0, 0.0, 'arrived'
        return 0.0, max(-max_angular, min(max_angular, k_heading*heading)), 'orienting'
    bearing = angle(math.atan2(gy - y, gx - x) - yaw)
    angular = max(-max_angular, min(max_angular, k_heading*bearing))
    if abs(bearing) > math.pi/4:
        return 0.0, angular, 'aligning'
    linear = min(max_linear, k_distance*distance)*max(0.0, math.cos(bearing))
    return linear, angular, 'approaching'
