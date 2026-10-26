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


def validate_mocap_calibration(calibration, names):
    """Explicit per-robot planar marker-to-axle calibration, empty means identity."""
    if not isinstance(calibration, dict) or (calibration and set(calibration) != set(names)):
        raise ValueError('mocap_calibration must map every configured robot name, or be empty')
    for setting in calibration.values():
        if not isinstance(setting, dict) or set(setting) != {'yaw_offset', 'marker_offset'}:
            raise ValueError('Calibration requires yaw_offset and marker_offset')
        offset = setting['marker_offset']
        if not isinstance(offset, (list, tuple)) or len(offset) != 2:
            raise ValueError('marker_offset must be [forward_m, left_m]')
        if any(type(v) not in (int, float) or not math.isfinite(v)
               for v in (setting['yaw_offset'], *offset)):
            raise ValueError('Calibration must contain finite numeric values')
    return calibration


def mocap_control_pose(pose, calibration=None):
    """Convert raw planar yaw/marker position into a horizontal axle pose.

    marker_offset is measured in the corrected vehicle frame, from axle to
    marker origin. Raw roll/pitch are deliberately not treated as vehicle axes.
    """
    validate_pose(pose)
    if calibration is None:
        return tuple(pose)
    yaw = angle(pose[2] + calibration['yaw_offset'])
    forward, left = calibration['marker_offset']
    result = (pose[0]-math.cos(yaw)*forward+math.sin(yaw)*left,
              pose[1]-math.sin(yaw)*forward-math.cos(yaw)*left, yaw)
    validate_pose(result)
    return result


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


class PointPhaseTracker:
    """Keep final orientation stable across small position excursions.

    Approach to half the accepted position tolerance before turning. Retain
    that phase up to twice the tolerance, but only report arrival inside the
    original position/yaw bounds. A changed goal resets the phase history.
    """
    def __init__(self):
        self.signature = None
        self.turning = []

    def commands(self, poses, goals, control):
        if len(poses) != len(goals):
            raise ValueError('One goal is required for every pose')
        # Validate the complete control contract before updating phase history.
        for pose, goal in zip(poses, goals):
            point_velocity(pose, goal, **control)
        tolerance = control.get('position_tolerance', .04)
        yaw_tolerance = control.get('yaw_tolerance', .08)
        signature = (tuple(tuple(g) for g in goals), tolerance, yaw_tolerance)
        if signature != self.signature:
            self.signature = signature
            self.turning = [False] * len(goals)
        result = []
        for i, (pose, goal) in enumerate(zip(poses, goals)):
            distance = math.hypot(pose[0]-goal[0], pose[1]-goal[1])
            heading = angle(goal[2]-pose[2])
            yaw_ready = abs(heading) <= yaw_tolerance
            if self.turning[i] and (distance > 2*tolerance or (yaw_ready and distance > tolerance)):
                self.turning[i] = False
            if not self.turning[i] and (distance <= .5*tolerance or (distance <= tolerance and yaw_ready)):
                self.turning[i] = True
            if self.turning[i]:
                limit = control.get('max_angular', .5)
                angular = max(-limit, min(limit, control.get('k_heading', 2.)*heading))
                result.append((0., 0. if yaw_ready else angular,
                               'arrived' if yaw_ready and distance <= tolerance else 'orienting'))
            else:
                result.append(point_velocity(pose, goal, **dict(control, position_tolerance=.5*tolerance)))
        return result
