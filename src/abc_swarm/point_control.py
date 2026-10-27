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
                   k_distance=0.9, k_heading=2.0, align_angle=math.pi/4,
                   limit_output=True, position_hysteresis=0.01, previous_phase=None):
    """Return (v, omega, phase); limit_output=False exposes the raw P demand.

    A caller retaining previous_phase enters final rotation at tolerance minus
    position_hysteresis, and leaves it only outside position_tolerance. The
    final arrived criterion always uses the configured position/yaw tolerances.
    Stateless callers retain the original single position gate.
    """
    validate_pose(pose)
    validate_pose(goal)
    limits = (max_linear, max_angular, position_tolerance, yaw_tolerance,
              k_distance, k_heading, align_angle)
    if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits):
        raise ValueError('Limits, tolerances and gains must be finite and positive')
    if type(limit_output) is not bool or align_angle > math.pi/2:
        raise ValueError('Use a boolean output limit flag and align_angle <= pi/2')
    if (type(position_hysteresis) not in (int, float) or not math.isfinite(position_hysteresis)
            or position_hysteresis < 0
            or (previous_phase is not None and position_hysteresis >= position_tolerance)):
        raise ValueError('position_hysteresis must be nonnegative and smaller than the stateful position tolerance')
    if previous_phase not in (None, 'aligning', 'approaching', 'orienting', 'arrived'):
        raise ValueError('Unknown previous point-control phase')
    x, y, yaw = pose
    gx, gy, gyaw = goal
    distance = math.hypot(gx - x, gy - y)
    heading = angle(gyaw - yaw)
    position_gate = position_tolerance
    if previous_phase in ('aligning', 'approaching'):
        position_gate -= position_hysteresis
    if distance <= position_gate:
        if abs(heading) <= yaw_tolerance:
            return 0.0, 0.0, 'arrived'
        angular = k_heading*heading
        return 0.0, max(-max_angular, min(max_angular, angular)) if limit_output else angular, 'orienting'
    bearing = angle(math.atan2(gy - y, gx - x) - yaw)
    angular = k_heading*bearing
    if limit_output:
        angular = max(-max_angular, min(max_angular, angular))
    if abs(bearing) > align_angle:
        return 0.0, angular, 'aligning'
    linear = k_distance*distance
    if limit_output:
        linear = min(max_linear, linear)
    linear *= max(0.0, math.cos(bearing))
    return linear, angular, 'approaching'
