"""Ideal planar kinematics and synchronous leader-follower experiments.

Feedback velocities equal the previous applied commands in this ideal model.
This does not model actuator dynamics, sensing delays, slip or collisions.
"""
import math
from abc_swarm.leader_follower_control import tracking_velocity


def integrate(pose, velocity, dt):
    x, y, yaw = pose
    v, w = velocity
    if abs(w) < 1e-9:
        return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw
    return (x + v / w * (math.sin(yaw + w * dt) - math.sin(yaw)),
            y - v / w * (math.cos(yaw + w * dt) - math.cos(yaw)), yaw + w * dt)


def relative_pose(leader, follower):
    dx, dy = follower[0] - leader[0], follower[1] - leader[1]
    c, s = math.cos(leader[2]), math.sin(leader[2])
    return c * dx + s * dy, -s * dx + c * dy, follower[2] - leader[2]


def run(controller, formation='single', spacing=0.5, dt=0.05, max_seconds=None):
    if formation not in ('single', 'triangle', 'snake'):
        raise ValueError('Unknown formation')
    if not all(math.isfinite(v) and v > 0 for v in (spacing, dt)):
        raise ValueError('Spacing and dt must be positive and finite')
    edges = [] if formation == 'single' else (
        [(0, 1, (-spacing, spacing)), (0, 2, (-spacing, -spacing))]
        if formation == 'triangle' else [(0, 1, (-spacing, 0)), (1, 2, (-spacing, 0))])
    poses = [(0.0, 0.0, 0.0)]
    for leader, follower, offset in edges:
        p = poses[leader]
        poses.append((p[0] + offset[0], p[1] + offset[1], p[2]))
    velocities = [(0.0, 0.0)] * len(poses)
    if max_seconds is None:
        max_seconds = controller.length * max(1, controller.laps) / controller.speed * 2 + 30
    if not math.isfinite(max_seconds) or max_seconds <= 0:
        raise ValueError('max_seconds must be positive and finite')
    rows, finished_at = [], None
    for step in range(math.ceil(max_seconds / dt)):
        elapsed = step * dt
        commands = [controller.step(poses[0])]
        errors, front_errors = [], []
        for leader, follower, offset in edges:
            relative = relative_pose(poses[leader], poses[follower])
            v, w = tracking_velocity(relative, velocities[leader], offset, 0.1, 0.5, 0.5)
            commands.append((max(-0.15, min(0.15, v)), max(-0.6, min(0.6, w))))
            errors.append(math.hypot(relative[0] - offset[0], relative[1] - offset[1]))
            front_errors.append(math.hypot(relative[0] + 0.1 * math.cos(relative[2]) - offset[0] - 0.1,
                                           relative[1] + 0.1 * math.sin(relative[2]) - offset[1]))
        rows.append((elapsed, tuple(poses), tuple(commands), controller.error,
                     tuple(errors), controller.progress, tuple(front_errors)))
        poses = [integrate(p, v, dt) for p, v in zip(poses, commands)]
        velocities = commands
        if controller.completed:
            if finished_at is None:
                finished_at = elapsed
            if elapsed - finished_at >= 10:
                break
    return rows
