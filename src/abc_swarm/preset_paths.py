"""Closed geometric paths, starting at (0, 0) with tangent along +x.

Distances are metres and angles radians. These are paths, not timed trajectories.
"""
import math


def preset_path(shape='circle', radius=1.0, width=2.5, height=2.0,
                resolution=0.025, clockwise=False, origin=(0.0, 0.0, 0.0)):
    values = (radius, width, height, resolution)
    if any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('Path dimensions and resolution must be positive and finite')
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError('Origin must be a finite planar pose')
    points = [(0.0, 0.0)]
    x, y, yaw = 0.0, 0.0, 0.0

    def line(length):
        nonlocal x, y
        if length == 0:
            return
        steps = max(1, math.ceil(length / resolution))
        for i in range(1, steps + 1):
            points.append((x + length * i / steps * math.cos(yaw),
                           y + length * i / steps * math.sin(yaw)))
        x, y = points[-1]

    def arc(angle):
        nonlocal x, y, yaw
        steps = max(1, math.ceil(radius * angle / resolution))
        for i in range(1, steps + 1):
            a = yaw + angle * i / steps
            points.append((x + radius * (math.sin(a) - math.sin(yaw)),
                           y - radius * (math.cos(a) - math.cos(yaw))))
        x, y = points[-1]
        yaw += angle

    if shape == 'circle':
        arc(2 * math.pi)
    elif shape == 'rounded_rectangle':
        if min(width, height) < 2 * radius:
            raise ValueError('Rectangle width and height must be at least 2 * radius')
        for length in (width - 2 * radius, height - 2 * radius) * 2:
            line(length)
            arc(math.pi / 2)
    else:
        raise ValueError('Unknown path shape: ' + shape)
    points[-1] = points[0]
    if clockwise:
        points = [(x, -y) for x, y in points]
    ox, oy, heading = origin
    c, s = math.cos(heading), math.sin(heading)
    return [(ox + c * x - s * y, oy + s * x + c * y) for x, y in points]
