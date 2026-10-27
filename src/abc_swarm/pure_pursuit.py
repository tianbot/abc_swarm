"""Pure Pursuit for forward-moving planar unicycle/differential-drive robots.

Curvature kappa = 2*y_target / distance_target**2 in the robot frame;
angular velocity = linear velocity * kappa. Progress is local and monotone,
so the shared start/end of a closed route cannot finish a lap at startup.
"""
import bisect
import math


class PurePursuit:
    def __init__(self, points, laps=1, speed=0.15, lookahead=0.25,
                 max_angular=0.5, goal_tolerance=0.05, max_error=0.5):
        if isinstance(laps, bool) or not isinstance(laps, int) or laps < 0:
            raise ValueError('laps must be a nonnegative integer (0 means continuous)')
        if any(not math.isfinite(v) or v <= 0 for v in
               (speed, lookahead, max_angular, goal_tolerance, max_error)):
            raise ValueError('Control parameters must be positive and finite')
        if len(points) < 4 or points[0] != points[-1]:
            raise ValueError('Expected a closed path with at least three segments')
        if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Path points must be finite pairs')
        self.points = list(points)
        self.s = [0.0]
        for a, b in zip(points, points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if length <= 1e-9:
                raise ValueError('Path segments must have nonzero length')
            self.s.append(self.s[-1] + length)
        self.length = self.s[-1]
        self.laps, self.speed, self.lookahead = laps, speed, lookahead
        self.max_angular, self.goal_tolerance = max_angular, goal_tolerance
        self.max_error = max_error
        self.progress = 0.0
        self.completed = False
        self.target = points[0]
        self.error = 0.0

    def point_at(self, distance):
        distance %= self.length
        i = min(len(self.points) - 2, bisect.bisect_right(self.s, distance) - 1)
        u = (distance - self.s[i]) / (self.s[i + 1] - self.s[i])
        a, b = self.points[i:i + 2]
        return a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])

    def step(self, pose):
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('Robot pose must be finite (x, y, yaw)')
        if self.completed:
            return 0.0, 0.0
        x, y, yaw = pose
        count = len(self.points) - 1
        lap = int(self.progress / self.length)
        local = self.progress % self.length
        index = min(count - 1, bisect.bisect_right(self.s, local) - 1)
        index += lap * count
        search_end = self.progress + max(0.5, 2 * self.lookahead)
        total = self.laps * self.length if self.laps else math.inf
        best = (math.inf, self.progress)
        # Project only onto nearby upcoming segments, never the entire circle.
        while True:
            cycle, i = divmod(index, count)
            start = cycle * self.length + self.s[i]
            if start > search_end or start >= total:
                break
            a, b = self.points[i:i + 2]
            dx, dy = b[0] - a[0], b[1] - a[1]
            u = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) /
                                  (dx * dx + dy * dy)))
            distance = math.hypot(x - a[0] - u * dx, y - a[1] - u * dy)
            progress = start + u * (self.s[i + 1] - self.s[i])
            if distance < best[0]:
                best = distance, progress
            index += 1
        self.error = best[0]
        if self.error > self.max_error:
            raise ValueError('Robot too far from current path segment; disable and reset')
        self.progress = max(self.progress, min(total, best[1]))
        remaining = total - self.progress
        if (remaining <= self.goal_tolerance and
                math.hypot(x - self.points[-1][0], y - self.points[-1][1]) <= self.goal_tolerance):
            self.completed = True
            return 0.0, 0.0
        self.target = self.point_at(min(total, self.progress + self.lookahead))
        dx, dy = self.target[0] - x, self.target[1] - y
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        curvature = 2 * lateral / max(dx * dx + dy * dy, 1e-8)
        speed = min(self.speed, max(0.01, 0.5 * remaining))
        if abs(curvature) > 1e-9:
            speed = min(speed, self.max_angular / abs(curvature))
        return speed, speed * curvature
