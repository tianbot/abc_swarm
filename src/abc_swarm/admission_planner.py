"""Sequential admission routes around registered vehicles, without ROS dependencies.

Visibility-graph search around expanded disks; not a map or a complete MAPF solver.
The caller retains ORCA, feedback checks and physical-body protection.
"""
import heapq
import math

from .point_control import point_velocity


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def segment_clear(a, b, obstacles, radius):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx+dy*dy
    for center in obstacles:
        ax, ay = a[0]-center[0], a[1]-center[1]
        if math.hypot(ax, ay) < radius-1e-8:
            # An initially close robot may leave the inflated planning disk,
            # but its first segment must never approach the other center.
            if ax*dx+ay*dy < -1e-8 or distance(b, center) < radius:
                return False
            continue
        t = max(0., min(1., -(ax*dx+ay*dy)/length2)) if length2 else 0.
        if math.hypot(ax+t*dx, ay+t*dy) < radius-1e-8:
            return False
    return True


def visibility_route(start, goal, obstacles, radius):
    """Return [start, ..., goal], or None; every edge clears the obstacle disks."""
    start, goal = tuple(start[:2]), tuple(goal[:2])
    if any(distance(goal, c) < radius for c in obstacles):
        return None
    if segment_clear(start, goal, obstacles, radius):
        return [start, goal]
    # Circumscribed polygon: adjacent chords remain outside the obstacle disk.
    samples = 16
    outer = (radius+.04)/math.cos(math.pi/samples)
    nodes = [start, goal]
    for center in obstacles:
        for i in range(samples):
            a = 2*math.pi*i/samples
            p = (center[0]+outer*math.cos(a), center[1]+outer*math.sin(a))
            if all(distance(p, c) >= radius for c in obstacles):
                nodes.append(p)
    costs = {0: 0.}; parents = {}; queue = [(0., 0)]
    while queue:
        cost, i = heapq.heappop(queue)
        if cost > costs[i]:
            continue
        if i == 1:
            path = [1]
            while path[-1] != 0:
                path.append(parents[path[-1]])
            return [nodes[n] for n in reversed(path)]
        for j, p in enumerate(nodes):
            if i == j or j == 0:
                continue
            candidate = cost+distance(nodes[i], p)
            if candidate < costs.get(j, math.inf) and segment_clear(nodes[i], p, obstacles, radius):
                costs[j], parents[j] = candidate, i
                heapq.heappush(queue, (candidate, j))
    return None


class AdmissionPlanner:
    """Keep one preferred mover, stable waypoints and measured route progress."""
    def __init__(self):
        self.active = None
        self.route = []
        self.progress = 0.
        self.best_distance = None
        self.temporary = False
        self.parking_count = 0
        self.state = 'idle'

    def pause(self):
        self.active = None
        self.route = []
        self.best_distance = None
        self.state = 'paused'

    def _select(self, poses, goals, pending, radius):
        for i in pending:
            obstacles = [p for j, p in enumerate(poses) if j != i]
            route = visibility_route(poses[i], goals[i], obstacles, radius)
            if route is not None:
                return i, route, False
        # Occupied destinations (e.g. an exact swap): temporarily park one car
        # away from every final goal, then let the next vehicle pass.
        if self.parking_count < len(poses):
            for i in pending:
                obstacles = [p for j, p in enumerate(poses) if j != i]
                choices = []
                for k in range(16):
                    a = 2*math.pi*k/16
                    p = (poses[i][0]+3*radius*math.cos(a), poses[i][1]+3*radius*math.sin(a))
                    if any(distance(p, g) < radius for g in goals):
                        continue
                    route = visibility_route(poses[i], p, obstacles, radius)
                    if route is not None:
                        choices.append((sum(distance(a,b) for a,b in zip(route,route[1:])), route))
                if choices:
                    self.parking_count += 1
                    return i, min(choices, key=lambda x:x[0])[1], True
        return None, [], False

    def commands(self, poses, goals, control, orca, clearance, waypoint_tolerance=.025):
        tolerance = waypoint_tolerance
        if type(tolerance) not in (int,float) or not math.isfinite(tolerance) or tolerance<=0:
            raise ValueError('waypoint_tolerance must be positive and finite')
        base = 2*(orca['robot_radius']+orca['safety_margin']+orca['front_distance'])
        spacing = min((distance(g,h) for i,g in enumerate(goals) for h in goals[:i]), default=math.inf)
        radius = max(base, min(base+max(clearance, control['position_tolerance']), .95*spacing))
        commands = [(0., 0., 'yielding')]*len(poses)
        if self.active is not None:
            i = self.active
            while self.route and distance(poses[i], self.route[0]) <= tolerance:
                if self.best_distance is not None:
                    self.progress += max(0., self.best_distance-distance(poses[i], self.route[0]))
                self.route.pop(0)
                self.best_distance = None
            if not self.route:
                if not self.temporary:
                    self.parking_count = 0
                self.pause()
            elif not segment_clear(poses[i], self.route[0], [p for j,p in enumerate(poses) if j!=i], base):
                # The planning clearance absorbs waypoint tracking error;
                # don't invalidate a valid route for consuming that clearance.
                self.pause()  # A yielding robot moved: replan against current feedback.
        if self.active is None:
            pending = [i for i,p in enumerate(poses) if distance(p,goals[i]) > tolerance]
            if not pending:
                self.state = 'positions_ready'
                return commands
            i, route, temporary = self._select(poses, goals, pending, radius)
            if i is None:
                self.state = 'route_wait'
                return commands
            self.active, self.route, self.temporary = i, route[1:], temporary
            self.best_distance = None
        i = self.active
        remaining = distance(poses[i], self.route[0])
        if self.best_distance is not None and remaining < self.best_distance-.001:
            self.progress += self.best_distance-remaining
            self.best_distance = remaining
        elif self.best_distance is None:
            self.best_distance = remaining
        waypoint = (*self.route[0], poses[i][2])  # Final yaw belongs to precision stabilization.
        v,w,_ = point_velocity(poses[i], waypoint, **dict(control, position_tolerance=tolerance))
        commands[i] = (v,w,'parking' if self.temporary else 'routing')
        self.state = commands[i][2]
        return commands
