"""Participant avoidance followed by guarded, precise pose stabilization."""
import math

from .orca import OrcaBlocked, OrcaInfeasible, orca_velocities
from .point_control import PointPhaseTracker, angle, validate_pose


DEFAULT_ADMISSION = dict(handoff_distance=.15, clearance=.02, position_margin=.005,
                         handoff_time=.3, settle_time=.5, progress_timeout=45.)


class AdmissionController:
    """Switch off ORCA only when per-goal capture disks are disjoint.

    The disks bound pure-stabilization positions and include physical body
    radii. Live position/clearance guards return to avoidance if that bound
    is exceeded. This is a sampled engineering guard, not a delay guarantee.
    """
    def __init__(self, settings=None):
        self.settings = dict(DEFAULT_ADMISSION if settings is None else settings)
        if set(self.settings) != set(DEFAULT_ADMISSION):
            raise ValueError('admission fields must match config/admission.yaml')
        for key, value in self.settings.items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('admission.'+key+' must be positive and finite')
        self.stage = 'assembling_orca'
        self.ready = False
        self.handoff_hold = self.settle_hold = 0.
        self.capture = None
        self.point = PointPhaseTracker()
        self.signature = None
        self.blocked = False
        self.recovering = False

    def pause(self):
        self.recovering = False
        self.handoff_hold = self.settle_hold = 0.
        self.ready = False
        if self.stage == 'ready':
            self.stage = 'fine_positioning'

    def _regions(self, poses, goals, control, orca):
        errors = [math.hypot(p[0]-g[0], p[1]-g[1]) for p, g in zip(poses, goals)]
        if max(errors) > self.settings['handoff_distance']:
            return None
        radii = [max(e, control['position_tolerance'])+self.settings['position_margin'] for e in errors]
        body = orca['robot_radius']+orca['safety_margin']
        reserve = 2*body+self.settings['clearance']
        for i, goal in enumerate(goals):
            for j in range(i):
                if math.hypot(goal[0]-goals[j][0], goal[1]-goals[j][1]) <= radii[i]+radii[j]+reserve:
                    return None
                if math.hypot(poses[i][0]-poses[j][0], poses[i][1]-poses[j][1]) <= reserve:
                    return None
        return radii

    def step(self, poses, goals, control, dt=.05, velocities=None, orca=None):
        self.blocked = False
        self.recovering = False
        if not poses or len(poses) != len(goals):
            raise ValueError('Admission needs one goal per robot')
        if not math.isfinite(dt) or not 0 < dt <= .2:
            raise ValueError('Invalid admission interval')
        for pose in list(poses)+list(goals):
            validate_pose(pose)
        signature = tuple(tuple(g) for g in goals)
        if self.signature is not None and signature != self.signature:
            raise ValueError('Admission goals must stay frozen; create a new admission task')
        self.signature = signature
        if velocities is not None and (len(velocities) != len(poses) or any(
                len(v) != 2 or not all(math.isfinite(x) for x in v) for v in velocities)):
            raise ValueError('One finite measured velocity is required per robot')
        if orca is not None and velocities is None:
            raise ValueError('ORCA admission requires measured velocities')
        if orca is not None and self.settings['handoff_distance'] < control['position_tolerance']:
            raise ValueError('handoff_distance must cover the final position tolerance')
        if orca is None:
            self.stage = 'fine_positioning'
        elif self.stage in ('fine_positioning', 'ready'):
            reserve = 2*(orca['robot_radius']+orca['safety_margin'])+self.settings['clearance']
            outside = any(math.hypot(p[0]-g[0], p[1]-g[1]) > r
                          for p, g, r in zip(poses, goals, self.capture))
            close = any(math.hypot(p[0]-q[0], p[1]-q[1]) <= reserve
                        for i, p in enumerate(poses) for q in poses[:i])
            if outside or close:
                self.stage = 'assembling_orca'
                self.ready = False
                self.handoff_hold = self.settle_hold = 0.
                self.point = PointPhaseTracker()
        if self.stage == 'assembling_orca':
            capture = self._regions(poses, goals, control, orca)
            self.handoff_hold = self.handoff_hold+dt if capture is not None else 0.
            if self.handoff_hold >= self.settings['handoff_time']:
                self.capture = capture
                self.stage = 'fine_positioning'
                self.point = PointPhaseTracker()
                return [(0., 0., 'handoff_barrier')]*len(poses)
            preferred = self.point.commands(poses, goals, control)
            try:
                commands = orca_velocities(poses, goals, velocities, control,
                                          preferred_commands=preferred, **orca)
                self.recovering = any(c[2]=='recovering' for c in commands)
                return commands
            except OrcaInfeasible:
                self.blocked = True
                self.settle_hold = 0.
                return [(0., 0., 'avoidance_wait')]*len(poses)
        commands = self.point.commands(poses, goals, control)
        ready = all(phase == 'arrived' for _, _, phase in commands)
        if velocities is not None:
            ready = ready and all(abs(v) < .02 and abs(w) < .05 for v, w in velocities)
        self.settle_hold = self.settle_hold+dt if ready else 0.
        self.ready = self.settle_hold >= self.settings['settle_time']
        if self.ready:
            self.stage = 'ready'
        elif self.stage == 'ready':
            self.stage = 'fine_positioning'
        return commands
