"""Participant avoidance followed by guarded, precise pose stabilization."""
import math

from .orca import OrcaBlocked, OrcaInfeasible, orca_velocities
from .point_control import point_velocity, angle, validate_pose
from .admission_planner import AdmissionPlanner


DEFAULT_ADMISSION = dict(handoff_distance=.15, clearance=.02, position_margin=.005,
                         handoff_time=.3, settle_time=.5, progress_timeout=45.,
                         stop_linear=.02, stop_angular=.05, waypoint_tolerance=.025)


class AdmissionController:
    """Switch off ORCA only when per-goal capture disks are disjoint.

    The disks bound pure-stabilization positions and include physical body
    radii. Live position/clearance guards return to avoidance if that bound
    is exceeded. This is a sampled engineering guard, not a delay guarantee.
    """
    def __init__(self, settings=None, planning=True):
        if type(planning) is not bool:
            raise ValueError('planning must be boolean')
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
        self.signature = None
        self.blocked = False
        self.recovering = False
        self.planner = AdmissionPlanner() if planning else None
        self.precision_progress = 0.
        self.best_pose_error = None
        self.trace = {}
        self.point_phases = None

    @property
    def progress(self):
        return (self.planner.progress if self.planner is not None else 0.)+self.precision_progress

    def _observe_pose_progress(self, poses, goals):
        error = sum(math.hypot(p[0]-g[0],p[1]-g[1])+.08*abs(angle(g[2]-p[2])) for p,g in zip(poses,goals))
        if self.best_pose_error is None:
            self.best_pose_error = error
        elif error < self.best_pose_error-.001:
            self.precision_progress += self.best_pose_error-error
            self.best_pose_error = error

    def pause(self):
        self.trace = {}
        if self.planner is not None:
            self.planner.pause()
        self.recovering = False
        self.handoff_hold = self.settle_hold = 0.
        self.ready = False
        if self.stage == 'ready':
            self.stage = 'fine_positioning'

    def record(self, raw, point, preferred, output, processing):
        """Observe each computation stage without altering its returned command."""
        self.trace = {key: [tuple(c[:2]) for c in values] for key, values in
                      [('raw',raw),('point',point),('preferred',preferred),('output',output)]}
        self.trace['processing'] = list(processing)
        return output

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
        raw = [point_velocity(p,g,limit_output=False,**control) for p,g in zip(poses,goals)]
        point = [point_velocity(p,g,**control) for p,g in zip(poses,goals)]
        if self.planner is None:
            self._observe_pose_progress(poses, goals)
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
                self.point_phases = None
                self.ready = False
                self.handoff_hold = self.settle_hold = 0.
        if self.stage == 'assembling_orca':
            capture = self._regions(poses, goals, control, orca)
            self.handoff_hold = self.handoff_hold+dt if capture is not None else 0.
            if self.handoff_hold >= self.settings['handoff_time']:
                if self.planner is not None:
                    self.planner.pause()
                    self.planner.state = 'positions_ready'
                self.capture = capture
                self.stage = 'fine_positioning'
                zeros = [(0., 0., 'handoff_barrier')]*len(poses)
                return self.record(raw,point,zeros,zeros,['handoff_barrier'])
            preferred = (self.planner.commands(poses, goals, control, orca, self.settings['clearance'],self.settings['waypoint_tolerance'])
                         if self.planner is not None else point)
            try:
                commands = orca_velocities(poses, goals, velocities, control,
                                          preferred_commands=preferred, **orca)
                self.recovering = any(c[2]=='recovering' for c in commands)
                return self.record(raw,point,preferred,commands,['point_control','planning','orca'] if self.planner is not None else ['point_control','orca'])
            except OrcaInfeasible:
                self.blocked = True
                self.settle_hold = 0.
                zeros = [(0., 0., 'avoidance_wait')]*len(poses)
                return self.record(raw,point,preferred,zeros,['orca_wait'])
        # 精定位记住各车阶段：先进入内圈，再对齐最终方向；超出最终
        # 位置容差才返回位置调整。余量来自position_hysteresis，不放宽到位判定。
        previous = self.point_phases or ['approaching']*len(poses)
        raw = [point_velocity(p,g,previous_phase=phase,limit_output=False,**control)
               for p,g,phase in zip(poses,goals,previous)]
        commands = [point_velocity(p,g,previous_phase=phase,**control)
                    for p,g,phase in zip(poses,goals,previous)]
        self.point_phases = [c[2] for c in commands]
        if self.planner is not None:
            self._observe_pose_progress(poses, goals)
        ready = all(phase == 'arrived' for _, _, phase in commands)
        if velocities is not None:
            ready = ready and all(abs(v) < self.settings['stop_linear'] and abs(w) < self.settings['stop_angular'] for v, w in velocities)
        self.settle_hold = self.settle_hold+dt if ready else 0.
        self.ready = self.settle_hold >= self.settings['settle_time']
        if self.ready:
            self.stage = 'ready'
        elif self.stage == 'ready':
            self.stage = 'fine_positioning'
        return self.record(raw,commands,commands,commands,['point_control','output_limits'])
