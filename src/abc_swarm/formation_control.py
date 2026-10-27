"""Frozen startup admission and one-owner leader/follower mission control."""
import math

from .admission import AdmissionController
from .leader_follower_control import tracking_velocity
from .orca import OrcaBlocked
from .point_control import angle, validate_pose, world_goals
from .preset_paths import preset_path
from .pure_pursuit import PurePursuit


class LeaderFollowerMission:
    """A rooted vehicle graph, independent of ROS and device-specific topics."""
    def __init__(self, names, options, admission=None, planning=True, startup=True):
        required = {'mode','edges','shape','radius','width','height','laps','clockwise','front_distance',
                    'k_x','k_y','lookahead','max_error','speed','max_linear','max_angular'}
        if not isinstance(options, dict) or set(options) != required:
            raise ValueError('follow fields do not match the launch contract')
        if options['mode'] not in ('manual', 'auto'):
            raise ValueError('follow.mode must be manual or auto')
        if len(names) < 2 or len(set(names)) != len(names):
            raise ValueError('Mission requires unique vehicle names and one leader')
        for key in ('radius','width','height','front_distance','k_x','k_y','lookahead','max_error','speed','max_linear','max_angular'):
            v = options[key]
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError('Invalid follow.'+key)
        if type(options['laps']) is not int or options['laps'] < 1:
            raise ValueError('follow.laps must be a positive integer')
        if type(options['clockwise']) is not bool:
            raise ValueError('follow.clockwise must be boolean')
        preset_path(options['shape'],radius=options['radius'],width=options['width'],height=options['height'])
        self.names, self.options = list(names), dict(options)
        if type(startup) is not bool:raise ValueError('startup must be boolean')
        self.startup = startup
        edges = options['edges']
        if not isinstance(edges, list) or len(edges) != len(names)-1:
            raise ValueError('Every follower requires one leader edge')
        self.edges = []
        for e in edges:
            if not isinstance(e, dict) or set(e) != {'leader','follower','x','y'}:
                raise ValueError('Edges require leader, follower, x and y')
            if e['leader'] not in names or e['follower'] not in names or e['follower'] == names[0]:
                raise ValueError('Invalid leader/follower mapping')
            validate_pose((e['x'], e['y'], 0.))
            self.edges.append((names.index(e['leader']), names.index(e['follower']), e['x'], e['y']))
        if len({e[1] for e in self.edges}) != len(names)-1:
            raise ValueError('Duplicate follower role')
        self.local_goals = self._formation((0., 0., 0.))  # Validate graph connectivity/cycles.
        depth = {0:0}
        while len(depth)<len(names):
            for leader,follower,_,_ in self.edges:
                if leader in depth:depth[follower]=depth[leader]+1
        self.edges.sort(key=lambda e:depth[e[1]])
        self.goals = None
        self.startup_goals = None
        self.admission_settings = admission
        self.planning = planning
        self.admission = AdmissionController(admission, planning)
        self.stage = 'assembling_orca' if startup else 'following'
        if not startup:self.admission.stage='skipped'
        self.completed = False
        self.path = None
        self.tracker = None
        self.path_error = 0.
        self.progress = 0.
        self.idle = 0.
        self.best = None
        self.phase_key = None
        self.formation_error = 0.
        self.leader_request = None
        self.raw_commands = []

    def prepare_tracker(self, control):
        if self.options['mode']=='auto':
            self.tracker=PurePursuit(self.path,laps=self.options['laps'],speed=self.options['speed'],
                                    lookahead=self.options['lookahead'],max_angular=self.options['max_angular'],
                                    goal_tolerance=control['position_tolerance'],max_error=self.options['max_error'])

    def _formation(self, anchor):
        goals = {0: tuple(anchor)}
        pending = list(self.edges)
        while pending:
            previous = len(pending)
            for edge in list(pending):
                leader, follower, x, y = edge
                if leader in goals:
                    goals[follower] = world_goals([(x, y, 0.)], 'initial_leader', goals[leader])[0]
                    pending.remove(edge)
            if len(pending) == previous:
                raise ValueError('Follow graph must be connected to the first vehicle without cycles')
        return [goals[i] for i in range(len(self.names))]

    def _stall(self, metric, dt):
        key = 'admission' if self.stage in ('assembling_orca','fine_positioning','ready') else self.stage
        if key != self.phase_key or self.best is None or metric > self.best+.003:
            self.phase_key, self.best, self.idle = key, metric, 0.
        else:
            self.idle += dt
        if self.idle > self.admission.settings['progress_timeout']:
            raise OrcaBlocked('no mission progress; inspect feedback/targets and restart')

    def _change(self, stage):
        self.stage = stage
        self.best = self.phase_key = None
        self.idle = 0.

    def pause(self):
        self.admission.pause()
        if self.stage == 'ready':
            self.stage = self.admission.stage

    def step(self, poses, velocities, control, orca, dt=.05, teleop=(0., 0.)):
        if len(poses) != len(self.names) or len(velocities) != len(poses):
            raise ValueError('Mission needs poses and measured velocities for all vehicles')
        if any(not isinstance(v,(tuple,list)) or len(v)!=2 or not all(math.isfinite(x) for x in v) for v in velocities):
            raise ValueError('Mission measured velocities must be finite body twists')
        for pose in poses:
            validate_pose(pose)
        if not math.isfinite(dt) or not 0 < dt <= .2:
            raise ValueError('Invalid mission interval')
        assembly_orca=orca if self.startup else None
        if self.completed:
            return [(0., 0., 'completed')]*len(poses)
        if self.startup_goals is None:
            self.startup_goals = self._formation(poses[0])
            self.goals = list(self.startup_goals)
            if self.options['mode'] == 'auto':
                self.path = preset_path(self.options['shape'],radius=self.options['radius'],
                                        width=self.options['width'],height=self.options['height'],
                                        clockwise=self.options['clockwise'],origin=self.goals[0])
            if not self.startup:
                self.prepare_tracker(control)
                return [(0.,0.,'follow_handoff')]*len(poses)
        if self.stage in ('assembling_orca','fine_positioning','ready','settling'):
            settling = self.stage == 'settling'
            if not settling and self.stage == 'ready':
                commands = self.admission.step(poses,self.goals,control,dt,velocities,assembly_orca)
                if not self.admission.ready:
                    self.stage = self.admission.stage
                    return commands
                self._change('following')
                self.prepare_tracker(control)
                return [(0., 0., 'follow_handoff')]*len(poses)
            commands = self.admission.step(poses,self.goals,control,dt,velocities,assembly_orca)
            self._stall(self.admission.progress,dt)
            if self.admission.ready:
                if settling:
                    self.completed = True
                    self._change('completed')
                else:
                    self._change('ready')
                return [(0., 0., 'group_ready')]*len(poses)
            if not settling:
                self.stage = self.admission.stage
            return commands
        # Explicit stop protection; this reports a fault instead of scaling commands.
        if orca is not None:
            reserve=2*(orca['robot_radius']+orca['safety_margin'])+self.admission.settings['clearance']
            if any(math.hypot(p[0]-q[0],p[1]-q[1])<=reserve for i,p in enumerate(poses) for q in poses[:i]):
                raise OrcaBlocked('vehicle clearance lost while following')
        if self.options['mode'] == 'auto':
            try:
                head = self.tracker.step(poses[0])
            except ValueError as exc:
                raise OrcaBlocked(str(exc))
            self.path_error, self.progress = self.tracker.error, self.tracker.progress
            self._stall(self.progress,dt)
            if self.tracker.completed:
                self.goals = self._formation(poses[0])
                self.admission = AdmissionController(self.admission_settings, self.planning)
                self._change('settling')
                return [(0., 0., 'finish_barrier')]*len(poses)
        else:
            if len(teleop) != 2 or not all(math.isfinite(v) for v in teleop):
                raise ValueError('Invalid leader request')
            # Manual Twist is already the operator's command. Do not rewrite it.
            head = tuple(teleop)
        self.leader_request = tuple(head)
        self.raw_commands = [None]*len(poses)
        self.raw_commands[0] = tuple(head)
        commands = [None]*len(poses)
        commands[0] = (*head,'leader_'+self.options['mode'])
        self.goals = [tuple(p) for p in poses]
        errors=[]
        for leader,follower,x,y in self.edges:
            lp,fp = poses[leader],poses[follower]
            self.goals[follower]=world_goals([(x,y,0.)],'initial_leader',lp)[0]
            dx,dy = fp[0]-lp[0],fp[1]-lp[1]
            c,s = math.cos(lp[2]),math.sin(lp[2])
            relative = (c*dx+s*dy,-s*dx+c*dy,angle(fp[2]-lp[2]))
            d=self.options['front_distance']
            errors.append(math.hypot(relative[0]+d*math.cos(relative[2])-x-d,
                                     relative[1]+d*math.sin(relative[2])-y))
            v,w = tracking_velocity(relative,velocities[leader],(x,y),self.options['front_distance'],
                                     self.options['k_x'],self.options['k_y'])
            self.raw_commands[follower] = (v,w)
            commands[follower] = (max(-self.options['max_linear'],min(self.options['max_linear'],v)),
                                  max(-self.options['max_angular'],min(self.options['max_angular'],w)),'following')
        self.formation_error=max(errors,default=0.)
        return commands
