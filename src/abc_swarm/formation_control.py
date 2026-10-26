"""Frozen startup admission and one-owner leader/follower mission control."""
import math

from .admission import AdmissionController
from .leader_follower_control import tracking_velocity
from .orca import OrcaBlocked
from .point_control import PointPhaseTracker, angle, validate_pose, world_goals
from .preset_paths import preset_path
from .pure_pursuit import PurePursuit


class LeaderFollowerMission:
    """A rooted vehicle graph, independent of ROS and device-specific topics."""
    def __init__(self, names, options, admission=None, planning=True, stationary_hold=None):
        required = {'mode','edges','shape','radius','width','height','laps','clockwise','front_distance',
                    'k_x','k_y','lookahead','max_error','tracking_window'}
        if not isinstance(options, dict) or set(options) != required:
            raise ValueError('follow fields do not match the launch contract')
        if options['mode'] not in ('manual', 'auto'):
            raise ValueError('follow.mode must be manual or auto')
        if len(names) < 2 or len(set(names)) != len(names):
            raise ValueError('Mission requires unique vehicle names and one leader')
        for key in ('radius','width','height','front_distance','k_x','k_y','lookahead','max_error','tracking_window'):
            v = options[key]
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError('Invalid follow.'+key)
        if type(options['laps']) is not int or options['laps'] < 1:
            raise ValueError('follow.laps must be a positive integer')
        if type(options['clockwise']) is not bool:
            raise ValueError('follow.clockwise must be boolean')
        preset_path(options['shape'],radius=options['radius'],width=options['width'],height=options['height'])
        self.names, self.options = list(names), dict(options)
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
        if stationary_hold is not None and not isinstance(stationary_hold,dict):
            raise ValueError('stationary_hold must be a parameter dictionary')
        self.hold_settings = (dict(settle_time=.4,linear_threshold=.01,angular_threshold=.05)
                              if stationary_hold is None else dict(stationary_hold))
        if set(self.hold_settings)!={'settle_time','linear_threshold','angular_threshold'} or any(
                type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in self.hold_settings.values()):
            raise ValueError('stationary_hold requires positive settle_time/linear_threshold/angular_threshold')
        self.hold_times = [0.]*len(names)
        self.hold_anchors = {}
        self.hold_trackers = {}
        self.goals = None
        self.startup_goals = None
        self.admission_settings = admission
        self.planning = planning
        self.admission = AdmissionController(admission, planning)
        self.stage = 'assembling_orca'
        self.completed = False
        self.path = None
        self.tracker = None
        self.path_error = 0.
        self.progress = 0.
        self.idle = 0.
        self.best = None
        self.phase_key = None
        self.ready_hold = 0.
        self.formation_error = 0.
        self.leader_scale = 0.

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
        self.hold_times = [0.]*len(self.names)
        self.hold_anchors.clear()
        self.hold_trackers.clear()
        self.admission.pause()
        self.ready_hold = 0.
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
        if self.completed:
            return [(0., 0., 'completed')]*len(poses)
        if self.startup_goals is None:
            self.startup_goals = self._formation(poses[0])
            self.goals = list(self.startup_goals)
            if self.options['mode'] == 'auto':
                self.path = preset_path(self.options['shape'],radius=self.options['radius'],
                                        width=self.options['width'],height=self.options['height'],
                                        clockwise=self.options['clockwise'],origin=self.goals[0])
        if self.stage in ('assembling_orca','fine_positioning','ready','settling'):
            settling = self.stage == 'settling'
            if not settling and self.stage == 'ready':
                commands = self.admission.step(poses,self.goals,control,dt,velocities,orca)
                if not self.admission.ready:
                    self.stage = self.admission.stage
                    self.ready_hold = 0.
                    return commands
                self.ready_hold += dt
                if self.ready_hold < .25:
                    return [(0., 0., 'group_ready')]*len(poses)
                self._change('following')
                if self.options['mode'] == 'auto':
                    self.tracker = PurePursuit(self.path,laps=self.options['laps'],speed=control['max_linear'],
                                              lookahead=self.options['lookahead'],max_angular=control['max_angular'],
                                              goal_tolerance=control['position_tolerance'],max_error=self.options['max_error'])
                return [(0., 0., 'follow_handoff')]*len(poses)
            commands = self.admission.step(poses,self.goals,control,dt,velocities,orca)
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
        # The same physical envelope guard remains active after admission.
        reserve = 2*(orca['robot_radius']+orca['safety_margin'])+self.admission.settings['clearance']
        if any(math.hypot(p[0]-q[0],p[1]-q[1]) <= reserve for i,p in enumerate(poses) for q in poses[:i]):
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
            head = (max(-control['max_linear'],min(control['max_linear'],teleop[0])),
                    max(-control['max_angular'],min(control['max_angular'],teleop[1])))
        commands = [None]*len(poses)
        commands[0] = (*head,'leader_'+self.options['mode'])
        self.goals = [tuple(p) for p in poses]
        errors=[]
        for leader,follower,x,y in self.edges:
            lp,fp = poses[leader],poses[follower]
            self.goals[follower]=world_goals([(x,y,0.)],'initial_leader',lp)[0]
            # Root intent and parent command release holds immediately; measured
            # stop alone would misclassify startup delay or a slow correction.
            stopped = (all(abs(v)<1e-4 for v in head)
                       and all(abs(v)<1e-4 for v in commands[leader][:2])
                       and abs(velocities[leader][0])<self.hold_settings['linear_threshold']
                       and abs(velocities[leader][1])<self.hold_settings['angular_threshold'])
            anchor = self.hold_anchors.get(follower)
            if anchor is not None and (math.hypot(lp[0]-anchor[0],lp[1]-anchor[1])>control['position_tolerance']
                                       or abs(angle(lp[2]-anchor[2]))>control['yaw_tolerance']):
                stopped = False
            self.hold_times[follower] = self.hold_times[follower]+dt if stopped else 0.
            if not stopped:
                self.hold_anchors.pop(follower,None)
                self.hold_trackers.pop(follower,None)
            if self.hold_times[follower]>=self.hold_settings['settle_time']:
                if follower not in self.hold_anchors:
                    self.hold_anchors[follower] = tuple(lp)
                    self.hold_trackers[follower] = PointPhaseTracker()
                goal = world_goals([(x,y,0.)],'initial_leader',self.hold_anchors[follower])[0]
                self.goals[follower] = goal
                v,w,phase = self.hold_trackers[follower].commands([fp],[goal],control)[0]
                commands[follower] = (v,w,'holding_'+phase)
                errors.append(max(math.hypot(goal[0]-fp[0],goal[1]-fp[1]),
                                  self.options['front_distance']*abs(angle(goal[2]-fp[2]))))
                continue
            dx,dy = fp[0]-lp[0],fp[1]-lp[1]
            c,s = math.cos(lp[2]),math.sin(lp[2])
            relative = (c*dx+s*dy,-s*dx+c*dy,angle(fp[2]-lp[2]))
            d=self.options['front_distance']
            errors.append(math.hypot(relative[0]+d*math.cos(relative[2])-x-d,
                                     relative[1]+d*math.sin(relative[2])-y))
            v,w = tracking_velocity(relative,velocities[leader],(x,y),self.options['front_distance'],
                                     self.options['k_x'],self.options['k_y'])
            commands[follower] = (max(-control['max_linear'],min(control['max_linear'],v)),
                                  max(-control['max_angular'],min(control['max_angular'],w)),'following')
        self.formation_error=max(errors,default=0.)
        self.leader_scale=max(0.,1-self.formation_error/self.options['tracking_window'])
        commands[0]=(head[0]*self.leader_scale,head[1]*self.leader_scale,'leader_'+self.options['mode'])
        return commands
