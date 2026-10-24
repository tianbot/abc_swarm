"""Three-robot geometric loop coordination, independent of ROS and device IO.

All robots assemble at their own start pose. Circle followers share the slowest
observed path progress. Triangle edges use a stop/turn barrier at every vertex.
"""
import math
from .point_control import angle, point_velocity, validate_pose, world_goals
from .preset_paths import preset_path
from .pure_pursuit import PurePursuit
from .orca import orca_velocities


class LoopFault(ValueError):
    """Tracking cannot safely continue on the frozen task."""


def loop_geometry(shape, diameter=2.0, side_length=2.0, clockwise=False, origin=(0,0,0)):
    validate_pose(origin)
    if shape not in ('circle','triangle'):
        raise ValueError('shape must be circle or triangle')
    if type(clockwise) is not bool:
        raise ValueError('clockwise must be boolean')
    for value in (diameter,side_length):
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('Loop dimensions must be positive and finite')
    if diameter<1.5 or side_length<1.5:
        raise ValueError('Mocap loop diameter and side length must be at least 1.5 m')
    sign=-1 if clockwise else 1
    if shape=='circle':
        r=diameter/2
        local=[(r*math.sin(2*math.pi*i/3),sign*r*(1-math.cos(2*math.pi*i/3)),
                sign*2*math.pi*i/3) for i in range(3)]
    else:
        local=[(0,0,0),(side_length,0,sign*2*math.pi/3),
               (side_length/2,sign*math.sqrt(3)*side_length/2,sign*4*math.pi/3)]
    starts=world_goals(local,'initial_leader',origin)
    if shape=='circle':
        paths=[preset_path('circle',radius=diameter/2,clockwise=clockwise,origin=p) for p in starts]
    else:
        vertices=[p[:2] for p in starts]
        paths=[[vertices[(i+j)%3] for j in range(4)] for i in range(3)]
    return starts,paths


def straight_velocity(pose,start,end,speed,max_angular,lookahead,tolerance,max_error,k_heading,k_distance):
    """Open straight-segment Pure Pursuit; never looks through the next corner."""
    dx,dy=end[0]-start[0],end[1]-start[1];length=math.hypot(dx,dy)
    ux,uy=dx/length,dy/length
    along=(pose[0]-start[0])*ux+(pose[1]-start[1])*uy
    projection=max(0,min(length,along))
    nearest=(start[0]+projection*ux,start[1]+projection*uy)
    error=math.hypot(pose[0]-nearest[0],pose[1]-nearest[1])
    if error>max_error:raise LoopFault('robot too far from its current triangle edge')
    remaining=math.hypot(end[0]-pose[0],end[1]-pose[1])
    if remaining<=tolerance:return (0,0,'edge_arrived'),length,error
    target=min(length,max(0,along)+lookahead)
    tx,ty=start[0]+target*ux-pose[0],start[1]+target*uy-pose[1]
    bearing=angle(math.atan2(ty,tx)-pose[2])
    if abs(bearing)>math.pi/3:
        return (0,max(-max_angular,min(max_angular,k_heading*bearing)),'edge_aligning'),projection,error
    lateral=-math.sin(pose[2])*tx+math.cos(pose[2])*ty
    curvature=2*lateral/max(tx*tx+ty*ty,1e-8)
    v=min(speed,k_distance*remaining)
    if abs(curvature)>1e-9:v=min(v,max_angular/abs(curvature))
    return (v,v*curvature,'tracking_edge'),projection,error


class CoordinatedLoop:
    def __init__(self,shape,diameter=2.0,side_length=2.0,clockwise=False,laps=1,
                 speed=.15,origin=(0,0,0),lookahead=.20,phase_window=.10,
                 settle_time=.5,max_error=.25,progress_timeout=30.0):
        self.starts,self.paths=loop_geometry(shape,diameter,side_length,clockwise,origin)
        if type(laps) is not int or laps<0:raise ValueError('laps must be nonnegative integer')
        for value in (speed,lookahead,phase_window,settle_time,max_error,progress_timeout):
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
                raise ValueError('Loop speed and control parameters must be positive and finite')
        if speed>.15:raise ValueError('Loop speed must not exceed 0.15 m/s')
        if phase_window>=min(diameter*math.pi/3,side_length/3):
            raise ValueError('phase_window too large for the configured geometry')
        self.shape,self.laps,self.speed=shape,laps,speed
        self.diameter,self.side_length=diameter,side_length
        self.lookahead,self.phase_window=lookahead,phase_window
        self.settle_time,self.max_error=settle_time,max_error
        self.progress_timeout=progress_timeout
        self.stage='assembling';self.completed=False
        self.goals=list(self.starts)
        self.trackers=None
        self.rounds_completed=0;self.common_progress=0.0;self.progress_spread=0.0
        self.errors=[0.0]*3
        self.hold=0.0;self.idle=0.0;self.metric=None;self.metric_key=None

    def _hold(self,ready,dt):
        self.hold=self.hold+dt if ready else 0
        return self.hold>=self.settle_time

    def _change(self,stage):
        self.stage=stage;self.hold=0;self.metric=None;self.metric_key=None;self.idle=0

    def _stall(self,metric,dt):
        key=(self.stage,self.rounds_completed)
        if key!=self.metric_key or self.metric is None or metric>self.metric+.005:
            self.metric_key,self.metric,self.idle=key,metric,0
        else:self.idle+=dt
        if self.idle>self.progress_timeout:raise LoopFault('no coordinated progress; inspect robot feedback and restart')

    def _scale(self,commands,progress):
        minimum=min(progress);self.progress_spread=max(progress)-minimum
        return [(v*max(0,1-(p-minimum)/self.phase_window),
                 w*max(0,1-(p-minimum)/self.phase_window),phase)
                for (v,w,phase),p in zip(commands,progress)]

    def _edge_goals(self,next_heading=False):
        result=[]
        for i in range(3):
            start=self.starts[(i+self.rounds_completed)%3]
            end=self.starts[(i+self.rounds_completed+1)%3]
            heading=end[2] if next_heading else math.atan2(end[1]-start[1],end[0]-start[0])
            result.append((end[0],end[1],heading))
        return result

    def step(self,poses,control,dt=.05,velocities=None,assembly_orca=None):
        if len(poses)!=3:raise ValueError('Coordinated loops require exactly three poses')
        for pose in poses:validate_pose(pose)
        if not math.isfinite(dt) or not 0<dt<=.2:raise ValueError('Invalid control interval')
        if self.completed:return [(0,0,'completed')]*3
        if self.stage in ('assembling','finishing'):
            self.goals=list(self.starts)
            if self.stage=='assembling' and assembly_orca is not None:
                if velocities is None:raise ValueError('Assembly ORCA needs measured velocities')
                commands=orca_velocities(poses,self.goals,velocities,control,**assembly_orca)
            else:commands=[point_velocity(p,g,**control) for p,g in zip(poses,self.goals)]
            ready=all(phase=='arrived' for _,_,phase in commands)
            if velocities is not None:ready=ready and all(abs(v)<.02 and abs(w)<.05 for v,w in velocities)
            error=sum(math.hypot(p[0]-g[0],p[1]-g[1])+.08*abs(angle(g[2]-p[2])) for p,g in zip(poses,self.goals))
            self._stall(-error,dt)
            if self._hold(ready,dt):
                if self.stage=='finishing':self.completed=True;self._change('completed')
                elif self.shape=='circle':
                    self.trackers=[PurePursuit(path,laps=self.laps,speed=min(self.speed,control['max_linear']),lookahead=self.lookahead,
                                              max_angular=control['max_angular'],goal_tolerance=control['position_tolerance'],
                                              max_error=self.max_error) for path in self.paths]
                    self._change('tracking_circle')
                else:self._change('tracking_edge');self.goals=self._edge_goals()
                return [(0,0,'group_barrier')]*3
            return commands
        if self.stage=='tracking_circle':
            try:commands=[(*tracker.step(pose),'tracking_circle') for tracker,pose in zip(self.trackers,poses)]
            except ValueError as exc:raise LoopFault(str(exc).replace('disable and reset','restart launch'))
            progress=[tracker.progress for tracker in self.trackers]
            self.common_progress=min(progress);self.errors=[t.error for t in self.trackers]
            self._stall(self.common_progress,dt)
            if all(t.completed for t in self.trackers):
                self._change('finishing');self.goals=list(self.starts)
                return [(0,0,'group_barrier')]*3
            return self._scale(commands,progress)
        if self.stage=='tracking_edge':
            self.goals=self._edge_goals()
            result=[straight_velocity(p,self.starts[(i+self.rounds_completed)%3],self.goals[i],
                                      min(self.speed,control['max_linear']),control['max_angular'],self.lookahead,
                                      control['position_tolerance'],self.max_error,control['k_heading'],control['k_distance'])
                    for i,p in enumerate(poses)]
            commands=[r[0] for r in result];progress=[r[1] for r in result]
            self.common_progress=self.rounds_completed*self.side_length+min(progress)
            self.errors=[r[2] for r in result]
            self._stall(self.common_progress,dt)
            ready=all(phase=='edge_arrived' for _,_,phase in commands)
            if velocities is not None:ready=ready and all(abs(v)<.02 and abs(w)<.05 for v,w in velocities)
            if self._hold(ready,dt):
                self.goals=self._edge_goals(next_heading=True)
                self.rounds_completed+=1;self._change('turning')
                return [(0,0,'group_barrier')]*3
            return self._scale(commands,progress)
        if self.stage=='turning':
            commands=[];ready=True;error=0
            for pose,goal in zip(poses,self.goals):
                if math.hypot(pose[0]-goal[0],pose[1]-goal[1])>max(.08,2*control['position_tolerance']):
                    raise LoopFault('robot moved away from triangle vertex during turn')
                yaw_error=angle(goal[2]-pose[2]);error+=abs(yaw_error)*.08
                arrived=abs(yaw_error)<=control['yaw_tolerance'];ready=ready and arrived
                w=0 if arrived else max(-control['max_angular'],min(control['max_angular'],control['k_heading']*yaw_error))
                commands.append((0,w,'vertex_turn'))
            self._stall(-error,dt)
            if velocities is not None:ready=ready and all(abs(v)<.02 and abs(w)<.05 for v,w in velocities)
            if self._hold(ready,dt):
                if self.laps and self.rounds_completed>=3*self.laps:
                    self.completed=True;self._change('completed')
                else:self._change('tracking_edge');self.goals=self._edge_goals()
                return [(0,0,'group_barrier')]*3
            return commands
        raise LoopFault('Unknown coordinated stage')
