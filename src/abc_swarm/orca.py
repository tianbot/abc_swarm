"""ORCA pair constraints and feasible front-point velocities for differential drive.

Pair construction adapted from RVO2 Agent.cc, commit
75822bba14462c2e872ca96423825017ecd24764 (Apache-2.0).
Copyright (c) 2008 University of North Carolina at Chapel Hill.
Local additions: exact small 2-D convex projection, body-velocity constraints,
conservative front-point envelope and fail-closed infeasibility handling.
See docs/orca.md and licenses/orca for source and license details.
SPDX-License-Identifier: Apache-2.0
"""
import math
from .point_control import point_velocity, validate_pose

EPS = 1e-9


class OrcaBlocked(ValueError):
    """The configured envelopes or velocity constraints cannot safely be used."""


class OrcaInfeasible(OrcaBlocked):
    """No velocity currently satisfies all planes and actuator bounds."""


def dot(a,b):
    return a[0]*b[0]+a[1]*b[1]


def add(a,b):
    return a[0]+b[0],a[1]+b[1]


def sub(a,b):
    return a[0]-b[0],a[1]-b[1]


def mul(a,k):
    return a[0]*k,a[1]*k


def det(a,b):
    return a[0]*b[1]-a[1]*b[0]


def project_velocity(preferred, constraints, radius):
    """Closest velocity with |v|<=radius and n.v>=b for every (n,b).

    For this 2-D problem the optimum is either the unconstrained disk projection,
    or the projection onto a feasible segment of one constraint boundary. Unlike
    a relaxed LP fallback, infeasibility returns None without breaking a plane.
    """
    if not math.isfinite(radius) or radius<=0:
        raise ValueError('Velocity radius must be positive')
    lines=[]
    for normal,bound in constraints:
        length=math.hypot(*normal)
        if length<EPS or not all(math.isfinite(x) for x in (*normal,bound)):
            raise ValueError('Invalid velocity half-plane')
        lines.append((mul(normal,1/length),bound/length))
    if not all(math.isfinite(x) for x in preferred):
        raise ValueError('Nonfinite preferred velocity')
    def feasible(v):
        return dot(v,v)<=radius*radius+EPS and all(dot(n,v)>=b-EPS for n,b in lines)
    length=math.hypot(*preferred)
    candidates=[]
    base=preferred if length<=radius else mul(preferred,radius/length)
    if feasible(base):candidates.append(base)
    for normal,bound in lines:
        if abs(bound)>radius+EPS:continue
        p=mul(normal,bound);direction=(-normal[1],normal[0])
        extent=math.sqrt(max(0,radius*radius-bound*bound));low,high=-extent,extent
        for n,b in lines:
            slope=dot(n,direction);rhs=b-dot(n,p)
            if abs(slope)<EPS:
                if rhs>EPS:low,high=1,0;break
            elif slope>0:low=max(low,rhs/slope)
            else:high=min(high,rhs/slope)
        if low<=high+EPS:
            t=max(low,min(high,dot(sub(preferred,p),direction)))
            v=add(p,mul(direction,t))
            if feasible(v):candidates.append(v)
    return min(candidates,key=lambda v:dot(sub(v,preferred),sub(v,preferred))) if candidates else None


def pair_plane(position,velocity,other_position,other_velocity,combined_radius,horizon):
    """RVO2 agent-agent ORCA half-plane, equal reciprocal responsibility."""
    p=sub(other_position,position);v=sub(velocity,other_velocity)
    dist2=dot(p,p);r2=combined_radius*combined_radius
    if dist2<=r2+EPS:
        raise OrcaBlocked('effective envelopes overlap; separate robots and restart')
    w=sub(v,mul(p,1/horizon));w2=dot(w,w);wp=dot(w,p)
    if wp<0 and wp*wp>r2*w2:
        length=math.sqrt(w2);unit=mul(w,1/length)
        direction=(unit[1],-unit[0])
        correction=mul(unit,combined_radius/horizon-length)
    else:
        leg=math.sqrt(dist2-r2)
        if det(p,w)>0:
            direction=((p[0]*leg-p[1]*combined_radius)/dist2,
                       (p[0]*combined_radius+p[1]*leg)/dist2)
        else:
            direction=(-(p[0]*leg+p[1]*combined_radius)/dist2,
                       -(-p[0]*combined_radius+p[1]*leg)/dist2)
        correction=sub(mul(direction,dot(v,direction)),v)
    normal=(-direction[1],direction[0])
    return normal,dot(normal,add(velocity,mul(correction,0.5)))


def front_point(pose,distance):
    return pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2])


def front_velocity(yaw,body_velocity,distance):
    v,w=body_velocity;c,s=math.cos(yaw),math.sin(yaw)
    return v*c-distance*w*s,v*s+distance*w*c


def orca_velocities(poses,goals,velocities,control,robot_radius=0.13,safety_margin=0.02,
                    front_distance=0.08,time_horizon=3.0,right_bias=0.01,velocity_buffer=0.002,
                    preferred_commands=None):
    """Return body (v,w,phase) for the whole group, or raise before any output.

    |v| and |w| bounds are half-planes inside the solve; no post-solve clipping.
    The front-point disk radius r+d+margin contains the physical axle disk.
    Inputs are actual measured body twists, not previous requested commands.
    """
    if not poses or len(poses)!=len(goals) or len(poses)!=len(velocities):
        raise ValueError('Each robot needs a pose, goal and measured velocity')
    for value in [robot_radius,front_distance,time_horizon]:
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('ORCA geometry and horizon must be positive and finite')
    for value in [safety_margin,right_bias,velocity_buffer]:
        if type(value) not in (int,float) or not math.isfinite(value) or value<0:
            raise ValueError('ORCA margin/bias must be finite and nonnegative')
    raw=[point_velocity(p,g,**control) for p,g in zip(poses,goals)]
    if preferred_commands is not None:
        if len(preferred_commands)!=len(poses) or any(
                len(c)!=3 or not all(math.isfinite(x) for x in c[:2]) or not isinstance(c[2],str)
                for c in preferred_commands):
            raise ValueError('One finite preferred command is required per pose')
        raw=preferred_commands
    for pose in poses:validate_pose(pose)
    if any(not isinstance(v,(list,tuple)) or len(v)!=2 or
           not all(type(x) in (int,float) and math.isfinite(x) for x in v) for v in velocities):
        raise ValueError('Nonfinite measured velocity')
    radius=robot_radius+safety_margin+front_distance
    centers=[front_point(p,front_distance) for p in poses]
    measured=[front_velocity(p[2],v,front_distance) for p,v in zip(poses,velocities)]
    # Conflicting final target disks cannot be solved by local avoidance.
    target_centers=[front_point(g,front_distance) for g in goals]
    for i in range(len(poses)):
        for j in range(i):
            if math.hypot(*sub(target_centers[i],target_centers[j]))<=2*radius+2*velocity_buffer*time_horizon:
                raise OrcaBlocked('target effective envelopes overlap; change target poses')
    result=[];max_v=control['max_linear'];max_w=control['max_angular']
    for i,(pose,(v,w,phase)) in enumerate(zip(poses,raw)):
        e=(math.cos(pose[2]),math.sin(pose[2]));side=(-e[1],e[0])
        planes=[(e,-max_v),(mul(e,-1),-max_v),
                (side,-front_distance*max_w),(mul(side,-1),-front_distance*max_w)]
        for j in range(len(poses)):
            if i!=j:
                normal,bound=pair_plane(centers[i],measured[i],centers[j],measured[j],2*radius,time_horizon)
                planes.append((normal,bound+velocity_buffer))
        preferred=front_velocity(pose[2],(v,w),front_distance)
        # Deterministic passing preference resolves an exact head-on symmetry.
        # The bias is only a preference; it never relaxes collision constraints.
        if v>0 and any(math.hypot(*sub(centers[i],c))<2.0 for j,c in enumerate(centers) if j!=i):
            preferred=add(preferred,mul(side,-min(right_bias,0.2*v)))
        safe=project_velocity(preferred,planes,math.hypot(max_v,front_distance*max_w))
        if safe is None:raise OrcaInfeasible('no feasible ORCA velocity within differential-drive limits')
        body=(dot(safe,e),dot(safe,side)/front_distance)
        result.append((*body,'avoiding' if phase=='arrived' and math.hypot(*safe)>1e-6 else phase))
    return result
