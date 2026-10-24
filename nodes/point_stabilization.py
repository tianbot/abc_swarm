#!/usr/bin/env python3
"""Multi-robot mocap pose stabilization. Optional ORCA participant avoidance, without operator arming."""
import json
import math
import re
import time
import threading

import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from dynamic_reconfigure.server import Server
from abc_swarm.cfg import point_controlConfig

from abc_swarm.coordinated_loop import CoordinatedLoop, LoopFault, loop_geometry
from abc_swarm.orca import OrcaBlocked, orca_velocities
from abc_swarm.feedback import odometry_error
from abc_swarm.point_control import formation_targets, point_velocity, validate_pose, world_goals


class PointStabilization:
    def __init__(self):
        self.fixed = rospy.get_param('~fixed_frame', 'world')
        self.task = rospy.get_param('~task', 'point')
        if self.task not in ('point','circle','triangle'):
            raise ValueError('task must be point, circle or triangle')
        self.loop = None
        self.loop_options = None
        self.loop_reference = 'initial_leader'
        self.loop_origin = (0,0,0)
        self.last_step_time = None
        if self.task != 'point':
            self.loop_options = rospy.get_param('~coordinated')
            expected = {'diameter','side_length','clockwise','laps','speed','reference','origin',
                        'lookahead','phase_window','settle_time','max_error','progress_timeout'}
            if not isinstance(self.loop_options,dict) or set(self.loop_options)!=expected:
                raise ValueError('Coordinated config fields do not match the launch contract')
            self.loop_options = dict(self.loop_options)
            self.loop_reference = self.loop_options.pop('reference')
            self.loop_origin = self.loop_options.pop('origin')
            if self.loop_reference not in ('world','initial_leader'):
                raise ValueError('Coordinated reference must be world or initial_leader')
            validate_pose(self.loop_origin)
            if self.loop_reference=='initial_leader' and any(value!=0 for value in self.loop_origin):
                raise ValueError('Configure reference=world to set an explicit origin')
            CoordinatedLoop(self.task,origin=self.loop_origin,**self.loop_options)  # Validate before publishers.

        self.robots = rospy.get_param('~robots')
        self.formation = rospy.get_param('~formation', 'config')
        if self.task != 'point':
            self.formation = self.task
            self.targets = loop_geometry(self.task, self.loop_options['diameter'], self.loop_options['side_length'],
                                         self.loop_options['clockwise'])[0]
            self.reference = self.loop_reference
        elif self.formation == 'config':
            self.targets = rospy.get_param('~target_poses')
            self.reference = rospy.get_param('~target_reference', 'initial_leader')
        else:
            self.targets = formation_targets(self.formation, rospy.get_param('~spacing', 0.5))
            self.reference = 'initial_leader'
        self.timeout = rospy.get_param('~tf_timeout', 0.5)
        self.control = {key: rospy.get_param('~' + key, default) for key, default in
                        [('max_linear', 0.15), ('max_angular', 0.5),
                         ('position_tolerance', 0.04), ('yaw_tolerance', 0.08),
                         ('k_distance', 0.9), ('k_heading', 2.0)]}
        if type(self.timeout) not in (int, float) or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('tf_timeout must be positive and finite')
        if not isinstance(self.robots, list) or not self.robots:
            raise ValueError('robots must be a nonempty list')
        for robot in self.robots:
            if not isinstance(robot, dict) or set(robot) != {'name', 'frame'}:
                raise ValueError('Each robot requires exactly name and frame')
            for value in robot.values():
                if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', value):
                    raise ValueError('Robot name/frame must be flat ROS identifiers')
            if robot['frame'] == self.fixed:
                raise ValueError('Robot frame cannot equal fixed frame')
        for key in ('name', 'frame'):
            if len({robot[key] for robot in self.robots}) != len(self.robots):
                raise ValueError('Duplicate robot ' + key)
        if not isinstance(self.targets, (list, tuple)) or len(self.targets) != len(self.robots):
            raise ValueError('One explicit target pose is required for every robot')
        if self.task != 'point' and len(self.robots)!=3:
            raise ValueError('Coordinated loops require exactly three robots')
        world_goals(self.targets, self.reference, (0, 0, 0))
        point_velocity((0, 0, 0), (0, 0, 0), **self.control)
        self.avoidance = rospy.get_param('~avoidance', 'none')
        if self.avoidance not in ('none', 'orca'):
            raise ValueError('avoidance must be none or orca')
        self.orca = {}
        self.feedback = {}
        self.feedback_lock = threading.Lock()
        self.avoidance_fault = ''
        self.progress_time = None
        self.best_error = None
        if self.avoidance == 'orca':
            cfg = rospy.get_param('~orca')
            defaults = dict(robot_radius=0.13, safety_margin=0.02, front_distance=0.08,
                            time_horizon=3.0, right_bias=0.01, velocity_buffer=0.002)
            if not isinstance(cfg, dict) or set(cfg) != set(defaults) | {'odom_timeout','odom_future_tolerance','stall_timeout'}:
                raise ValueError('ORCA config must contain exactly the fields in config/orca.yaml')
            for key, value in cfg.items():
                if type(value) not in (int,float) or not math.isfinite(value) or value < 0:
                    raise ValueError('Invalid ORCA value: ' + key)
                if key not in ('safety_margin','right_bias','velocity_buffer') and value <= 0:
                    raise ValueError('ORCA value must be positive: ' + key)
            self.orca = {key:cfg[key] for key in defaults}
            self.odom_timeout = cfg['odom_timeout']
            self.future_tolerance = cfg['odom_future_tolerance']
            self.stall_timeout = cfg['stall_timeout']
            if self.task != 'point':
                nominal = math.sqrt(3)*self.loop_options['diameter']/2 if self.task=='circle' else self.loop_options['side_length']/2
                reserve = 2*(cfg['robot_radius']+cfg['safety_margin']+cfg['front_distance']) + self.loop_options['phase_window'] + 2*cfg['velocity_buffer']*cfg['time_horizon']
                if nominal <= reserve:
                    raise ValueError('Loop geometry cannot fit the configured ORCA envelopes and progress spacing')
        self.listener = tf.TransformListener()
        self.outputs = [rospy.Publisher('/' + r['name'] + '/cmd_vel', Twist, queue_size=1)
                        for r in self.robots]
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.goals_pub = rospy.Publisher('~goals', PoseArray, queue_size=1, latch=True)
        self.path_outputs = [rospy.Publisher('~paths/'+r['name'], Path, queue_size=1, latch=True)
                             for r in self.robots] if self.task != 'point' else []
        self.goals = None
        self.completed = False
        self.final_details = []
        self.last_status = None
        self.last_publish = 0
        rospy.on_shutdown(self.stop)
        self.tuning = Server(point_controlConfig, self.configure)
        self.odom_subscribers = []
        if self.avoidance == 'orca':
            self.odom_subscribers = [rospy.Subscriber('/'+robot['name']+'/odom', Odometry,
                                     self.odom_cb, callback_args=robot['name'], queue_size=1)
                                     for robot in self.robots]

    def odom_cb(self, msg, name):
        with self.feedback_lock:
            self.feedback[name] = (msg.header.stamp.to_sec(), time.monotonic(),
                                   msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def measured_velocities(self):
        with self.feedback_lock:
            samples = dict(self.feedback)
        velocities = []
        for robot in self.robots:
            sample = samples.get(robot['name'])
            if sample is None:
                raise ValueError('waiting for odometry: /'+robot['name']+'/odom')
            stamp, received, v, w = sample
            issue = odometry_error(stamp, rospy.Time.now().to_sec(), time.monotonic()-received,
                                   v,w,self.odom_timeout,self.future_tolerance)
            if issue:
                raise ValueError('/'+robot['name']+'/odom: '+issue)
            velocities.append((v,w))
        return velocities

    def configure(self, config, level):
        gains = (config['k_distance'], config['k_heading'])
        if not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in gains):
            for key in ('k_distance', 'k_heading'):
                config[key] = self.control[key]
            return config
        # Replace the complete settings snapshot; a control cycle uses one version.
        self.control = dict(self.control, k_distance=gains[0], k_heading=gains[1])
        return config

    def stop(self):
        for output in self.outputs:
            output.publish(Twist())

    def poses(self):
        poses = []
        for robot in self.robots:
            frame = robot['frame']
            stamp = self.listener.getLatestCommonTime(self.fixed, frame)
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp == rospy.Time(0) or not 0 <= age <= self.timeout:
                raise ValueError('stale pose: ' + frame)
            xyz, q = self.listener.lookupTransform(self.fixed, frame, stamp)
            if not all(math.isfinite(v) for v in (*xyz, *q)):
                raise ValueError('invalid pose: ' + frame)
            pose = (xyz[0], xyz[1], tf.transformations.euler_from_quaternion(q)[2])
            validate_pose(pose)
            poses.append(pose)
        return poses

    def publish_goals(self):
        msg = PoseArray()
        msg.header.frame_id, msg.header.stamp = self.fixed, rospy.Time.now()
        for x, y, yaw in self.goals:
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = q
            msg.poses.append(pose)
        self.goals_pub.publish(msg)

    def status(self, state, reason, details=None):
        value = json.dumps({'schema_version': 2, 'mode': 'point_stabilization' if self.task=='point' else 'coordinated_tracking',
                            'task': self.task, 'stage': self.loop.stage if self.loop is not None else (state if self.task=='point' else 'waiting'),
                            'common_progress': self.loop.common_progress if self.loop is not None else 0.0,
                            'progress_spread': self.loop.progress_spread if self.loop is not None else 0.0,
                            'edge_rounds_completed': self.loop.rounds_completed if self.loop is not None else 0,
                            'avoidance': self.avoidance,
                            'avoidance_scope': 'point' if self.task=='point' else 'assembly',
                            'avoidance_active': self.avoidance=='orca' and state=='running' and (self.task=='point' or (self.loop is not None and self.loop.stage=='assembling')),
                            'state': state, 'ready': state in ('running', 'completed'),
                            'enabled': state == 'running', 'reason': reason,
                            'robots': details or [], 'target_reference': self.reference, 'formation': self.formation,
                            'vrpn_server': rospy.get_param('/vrpn_client_node/server', ''),
                            'vrpn_port': rospy.get_param('/vrpn_client_node/port', 0)}, sort_keys=True)
        # Numeric errors change continuously; cap status updates at 5 Hz.
        if value != self.last_status and time.monotonic() - self.last_publish >= 0.2:
            self.status_pub.publish(String(data=value))
            self.last_status, self.last_publish = value, time.monotonic()

    def publish_paths(self):
        for points,output in zip(self.loop.paths,self.path_outputs):
            path = Path();path.header.frame_id = self.fixed;path.header.stamp = rospy.Time.now()
            for i,(x,y) in enumerate(points):
                msg = PoseStamped();msg.header = path.header
                msg.pose.position.x,msg.pose.position.y = x,y
                nxt = points[(i+1) % (len(points)-1)]
                yaw = math.atan2(nxt[1]-y,nxt[0]-x)
                q = tf.transformations.quaternion_from_euler(0,0,yaw)
                (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w) = q
                path.poses.append(msg)
            output.publish(path)

    def step_coordinated(self,poses,measured):
        if self.loop is None:
            origin = poses[0] if self.loop_reference=='initial_leader' else self.loop_origin
            self.loop = CoordinatedLoop(self.task,origin=origin,**self.loop_options)
            self.publish_paths()
        now = time.monotonic()
        dt = .05 if self.last_step_time is None else max(.001,min(.1,now-self.last_step_time))
        self.last_step_time = now
        commands = self.loop.step(poses,self.control,dt,measured,
                                  self.orca if self.avoidance=='orca' else None)
        if self.goals != self.loop.goals:
            self.goals = list(self.loop.goals);self.publish_goals()
        self.completed = self.loop.completed
        details = []
        for robot,pose,goal,(v,w,phase),output,error in zip(
                self.robots,poses,self.goals,commands,self.outputs,self.loop.errors):
            cmd = Twist();cmd.linear.x,cmd.angular.z = v,w;output.publish(cmd)
            details.append({'name':robot['name'],'frame':robot['frame'],'goal':goal,'phase':phase,
                            'position_error':math.hypot(goal[0]-pose[0],goal[1]-pose[1]),
                            'yaw_error':math.atan2(math.sin(goal[2]-pose[2]),math.cos(goal[2]-pose[2])),
                            'path_error':error,'command':[v,w]})
        self.status('completed' if self.completed else 'running',
                    'coordinated '+self.task+': '+self.loop.stage,details)

    def step(self):
        try:
            if self.avoidance_fault:
                self.stop()
                self.status('fault', self.avoidance_fault)
                return
            poses = self.poses()  # Validate the entire group before publishing any motion.
            measured = self.measured_velocities() if self.avoidance == 'orca' else None
            if self.task != 'point':
                self.step_coordinated(poses,measured)
                return
            if self.goals is None:
                self.goals = world_goals(self.targets, self.reference, poses[0])
                self.publish_goals()
            if self.completed:
                self.stop()
                self.status('completed', 'all target poses reached; restart launch for another run', self.final_details)
                return
            control = self.control
            if self.avoidance == 'orca':
                results = orca_velocities(poses, self.goals, measured, control, **self.orca)
                error = sum(math.hypot(g[0]-p[0],g[1]-p[1]) + self.orca['front_distance'] *
                            abs(math.atan2(math.sin(g[2]-p[2]),math.cos(g[2]-p[2])))
                            for p,g in zip(poses,self.goals))
                if self.best_error is None or error < self.best_error-0.005:
                    self.best_error, self.progress_time = error, time.monotonic()
                if time.monotonic()-self.progress_time > self.stall_timeout:
                    raise OrcaBlocked('ORCA progress timeout; inspect geometry and restart')
            else:
                results = [point_velocity(pose, goal, **control)
                           for pose, goal in zip(poses, self.goals)]
            self.completed = all(phase == 'arrived' for _, _, phase in results)
            details = []
            for robot, pose, goal, (v, w, phase), output in zip(
                    self.robots, poses, self.goals, results, self.outputs):
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                output.publish(cmd)
                details.append({'name': robot['name'], 'frame': robot['frame'], 'goal': goal,
                                'phase': phase, 'position_error': math.hypot(goal[0]-pose[0], goal[1]-pose[1]),
                                'yaw_error': math.atan2(math.sin(goal[2]-pose[2]), math.cos(goal[2]-pose[2]))})
            self.final_details = details
            self.status('completed' if self.completed else 'running',
                        'all target poses reached' if self.completed else 'tracking frozen target poses', details)
        except (OrcaBlocked, LoopFault) as exc:
            self.avoidance_fault = str(exc)
            self.stop()
            self.status('fault', self.avoidance_fault)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, ValueError) as exc:
            self.last_step_time = None
            if self.avoidance == 'orca' and self.progress_time is not None:
                self.progress_time = time.monotonic()  # Feedback downtime is not planning stall time.
            self.stop()
            self.status('waiting', str(exc))

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('point_stabilization')
    PointStabilization().run()
