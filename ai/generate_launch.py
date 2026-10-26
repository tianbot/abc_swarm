#!/usr/bin/env python3
"""Offline, deterministic experiment compiler. No ROS imports or process execution."""
import argparse
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

HERE = Path(__file__).resolve().parent
CATALOG = json.loads((HERE / 'capabilities.json').read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names, where):
    require(isinstance(value, dict), where + ': expected an object')
    expected = set(names.split())
    require(set(value) == expected,
            '{}: missing={}, unknown={}'.format(where, sorted(expected - set(value)),
                                               sorted(set(value) - expected)))


def number(value, where, positive=False):
    require(type(value) in (int, float) and math.isfinite(value), where + ': finite number required')
    if positive:
        require(value > 0, where + ': must be positive')


def identifier(value, where):
    require(isinstance(value, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', value),
            where + ': use a flat ROS name, letters/digits/underscore, starting with a letter')


def orca_settings(value):
    fields(value,'robot_radius safety_margin front_distance time_horizon right_bias velocity_buffer odom_timeout odom_future_tolerance stall_timeout','orca')
    for key,v in value.items():
        number(v,'orca.'+key,positive=key not in ('safety_margin','right_bias','velocity_buffer'))
        require(v>=0,'orca.'+key+': cannot be negative')


def validate(spec):
    hint = CATALOG['experiments'].get(spec.get('experiment'), {}) if isinstance(spec, dict) and isinstance(spec.get('experiment'), str) else {}
    point = hint.get('point', False)
    coordinated = hint.get('coordinated')
    orca = hint.get('orca', False) or bool(coordinated and spec.get('orca') is not None)
    fields(spec, 'contract_version name localization fixed_frame vrpn robots experiment path startup'
           + (' trajectory orca' if coordinated else (' targets' if point else ' spacing') + (' orca' if orca else ''))
           + (' mocap_control' if isinstance(spec,dict) and 'mocap_control' in spec else '')
           + (' admission' if isinstance(spec,dict) and 'admission' in spec else ''), 'experiment')
    require(type(spec['contract_version']) is int and spec['contract_version'] == 2, 'contract_version: expected 2 (automatic startup; review v1 input before migrating)')
    identifier(spec['name'], 'name')
    for key in ('localization', 'fixed_frame'):
        require(spec[key] == CATALOG[key], key + ': currently only ' + CATALOG[key])
    require(isinstance(spec['experiment'], str) and spec['experiment'] in CATALOG['experiments'],
            'experiment: unsupported; see ai/capabilities.json')
    capability = CATALOG['experiments'][spec['experiment']]
    require(isinstance(spec['robots'], list) and len(spec['robots']) == capability['robots'],
            'robots: {} requires exactly {} devices; arbitrary fleet sizes are not implemented'.format(
                spec['experiment'], capability['robots']))
    for i, robot in enumerate(spec['robots']):
        where = 'robots[{}]'.format(i)
        fields(robot, 'name model rigid_body initial_pose', where)
        require(robot['model'] == CATALOG['model'], where + ': unsupported motion model; an adapter is required')
        for key in ('name', 'rigid_body'):
            identifier(robot[key], where + '.' + key)
        require(robot['rigid_body'] != spec['fixed_frame'], where + ': rigid body cannot equal world')
        pose = robot['initial_pose']
        require(pose is None or isinstance(pose, list) and len(pose) == 3,
                where + '.initial_pose: null or [x_m, y_m, yaw_rad]')
        if pose is not None:
            for val in pose:
                number(val, where + '.initial_pose')
    for key in ('name', 'rigid_body'):
        require(len({r[key] for r in spec['robots']}) == len(spec['robots']), 'robots: duplicate ' + key)
    if 'admission' in spec:
        require(capability.get('startup_admission',False),'admission: unsupported for this experiment')
        a=spec['admission']
        require(isinstance(a,dict) and type(a.get('enabled')) is bool,'admission.enabled: boolean required')
        fields(a,'enabled settings orca' if a['enabled'] else 'enabled','admission')
        if a['enabled']:
            fields(a['settings'],'handoff_distance clearance position_margin handoff_time settle_time progress_timeout','admission.settings')
            for key,v in a['settings'].items():number(v,'admission.'+key,positive=True)
            orca_settings(a['orca'])
            d=a['orca']['front_distance'];spacing=spec['spacing']
            number(spacing,'spacing',positive=True)
            local=([(0,0,0),(-spacing,spacing,0),(-spacing,-spacing,0)] if capability['formation']=='triangle'
                   else [(0,0,0),(-spacing,0,0),(-2*spacing,0,0)])
            reserve=2*(a['orca']['robot_radius']+a['orca']['safety_margin']+d)+2*a['orca']['velocity_buffer']*a['orca']['time_horizon']
            require(all(math.hypot(p[0]-q[0],p[1]-q[1])>reserve for i,p in enumerate(local) for q in local[:i]),
                    'admission: target effective envelopes overlap')
    if 'mocap_control' in spec:
        require(capability.get('mocap_control',False), 'mocap_control: unsupported for this experiment')
        require(spec.get('admission',{}).get('enabled',True), 'mocap_control requires admission enabled for follower demos')
        control=spec['mocap_control']
        fields(control,'velocity_source position_tolerance calibration','mocap_control')
        require(control['velocity_source'] in ('odom','mocap'), 'mocap_control.velocity_source: odom or mocap')
        number(control['position_tolerance'],'mocap_control.position_tolerance',positive=True)
        require(.02<=control['position_tolerance']<=.15, 'mocap_control.position_tolerance: 0.02..0.15 m')
        calibration=control['calibration']
        if calibration is not None:
            fields(calibration,' '.join(r['name'] for r in spec['robots']),'mocap_control.calibration')
            for name,setting in calibration.items():
                fields(setting,'yaw_offset marker_offset','calibration.'+name)
                number(setting['yaw_offset'],'calibration.yaw_offset')
                require(isinstance(setting['marker_offset'],list) and len(setting['marker_offset'])==2,
                        'calibration.marker_offset: [forward_m,left_m] required')
                for value in setting['marker_offset']:number(value,'calibration.marker_offset')
        if spec.get('admission',{}).get('enabled'):
            a=spec['admission'];o=a['orca'];tol=control['position_tolerance']
            require(a['settings']['handoff_distance']>=tol,'admission: handoff distance must cover final tolerance')
            target_distance=spec['spacing']*(math.sqrt(2) if capability['formation']=='triangle' else 1)
            reserve=2*(o['robot_radius']+o['safety_margin']+tol+a['settings']['position_margin'])+a['settings']['clearance']
            require(target_distance>reserve,'admission: fine positioning capture regions cannot fit formation')
    fields(spec['vrpn'], 'server port', 'vrpn')
    try:
        ipaddress.IPv4Address(spec['vrpn']['server'])
    except (ValueError, TypeError, ipaddress.AddressValueError):
        raise ValueError('vrpn.server: explicit IPv4 address required, no substitutions')
    require(isinstance(spec['vrpn']['server'], str), 'vrpn.server: string required')
    require(type(spec['vrpn']['port']) is int and 1 <= spec['vrpn']['port'] <= 65535, 'vrpn.port: 1..65535')
    if not point and not coordinated:
        number(spec['spacing'], 'spacing', positive=True)
    fields(spec['startup'], 'start_driver start_vrpn use_rviz start_teleop', 'startup')
    for key, value in spec['startup'].items():
        require(type(value) is bool, 'startup.' + key + ': boolean required')
    if coordinated:
        require(not spec['startup']['start_teleop'], 'coordinated tracking cannot start teleop')
        trajectory = spec['trajectory']
        dimension = 'diameter' if coordinated=='circle' else 'side_length'
        fields(trajectory, dimension + ' speed laps clockwise reference origin', 'trajectory')
        number(trajectory[dimension], 'trajectory.'+dimension, positive=True)
        require(trajectory[dimension]>=1.5, 'trajectory size must be at least 1.5 m')
        number(trajectory['speed'], 'trajectory.speed', positive=True)
        require(trajectory['speed']<=0.15, 'trajectory.speed must not exceed 0.15 m/s')
        require(type(trajectory['laps']) is int and trajectory['laps']>=1, 'trajectory.laps: positive integer required')
        require(type(trajectory['clockwise']) is bool, 'trajectory.clockwise must be boolean')
        require(trajectory['reference'] in ('world','initial_leader'), 'trajectory.reference: world or initial_leader')
        require(isinstance(trajectory['origin'],list) and len(trajectory['origin'])==3, 'trajectory.origin: [x_m,y_m,yaw_rad] required')
        for value in trajectory['origin']:number(value,'trajectory.origin')
        require(trajectory['reference']=='world' or trajectory['origin']==[0,0,0], 'trajectory.origin requires reference=world')
    if point:
        require(not spec['startup']['start_teleop'], 'point stabilization cannot start teleop')
        fields(spec['targets'], 'reference poses', 'targets')
        require(spec['targets']['reference'] in ('world', 'initial_leader'), 'targets.reference: world or initial_leader')
        poses = spec['targets']['poses']
        require(isinstance(poses, list) and len(poses) == len(spec['robots']), 'targets: one pose per robot required')
        for pose in poses:
            require(isinstance(pose, list) and len(pose) == 3, 'targets.poses: [x_m, y_m, yaw_rad] required')
            for value in pose:
                number(value, 'targets.poses')
    if orca:
        fields(spec['orca'], 'robot_radius safety_margin front_distance time_horizon right_bias velocity_buffer odom_timeout odom_future_tolerance stall_timeout', 'orca')
        for key, value in spec['orca'].items():
            number(value, 'orca.' + key, positive=key not in ('safety_margin','right_bias','velocity_buffer'))
            require(value >= 0, 'orca.' + key + ': cannot be negative')
        d = spec['orca']['front_distance']
        if coordinated:
            # Start-goal spacing lower bound; actual assembly still checks measured geometry.
            size = spec['trajectory']['diameter' if coordinated=='circle' else 'side_length']
            sign = -1 if spec['trajectory']['clockwise'] else 1
            if coordinated=='circle':
                local = [(size/2*math.sin(2*math.pi*i/3), sign*size/2*(1-math.cos(2*math.pi*i/3)), sign*2*math.pi*i/3) for i in range(3)]
            else:
                local = [(0,0,0),(size,0,sign*2*math.pi/3),(size/2,sign*math.sqrt(3)*size/2,sign*4*math.pi/3)]
            centers = [(x+d*math.cos(yaw),y+d*math.sin(yaw)) for x,y,yaw in local]
        else:
            centers = [(x+d*math.cos(yaw),y+d*math.sin(yaw)) for x,y,yaw in spec['targets']['poses']]
        clearance = 2*(spec['orca']['robot_radius']+spec['orca']['safety_margin']+d) + 2*spec['orca']['velocity_buffer']*spec['orca']['time_horizon']
        if coordinated:
            nominal = math.sqrt(3)*size/2 if coordinated=='circle' else size/2
            require(nominal > clearance+0.10, 'orca: loop spacing cannot fit configured envelopes and phase window')
        require(all(math.hypot(a[0]-b[0],a[1]-b[1])>clearance for i,a in enumerate(centers) for b in centers[:i]),
                'orca: target effective envelopes overlap; separate target poses')
    if capability['path']:
        require(not spec['startup']['start_teleop'], 'automatic path cannot start teleop')
        path = spec['path']
        fields(path, 'shape radius width height speed laps clockwise', 'path')
        require(path['shape'] in ('circle', 'rounded_rectangle'), 'path.shape: circle or rounded_rectangle')
        for key in ('radius', 'width', 'height', 'speed'):
            number(path[key], 'path.' + key, positive=True)
        require(path['speed'] <= 0.15, 'path.speed: maximum supported demo request is 0.15 m/s')
        require(type(path['laps']) is int and path['laps'] >= 1, 'path.laps: positive integer; continuous motion excluded')
        require(type(path['clockwise']) is bool, 'path.clockwise: boolean required')
        if path['shape'] == 'rounded_rectangle':
            require(min(path['width'], path['height']) >= 2 * path['radius'], 'path: width and height must be >= 2 * radius')
    else:
        require(spec['path'] is None, 'manual experiment requires path: null')
    return capability


def text_value(value):
    return str(value).lower() if type(value) is bool else str(value)


def render(spec):
    capability = validate(spec)
    root = ET.Element('launch')
    root.append(ET.Comment(' Generated by abc_swarm ai/generate_launch.py. Feedback readiness starts the experiment automatically. '))
    for key, value in spec['startup'].items():
        ET.SubElement(root, 'arg', name=key, default=text_value(value))
    include = ET.SubElement(root, 'include', file='$(find abc_swarm)/launch/mocap/' + capability['launch'])
    args = {'vrpn_server_ip': spec['vrpn']['server'], 'vrpn_port': spec['vrpn']['port']}
    if capability.get('mode'):
        args['mode']=capability['mode']
    if capability.get('startup_admission'):
        args['startup']=spec.get('admission',{}).get('enabled',True)
    if not capability.get('point') and not capability.get('coordinated'):
        args['spacing'] = spec['spacing']
    for key in spec['startup']:
        if key != 'start_teleop' or not (capability['path'] or capability.get('point') or capability.get('coordinated')):
            args[key] = '$(arg ' + key + ')'
    for i, robot in enumerate(spec['robots'], 1):
        args['robot_name_' + str(i)] = robot['name']
        args['rigid_body_' + str(i)] = robot['rigid_body']
    if capability['path']:
        args.update(spec['path'])
        if capability.get('startup_admission'):
            args['max_linear']=args.pop('speed')
    if capability.get('coordinated'):
        args.update({key:value for key,value in spec['trajectory'].items() if key!='origin'})
        args.update(dict(zip(('origin_x','origin_y','origin_yaw'),spec['trajectory']['origin'])))
        args['avoidance'] = 'orca' if spec['orca'] is not None else 'none'
    if 'mocap_control' in spec:
        args['velocity_source']=spec['mocap_control']['velocity_source']

    for key, value in args.items():
        ET.SubElement(include, 'arg', name=key, value=text_value(value))
    template = HERE.parent / 'launch/mocap' / capability['launch']
    declared = {arg.get('name') for arg in ET.parse(str(template)).getroot().findall('arg')}
    require(set(args) <= declared, 'launch interface changed: undeclared arguments ' + str(sorted(set(args) - declared)))
    if capability.get('point'):
        for key, value in [('target_reference', spec['targets']['reference']),
                           ('target_poses', spec['targets']['poses'])]:
            ET.SubElement(root, 'rosparam', param='/abc_swarm_demo/' + key).text = json.dumps(value)
    if capability.get('orca') or (capability.get('coordinated') and spec['orca'] is not None):
        ET.SubElement(root, 'rosparam', param='/abc_swarm_demo/orca').text = json.dumps(spec['orca'])
    if 'mocap_control' in spec:
        for key,value in [('position_tolerance',spec['mocap_control']['position_tolerance']),
                          ('mocap_calibration',spec['mocap_control']['calibration'] or {})]:
            ET.SubElement(root,'rosparam',param='/abc_swarm_demo/'+key).text=json.dumps(value)
    if spec.get('admission',{}).get('enabled'):
        for key,value in [('admission',spec['admission']['settings']),('orca',spec['admission']['orca'])]:
            ET.SubElement(root,'rosparam',param='/abc_swarm_demo/'+key).text=json.dumps(value)
    if admission_planning_enabled(spec, capability):
        ET.SubElement(root,'param',name='/abc_swarm_demo/planning',value='true',type='bool')
    # Python 3.8 (Noetic) has no ET.indent.
    return minidom.parseString(ET.tostring(root, encoding='utf-8')).toprettyxml(indent='  ')


def admission_planning_enabled(spec, capability):
    return bool(capability.get('admission_planning') and (
        capability.get('startup_admission') and spec.get('admission',{}).get('enabled',True)
        or capability.get('orca') or capability.get('coordinated') and spec.get('orca') is not None))


def planning_report(spec, capability):
    if not admission_planning_enabled(spec, capability):return ''
    return ('\n\n入位规划已启用：分段绕行、逐车让行，目标被占用时可安排临时停车点；'
            'ORCA负责途中互避，到齐后仍检查最终位置和朝向。临时路线可能超出起点/终点包围区域，'
            '启动前须确认周围空场；当前没有墙壁、场地边界或未知障碍模型，也不能修复刚体标定或底盘转向问题。\n')


def report(spec):
    capability = validate(spec)
    if capability.get('coordinated'):
        return loop_report(spec)+mocap_control_report(spec)+planning_report(spec,capability)
    if capability.get('point'):
        return point_report(spec)+mocap_control_report(spec)+planning_report(spec,capability)
    lines = ['# 客户实验运行单', '', '实验：`{}`；入口：`{}`。'.format(spec['experiment'], capability['launch']),
             '设备顺序即角色：01 项为头车；triangle 的 02/03 跟头车，snake 为逐车跟前车。',
             '坐标单位 m/rad，均为 world 下轴心参考点；以下是人工摆位记录，不设置 TF、不移动到起点。', '',
             '| 设备 | 刚体 | 客户记录的起点 [x,y,yaw] | 建议摆位 [x,y,yaw] |',
             '| --- | --- | --- | --- |']
    leader_pose = spec['robots'][0]['initial_pose']
    for i, robot in enumerate(spec['robots']):
        expected = None
        if leader_pose is not None:
            x, y, yaw = leader_pose
            dx = -spec['spacing'] * (i if capability['formation'] == 'snake' else min(i, 1))
            dy = spec['spacing'] * (1 if i == 1 else -1) if capability['formation'] == 'triangle' and i else 0
            expected = [round(x + math.cos(yaw)*dx - math.sin(yaw)*dy, 4),
                        round(y + math.sin(yaw)*dx + math.cos(yaw)*dy, 4), yaw]
        lines.append('| {} | {} | {} | {} |'.format(robot['name'], robot['rigid_body'],
                     robot['initial_pose'] if robot['initial_pose'] is not None else '未知，现场核对',
                     expected if expected is not None else '头车位置未知，现场定起点后摆位'))
    lines += ['', '起点未知或与建议不一致时，先人工摆位并核对实时 TF。路径在启动/重置时捕获头车实测位置；不会采用表中的坐标覆盖实测值。',
              '路径外侧还会扫过编队；生成器没有场地尺寸、障碍物和车辆外形，不作碰撞或可达性保证。',
              '', '## 复核及启动', '', '当前生成只通过离线契约检查，不代表 ROS 环境或实车验收。',
              '确认本机 Noetic 和工作空间已 source，在本目录执行只解析、不启动的检查：', '',
              '```bash', 'roslaunch --nodes ./experiment.launch start_driver:=false start_vrpn:=false use_rviz:=false start_teleop:=false', '```', '',
              '启动前确认网络、刚体映射、摆位和完整编队活动空间。start_driver/start_vrpn 为 false 时表示外部后台已运行，不是取消定位需求。',
              '本机 ROS master 与场地网络配置见 docs/field-test.md。一次只运行一套实验，已存在后台时关闭对应启动项。', '',
              '```bash', 'roslaunch ./experiment.launch', '```', '',
              '另一个已 source 的终端查看状态，反馈有效后自动 running；RViz 显示运行路径，不是启动确认门。', '',
              '```bash', 'rostopic echo /abc_swarm_demo/status', '```', '',
              '启动后反馈有效即自动运行，没有额外的启用 service；后车可能立即纠正位置。',
              '退出实验使用 roslaunch 终端 Ctrl+C，确认全部车辆静止。k 只停止头车键盘请求，后车仍可收敛。',
              '路径完成或故障后，重新启动 launch 可开始新一轮；可选 reset 会重建路径并自动运行。',
              '手动实验在键盘终端 i 前进、u/o 缓弯、k 停止；输入超时会停止。',
              '', '## 验收记录', '', '- 离线生成检查：通过。', '- ROS 解析：待现场执行。',
              '- 动捕、驱动、反馈和停止功能：未测。', '- 实际实验效果：未测。',
              '- 填写日期、仓库提交、现场差异及结果，不能把生成通过记为实车通过。', '']
    if capability.get('startup_admission'):
        if not spec.get('admission',{}).get('enabled',True):
            lines += ['', '显式startup=false：跳过ORCA/精定位入队，使用原始TF及底盘odom直接跟随。', '']
            return '\n'.join(lines)
        lines += ['', '## 自动入队与跟随交接', '',
                  '默认冻结01实测初始位姿及编队目标，ORCA入队 → 受保护的纯点镇定 → 全体位置/朝向/实测速度稳定 → 跟随。',
                  '遥控请求在入队期间被忽略；manual到齐后接收键盘/GUI新请求，auto到齐后头车沿预设路径运行。',
                  '默认速度反馈来自动捕控制中心位姿，支持显式逐车标定。原始动捕TF不修改；身体半径/标记架范围必须现场核对。',
                  '目标区域不能安全分离时保留ORCA；精定位越界会重新避让。它不提供静态障碍或跟随段全程ORCA。',
                  'admission未填写时使用包内admission.yaml及orca.yaml；没有隐含人工启用service。',
                  '显式入队配置：'+json.dumps(spec.get('admission','包内默认配置'),ensure_ascii=False), '']
        lines = [line.replace('不设置 TF、不移动到起点','起点记录不覆盖实测TF；实际启动会自动入队')
                 .replace('起点未知或与建议不一致时，先人工摆位并核对实时 TF。','起点未知时先核对实时TF及入队通道。')
                 .replace('启动/重置','启动')
                 .replace('路径完成或故障后，重新启动 launch 可开始新一轮；可选 reset 会重建路径并自动运行。',
                          '路径完成或故障后，退出并重新启动launch，捕获新的起点；不增加手动启用步骤。') for line in lines]
        return '\n'.join(lines)+mocap_control_report(spec)+planning_report(spec,capability)
    return '\n'.join(lines)


def mocap_control_report(spec):
    if 'mocap_control' not in spec:return ''
    c=spec['mocap_control']
    return ('\n\n## 动捕控制反馈与标定\n\n速度来源：`{}`；最终位置容差：{} m。\n\n'
            'mocap 使用控制中心位姿约0.2秒窗口估计实测速度，不依赖底盘 odom；动捕失效仍全组停车。'
            '标定从原始平面 yaw 加 yaw_offset，再扣除车辆前/左坐标中的轴心到标记原点偏移；原始 TF 不修改。'
            'null 表示刚体已对齐控制中心，使用原始位姿。不能复制其他客户的标定值。\n\n'
            '```json\n{}\n```\n').format(c['velocity_source'],c['position_tolerance'],
                                          json.dumps(c['calibration'],ensure_ascii=False,indent=2))


def point_report(spec):
    avoidance = CATALOG['experiments'][spec['experiment']].get('orca', False)
    feedback = '有效动捕及动捕估计速度' if spec.get('mocap_control',{}).get('velocity_source')=='mocap' else '有效动捕与底盘 odom'
    lines = ['# 三车点镇定运行单' + ('（ORCA 车车避让）' if avoidance else ''), '',
             '目标参考系：`' + spec['targets']['reference'] + '`。world 为绝对目标；initial_leader 在全部定位有效时按首车实测位姿冻结目标。',
             '初始位姿仅供记录，控制目标来自 targets；没有头车领航和后车跟随角色，按设备数组顺序一对一分配。',
             ('启动后自动运行，ORCA 仅避让本实验三车；需要三车'+feedback+'，可能短距离倒车。没有静态障碍感知或自动切入编队。' if avoidance else '启动后自动运行，各车先到目标位置再调整最终航向；没有 ORCA、路径避障或自动切入编队。'), '',
             '| 设备 | 刚体 | 初始位姿记录 | 控制目标 [m,m,rad] |', '| --- | --- | --- | --- |']
    for robot, goal in zip(spec['robots'], spec['targets']['poses']):
        lines.append('| {} | {} | {} | {} |'.format(robot['name'], robot['rigid_body'], robot['initial_pose'], goal))
    lines += ['', ('启动前核对实际车体半径和安全余量，初始有效包络不能重叠；本地避让不保证任意目标可达，不能代替实车验收。' if avoidance else '启动前确认每台车通道互不冲突；禁止把本实验用于任意交叉/密集起点的无碰撞集结。'),
              '加载 Noetic 和对应版本 abc_swarm 工作空间后，在本目录只解析检查：', '',
              '```bash', 'roslaunch --nodes ./experiment.launch start_driver:=false start_vrpn:=false use_rviz:=false', '```', '',
              '客户准备好现场后启动（AI 不代执行）：', '',
              '```bash', 'roslaunch ./experiment.launch', '```', '',
              '```bash', 'rostopic echo /abc_swarm_demo/status', '```', '',
              'state=completed 表示三车位置和航向均到达，之后持续停车；退出用 Ctrl+C 并确认实际静止。',
              ('任意必需反馈失效时全组停车，恢复后继续冻结目标；包络重叠、无可行速度或进展超时会 fault 停车，需检查并重启。' if avoidance else '任意所需位姿缺失/过期时全组停车；恢复定位后继续原冻结目标，不重建起点。'),
              '无启用 service。新一轮需重启 launch；不要同时运行跟随/路径节点或其他 cmd_vel 发布者。',
              '', '离线生成检查通过；ROS 解析、驱动/动捕及实车到位/停车仍须现场记录，不代表已验收。', '']
    return '\n'.join(lines)


def loop_report(spec):
    shape=CATALOG['experiments'][spec['experiment']]['coordinated']
    task=spec['trajectory']
    feedback = '动捕估计实测速度' if spec.get('mocap_control',{}).get('velocity_source')=='mocap' else '有效底盘 odom'
    lines=['# 三车动捕协调路径运行单', '',
           ('任务：同一个圆，三车相隔约120°、同向绕行。' if shape=='circle' else '任务：正三角形三个顶点，每轮各走一条直边，到齐停车并原地转向后一起走下一边。'),
           '参数：'+json.dumps(task,ensure_ascii=False),
           'reference=initial_leader 时，按01启动实际位姿冻结第一起点及切线/第一条边；world 时 origin 是这一参考位姿，不能当成圆心。',
           '流程：全体点镇定到起点/朝向并稳定 → 协调路径跟踪 → 指定圈数完成停车，没有人工启用 service。',
           ('起点点镇定启用 ORCA，需三车'+feedback+'；路径段保持几何跟踪，不提供全程 ORCA 避障。' if spec['orca'] is not None else '起点点镇定未启用 ORCA；请准备互不冲突的摆位通道。路径段没有障碍感知。'), '',
           '| 设备 | 动捕刚体 | 初始位姿记录 |','| --- | --- | --- |']
    for robot in spec['robots']:
        lines.append('| {} | {} | {} |'.format(robot['name'],robot['rigid_body'],robot['initial_pose']))
    lines += ['', '启动前退出旧实验并核对完整活动范围。相位协调会降低快车速度以等待最慢车，0.15为上限。',
              '加载 Noetic 和对应工作空间后，只解析：', '', '```bash',
              'roslaunch --nodes ./experiment.launch start_driver:=false start_vrpn:=false use_rviz:=false','```','',
              '由现场操作人员启动（反馈齐全即自动运动）：','','```bash','roslaunch ./experiment.launch','```','',
              '```bash','rostopic echo /abc_swarm_demo/status','```','',
              'mode=coordinated_tracking；stage 显示 assembling/tracking_circle/tracking_edge/turning/finishing/completed。',
              '任一必需反馈失效全组停车，恢复后继续冻结的任务；偏离路径或无进展会 fault，需检查并重启。',
              '停止用 roslaunch 终端 Ctrl+C，并确认实际静止。离线生成检查通过；ROS/实车验收需另记，不能把坐标示例当成已确认场地。','']
    return '\n'.join(lines)


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key: ' + key)
        result[key] = value
    return result


def bundle(spec):
    # Freeze the compiler, catalog and launch/config composition used for delivery.
    paths = [HERE / 'generate_launch.py', HERE / 'capabilities.json',
             HERE.parent / 'launch/mocap' / validate(spec)['launch']]
    paths += sorted((HERE.parent / 'launch/common').glob('*.launch.xml'))
    paths += [HERE.parent / 'config/mocap.yaml', HERE.parent / 'config/path_tracking.yaml']
    if validate(spec).get('point'):
        paths.extend([HERE.parent / 'config/point_stabilization.yaml',
                      HERE.parent / 'config/point_control.cfg'])
    if validate(spec).get('startup_admission') or validate(spec).get('point') or validate(spec).get('coordinated'):
        paths.extend([HERE.parent/'config/admission.yaml',HERE.parent/'config/point_stabilization.yaml',HERE.parent/'config/orca.yaml'])
    if validate(spec).get('orca'):
        paths.extend([HERE.parent / 'config/orca.yaml', HERE.parent / 'licenses/orca/UPSTREAM.json',
                      HERE.parent / 'launch/mocap/demo_mocap_three_mini_point_stabilization.launch'])
    if validate(spec).get('coordinated'):
        paths.extend([HERE.parent / 'config/coordinated_tracking.yaml',HERE.parent / 'config/point_stabilization.yaml',
                      HERE.parent / 'config/orca.yaml',HERE.parent / 'config/point_control.cfg'])
    hashes = {str(p.relative_to(HERE.parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return {'composition.sha256.json': json.dumps(hashes, indent=2, sort_keys=True) + '\n', ** {'experiment.json': json.dumps(spec, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
            'experiment.launch': render(spec), 'README.md': report(spec)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('spec', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='new delivery directory; refuses overwrite')
    parser.add_argument('--check', action='store_true', help='read-only: compare existing bundle with deterministic output')
    args = parser.parse_args()
    try:
        spec = json.loads(args.spec.read_text(), object_pairs_hook=unique_pairs)
        validate(spec)
        files = bundle(spec)
        if args.check:
            require({p.name for p in args.output.iterdir()} == set(files),
                    'delivery directory: expected exactly the four generated files')
            for name, content in files.items():
                require((args.output / name).read_text() == content, name + ': changed or out of date; regenerate into a new directory')
            print('PASS: contract and generated bundle match; hardware not tested')
        else:
            args.output.mkdir(parents=True, exist_ok=False)
            for name, content in files.items():
                (args.output / name).write_text(content)
            print('Generated {} (launch will auto-start on ready; no ROS processes started)'.format(args.output))
        return 0
    except (ValueError, OSError, TypeError) as exc:
        print('ERROR: ' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
