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


def validate(spec):
    fields(spec, 'contract_version name localization fixed_frame vrpn robots experiment spacing path startup', 'experiment')
    require(type(spec['contract_version']) is int and spec['contract_version'] == 1, 'contract_version: expected 1')
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
    fields(spec['vrpn'], 'server port', 'vrpn')
    try:
        ipaddress.IPv4Address(spec['vrpn']['server'])
    except (ValueError, TypeError, ipaddress.AddressValueError):
        raise ValueError('vrpn.server: explicit IPv4 address required, no substitutions')
    require(isinstance(spec['vrpn']['server'], str), 'vrpn.server: string required')
    require(type(spec['vrpn']['port']) is int and 1 <= spec['vrpn']['port'] <= 65535, 'vrpn.port: 1..65535')
    number(spec['spacing'], 'spacing', positive=True)
    fields(spec['startup'], 'start_driver start_vrpn use_rviz start_teleop', 'startup')
    for key, value in spec['startup'].items():
        require(type(value) is bool, 'startup.' + key + ': boolean required')
    if capability['path']:
        require(not spec['startup']['start_teleop'], 'automatic path cannot start teleop')
        path = spec['path']
        fields(path, 'shape radius width height speed laps clockwise', 'path')
        require(path['shape'] in ('circle', 'rounded_rectangle'), 'path.shape: circle or rounded_rectangle')
        for key in ('radius', 'width', 'height', 'speed'):
            number(path[key], 'path.' + key, positive=True)
        require(path['speed'] <= 0.1, 'path.speed: maximum supported demo request is 0.1 m/s')
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
    root.append(ET.Comment(' Generated by abc_swarm ai/generate_launch.py. Motion starts disabled. '))
    for key, value in spec['startup'].items():
        ET.SubElement(root, 'arg', name=key, default=text_value(value))
    include = ET.SubElement(root, 'include', file='$(find abc_swarm)/launch/mocap/' + capability['launch'])
    args = {'vrpn_server_ip': spec['vrpn']['server'], 'vrpn_port': spec['vrpn']['port'], 'spacing': spec['spacing']}
    for key in spec['startup']:
        if key != 'start_teleop' or not capability['path']:
            args[key] = '$(arg ' + key + ')'
    if not capability['path']:
        args['enabled'] = False
    for i, robot in enumerate(spec['robots'], 1):
        args['robot_name_' + str(i)] = robot['name']
        args['rigid_body_' + str(i)] = robot['rigid_body']
    if capability['path']:
        args.update(spec['path'])
    for key, value in args.items():
        ET.SubElement(include, 'arg', name=key, value=text_value(value))
    template = HERE.parent / 'launch/mocap' / capability['launch']
    declared = {arg.get('name') for arg in ET.parse(str(template)).getroot().findall('arg')}
    require(set(args) <= declared, 'launch interface changed: undeclared arguments ' + str(sorted(set(args) - declared)))
    # Python 3.8 (Noetic) has no ET.indent.
    return minidom.parseString(ET.tostring(root, encoding='utf-8')).toprettyxml(indent='  ')


def report(spec):
    capability = validate(spec)
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
              '确认网络、刚体、odom、摆位和场地路径预览。start_driver/start_vrpn 为 false 时表示外部后台已运行，不是取消定位需求。',
              '本机 ROS master 与场地网络配置见 docs/field-test.md。一次只运行一套实验，已存在后台时关闭对应启动项。', '',
              '```bash', 'roslaunch ./experiment.launch', '```', '',
              '另一个已 source 的终端查看状态，等待 ready；自动实验先核对 RViz 中完整编队路径。', '',
              '```bash', 'rostopic echo /abc_swarm_demo/status', '```', '',
              '现场操作人员确认后启用（AI 不代执行）：', '',
              '```bash', 'rosservice call /abc_swarm_demo/set_enabled "data: true"', '```', '',
              '停止并确认车已静止，再 Ctrl+C：', '',
              '```bash', 'rosservice call /abc_swarm_demo/set_enabled "data: false"', '```', '',
              '自动实验停止后可调用 /abc_swarm_demo/reset；等待重新 ready 并核对预览，再启用。',
              '手动实验在键盘终端 i 前进、u/o 缓弯、k 停止；输入超时会停止。',
              '', '## 验收记录', '', '- 离线生成检查：通过。', '- ROS 解析：待现场执行。',
              '- 动捕、驱动、反馈和停止功能：未测。', '- 实际实验效果：未测。',
              '- 填写日期、仓库提交、现场差异及结果，不能把生成通过记为实车通过。', '']
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
            print('Generated {} (motion disabled; no ROS processes started)'.format(args.output))
        return 0
    except (ValueError, OSError, TypeError) as exc:
        print('ERROR: ' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
