#!/usr/bin/env python3
"""Customer experiment contract: success, rejection, deterministic delivery, no ROS."""
import copy
import importlib.util
import json
import hashlib
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODULE = importlib.util.spec_from_file_location('swarm_delivery', ROOT / 'ai/generate_launch.py')
harness = importlib.util.module_from_spec(MODULE)
MODULE.loader.exec_module(harness)
EXAMPLE = json.loads((ROOT / 'ai/experiment.example.json').read_text())


def scenario(experiment):
    spec = copy.deepcopy(EXAMPLE)
    cap = harness.CATALOG['experiments'][experiment]
    spec['experiment'] = experiment
    spec['robots'] = spec['robots'][:cap['robots']]
    if cap.get('coordinated'):
        spec = json.loads((ROOT / ('ai/coordinated-'+cap['coordinated']+'.example.json')).read_text())
    elif cap.get('point'):
        spec = json.loads((ROOT / ('ai/orca.example.json' if cap.get('orca') else 'ai/point-stabilization.example.json')).read_text())
    elif not cap['path']:
        spec['path'] = None
        spec['startup']['start_teleop'] = True
    return spec


class AIHarnessTests(unittest.TestCase):
    def test_v3_explicit_input_and_planning_are_validated(self):
        spec=scenario('manual_triangle')
        old=copy.deepcopy(spec);old['contract_version']=2
        with self.assertRaisesRegex(ValueError,'expected 3'):harness.validate(old)
        spec['teleop']={'linear':.09,'angular':.7};spec['planning']=False
        spec['mocap_control']={'velocity_source':'mocap','position_tolerance':.04,'calibration':None,
                              'feedback':{'tf_timeout':.6,'odom_timeout':1.5,'odom_future_tolerance':.1},
                              'velocity_window':{'window':.25,'min_interval':.1}}
        root=ET.fromstring(harness.render(spec))
        self.assertEqual(root.find('include/arg[@name="teleop_linear"]').get('value'),'0.09')
        self.assertEqual(root.find('include/arg[@name="teleop_angular"]').get('value'),'0.7')
        self.assertEqual(root.find('include/arg[@name="planning"]').get('value'),'false')
        self.assertEqual(root.find('param[@name="/abc_swarm_demo/odom_timeout"]').get('value'),'1.5')
        self.assertIsNotNone(root.find('rosparam[@param="/abc_swarm_demo/mocap_velocity"]'))
        self.assertIn('0.7rad/s',harness.report(spec))
        for change in [lambda s:s['teleop'].update(angular=float('nan')),
                       lambda s:s.update(planning='false'),lambda s:s['teleop'].update(linear=.3)]:
            bad=copy.deepcopy(spec);change(bad)
            with self.assertRaises(ValueError):harness.validate(bad)
        spec=scenario('coordinated_circle');spec['planning']=True
        with self.assertRaisesRegex(ValueError,'ORCA'):harness.validate(spec)

    def test_admission_planning_is_declared_only_when_avoidance_is_enabled(self):
        for experiment,enabled in [('manual_triangle',True),('point_stabilization_orca',True),
                                   ('point_stabilization',False),('coordinated_circle',False)]:
            spec=scenario(experiment)
            arg=ET.fromstring(harness.render(spec)).find('include/arg[@name="planning"]')
            self.assertEqual(arg is not None and arg.get('value')=='true',enabled)
            self.assertEqual('入位规划已启用' in harness.report(spec),enabled)
        spec=scenario('manual_triangle');spec['admission']={'enabled':False}
        self.assertEqual(ET.fromstring(harness.render(spec)).find('include/arg[@name="planning"]').get('value'),'false')

    def test_follower_admission_is_explicit_validated_and_can_opt_out(self):
        settings=dict(handoff_distance=.15,clearance=.02,position_margin=.005,handoff_time=.3,settle_time=.5,progress_timeout=45.,stop_linear=.02,stop_angular=.05,waypoint_tolerance=.025)
        orca=json.loads((ROOT/'ai/orca.example.json').read_text())['orca']
        for experiment in ['manual_triangle','manual_snake','path_triangle','path_snake']:
            spec=scenario(experiment);spec['admission']={'enabled':True,'settings':settings,'orca':orca}
            self.assertFalse(harness.CATALOG['experiments'][experiment]['stationary_pose_hold'])
            self.assertIn('同一前置点leader-follower控制律',harness.report(spec))
            root=ET.fromstring(harness.render(spec))
            self.assertEqual(root.find('include/arg[@name="startup"]').get('value'),'true')
            self.assertIsNotNone(root.find('rosparam[@param="/abc_swarm_demo/admission"]'))
            for change in [lambda s:s['admission'].update(enabled='true'),
                           lambda s:s['admission']['settings'].update(handoff_distance=0),
                           lambda s:s['admission']['orca'].update(robot_radius=float('nan'))]:
                bad=copy.deepcopy(spec);change(bad)
                with self.assertRaises(ValueError):harness.validate(bad)
            spec['admission']={'enabled':False}
            self.assertEqual(ET.fromstring(harness.render(spec)).find('include/arg[@name="startup"]').get('value'),'false')
            spec['mocap_control']={'velocity_source':'mocap','position_tolerance':.04,'calibration':None}
            harness.validate(spec)
            self.assertIn('velocity_source',harness.render(spec))

    def test_mocap_control_extension_is_explicit_and_rejects_bad_calibration(self):
        for experiment in ['point_stabilization','point_stabilization_orca','coordinated_circle','coordinated_triangle']:
            spec=scenario(experiment)
            spec['mocap_control']={'velocity_source':'mocap','position_tolerance':.08,
                                  'calibration':{r['name']:{'yaw_offset':.2,'marker_offset':[.03,.01]} for r in spec['robots']}}
            root=ET.fromstring(harness.render(spec))
            self.assertEqual(root.find('include/arg[@name="velocity_source"]').get('value'),'mocap')
            self.assertIn('0.08',root.find('rosparam[@param="/abc_swarm_demo/position_tolerance"]').text)
            self.assertIn('marker_offset',harness.report(spec))
            if experiment=='point_stabilization_orca':
                hashes=json.loads(harness.bundle(spec)['composition.sha256.json'])
                self.assertIn('launch/mocap/demo_mocap_three_mini_point_stabilization.launch',hashes)
            for change in [lambda s:s['mocap_control'].update(velocity_source='cmd_vel'),
                           lambda s:s['mocap_control'].update(position_tolerance=.5),
                           lambda s:s['mocap_control']['calibration'].pop(s['robots'][0]['name']),
                           lambda s:s['mocap_control']['calibration'][s['robots'][0]['name']].update(yaw_offset=True),
                           lambda s:s['mocap_control']['calibration'][s['robots'][0]['name']].update(marker_offset=[0,float('nan')])]:
                bad=copy.deepcopy(spec);change(bad)
                with self.assertRaises(ValueError):harness.validate(bad)
        unsupported=scenario('path_single');unsupported['mocap_control']=spec['mocap_control']
        with self.assertRaises(ValueError):harness.validate(unsupported)

    def test_all_supported_experiments_use_existing_entrypoints_without_arming(self):
        for experiment, cap in harness.CATALOG['experiments'].items():
            with self.subTest(experiment=experiment):
                root = ET.fromstring(harness.render(scenario(experiment)))
                self.assertEqual(len(root.findall('include')), 1)
                self.assertFalse(root.findall('node'))
                include = root.find('include')
                self.assertTrue(include.get('file').endswith('/' + cap['launch']))
                args = {a.get('name'): a.get('value') for a in include.findall('arg')}
                self.assertEqual(args['vrpn_server_ip'], EXAMPLE['vrpn']['server'])
                self.assertEqual(args['rigid_body_1'], 'tianbot_mini_01')
                self.assertNotIn('initial_pose', args)
                self.assertNotIn('enabled', args)
                if experiment in ('manual_triangle','manual_snake'):
                    self.assertEqual(args['mode'],'manual')

    def test_rounded_rectangle_and_separate_backend_are_supported(self):
        spec = scenario('path_snake')
        spec['path'].update(shape='rounded_rectangle', clockwise=True)
        spec['startup'].update(start_driver=False, start_vrpn=False)
        root = ET.fromstring(harness.render(spec))
        args = {a.get('name'): a.get('value') for a in root.find('include').findall('arg')}
        self.assertEqual(args['shape'], 'rounded_rectangle')
        self.assertEqual(args['clockwise'], 'true')
        self.assertEqual(root.find("arg[@name='start_driver']").get('default'), 'false')

    def test_unknown_start_position_is_preserved_and_never_turned_into_tf(self):
        spec = scenario('path_single')
        spec['robots'][0]['initial_pose'] = None
        self.assertIn('现场核对', harness.report(spec))
        self.assertNotIn('static_transform', harness.render(spec))

    def test_rejects_invalid_customer_inputs(self):
        changes = [
            lambda s: s.update(contract_version=True),
            lambda s: s.update(contract_version=1),
            lambda s: s.update(unknown='ignored?'),
            lambda s: s.update(experiment='five_mini_snake'),
            lambda s: s.update(localization='gazebo'),
            lambda s: s.update(fixed_frame='map'),
            lambda s: s['robots'].append(copy.deepcopy(s['robots'][0])),
            lambda s: s['robots'][1].update(model='rmtt'),
            lambda s: s['robots'][1].update(name=s['robots'][0]['name']),
            lambda s: s['robots'][1].update(rigid_body=s['robots'][0]['rigid_body']),
            lambda s: s['robots'][0].update(rigid_body='world'),
            lambda s: s['robots'][0].update(name='$(eval 1)'),
            lambda s: s['robots'][0].update(initial_pose=[0, float('nan'), 0]),
            lambda s: s['vrpn'].update(server='$(optenv VRPN)'),
            lambda s: s['vrpn'].update(port=True),
            lambda s: s['startup'].update(start_driver='false'),
            lambda s: s['startup'].update(start_teleop=True),
            lambda s: s.update(spacing=0),
            lambda s: s['path'].update(speed=0.16),
            lambda s: s['path'].update(laps=0),
            lambda s: s['path'].update(laps=True),
            lambda s: s['path'].update(radius=float('inf')),
            lambda s: s['path'].update(shape='rounded_rectangle', width=1.0),
            lambda s: s['path'].update(shape='line'),
            lambda s: s['path'].update(clockwise='false'),
            lambda s: s['path'].update(enabled=True),
        ]
        for change in changes:
            spec = copy.deepcopy(EXAMPLE)
            change(spec)
            with self.subTest(spec=spec), self.assertRaises(ValueError):
                harness.bundle(spec)
        spec = scenario('manual_snake')
        spec['path'] = EXAMPLE['path']
        with self.assertRaises(ValueError):
            harness.bundle(spec)

    def test_manual_placement_rotates_offsets_and_chain_uses_each_predecessor(self):
        spec = scenario('path_snake')
        spec['robots'][0]['initial_pose'] = [1, 2, 0]
        self.assertIn('[0.0, 2.0, 0]', harness.report(spec))
        spec = scenario('path_triangle')
        spec['robots'][0]['initial_pose'] = [1, 2, 1.5707963267948966]
        self.assertIn('[0.5, 1.5, 1.5707963267948966]', harness.report(spec))

    def test_cli_refuses_overwrite_and_detects_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'delivery'
            cmd = [sys.executable, str(ROOT / 'ai/generate_launch.py'),
                   str(ROOT / 'ai/experiment.example.json'), '--output', str(out)]
            def run(args):
                return subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertEqual(run(cmd).returncode, 0)
            before = {p.name: p.read_bytes() for p in out.iterdir()}
            self.assertEqual(run(cmd).returncode, 2)
            self.assertEqual(run(cmd + ['--check']).returncode, 0)
            self.assertEqual(before, {p.name: p.read_bytes() for p in out.iterdir()})
            for name in ('experiment.launch', 'README.md', 'composition.sha256.json'):
                path = out / name
                path.write_text('tampered')
                self.assertEqual(run(cmd + ['--check']).returncode, 2)
                path.write_bytes(before[name])
            (out / 'experiment.launch').unlink()
            self.assertEqual(run(cmd + ['--check']).returncode, 2)

    def test_point_targets_are_explicit_and_checked(self):
        spec = scenario('point_stabilization')
        text = harness.render(spec)
        self.assertIn('/abc_swarm_demo/target_poses', text)
        self.assertIn('控制目标', harness.report(spec))
        self.assertNotIn('人工摆位记录，不设置 TF、不移动到起点', harness.report(spec))
        for change in [lambda s: s.pop('targets'),
                       lambda s: s['targets'].update(reference='map'),
                       lambda s: s['targets'].update(poses=[[0,0,0]]),
                       lambda s: s['targets']['poses'][0].__setitem__(0,'$(eval 1)'),
                       lambda s: s['startup'].update(start_teleop=True)]:
            invalid = copy.deepcopy(spec)
            change(invalid)
            with self.assertRaises(ValueError):harness.bundle(invalid)

    def test_orca_delivery_has_explicit_geometry_and_rejects_unsafe_contracts(self):
        spec=scenario('point_stabilization_orca')
        self.assertIn('/abc_swarm_demo/orca',harness.render(spec))
        self.assertIn('ORCA',harness.report(spec))
        for change in [lambda s:s.pop('orca'),lambda s:s['orca'].update(robot_radius=-1),
                       lambda s:s['orca'].update(safety_margin=True),lambda s:s['orca'].update(unknown=1),
                       lambda s:s['targets'].update(poses=[[0,0,0],[0,0,0],[1,1,0]])]:
            invalid=copy.deepcopy(spec);change(invalid)
            with self.assertRaises(ValueError):harness.bundle(invalid)

    def test_coordinated_contract_geometry_and_startup_are_explicit(self):
        for experiment in ('coordinated_circle','coordinated_triangle'):
            spec=scenario(experiment)
            self.assertIn('协调路径',harness.report(spec))
            self.assertIn('origin_x',harness.render(spec))
            size='diameter' if experiment.endswith('circle') else 'side_length'
            for change in [lambda s:s['trajectory'].update(speed=.16),lambda s:s['trajectory'].update(laps=0),
                           lambda s:s['trajectory'].update(reference='bad'),lambda s:s['trajectory'].update(clockwise='false'),
                           lambda s:s['trajectory'].__setitem__(size,1),lambda s:s['startup'].update(start_teleop=True),
                           lambda s:s.pop('orca')]:
                invalid=copy.deepcopy(spec);change(invalid)
                with self.assertRaises(ValueError):harness.bundle(invalid)
            spec['orca']=scenario('point_stabilization_orca')['orca']
            self.assertIn('/abc_swarm_demo/orca',harness.render(spec))

    def test_installed_bundle_hashes_installed_control_files(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            share = prefix/'share/abc_swarm'
            for name in ('ai', 'config', 'launch', 'licenses'):
                shutil.copytree(ROOT/name, share/name)
            core = prefix/'lib/python3/dist-packages/abc_swarm'
            nodes = prefix/'lib/abc_swarm'
            shutil.copytree(ROOT/'src/abc_swarm', core)
            shutil.copytree(ROOT/'nodes', nodes)
            # Detect installed content, rather than silently hashing another checkout.
            installed = nodes/'point_stabilization.py'
            installed.write_text(installed.read_text()+'\n# installed fixture\n')
            with patch.object(harness, 'HERE', share/'ai'):
                hashes = json.loads(harness.bundle(scenario('manual_triangle'))['composition.sha256.json'])
            self.assertEqual(hashes['nodes/point_stabilization.py'], hashlib.sha256(installed.read_bytes()).hexdigest())
            self.assertIn('src/abc_swarm/formation_control.py', hashes)

    def test_explicit_position_hysteresis_is_validated_rendered_and_reported(self):
        spec=scenario('coordinated_triangle')
        spec['mocap_control']={'velocity_source':'mocap','position_tolerance':.10,
                               'position_hysteresis':.07,'calibration':None}
        root=ET.fromstring(harness.render(spec))
        self.assertEqual(root.find('rosparam[@param="/abc_swarm_demo/position_hysteresis"]').text,'0.07')
        self.assertIn('切换余量：0.07 m',harness.report(spec))
        for value in [True,-.01,.10,float('nan')]:
            spec['mocap_control']['position_hysteresis']=value
            with self.assertRaises(ValueError):harness.validate(spec)

    def test_triangle_requires_explicit_centroid_instead_of_old_vertex_origin(self):
        spec=scenario('coordinated_triangle')
        self.assertEqual(spec['trajectory']['reference'],'world')
        self.assertEqual(spec['trajectory']['center'],[0,0,0])
        self.assertIn('重心',harness.report(spec))
        spec['trajectory']['origin']=spec['trajectory'].pop('center')
        with self.assertRaisesRegex(ValueError,'replace origin'):harness.validate(spec)

    def test_duplicate_json_keys_rejected_without_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            path.write_text('{"name": "a", "name": "b"}')
            out = Path(directory) / 'delivery'
            result = subprocess.run([sys.executable, str(ROOT / 'ai/generate_launch.py'),
                                     str(path), '--output', str(out)], stderr=subprocess.PIPE)
            self.assertEqual(result.returncode, 2)
            self.assertIn(b'duplicate JSON key', result.stderr)
            self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
