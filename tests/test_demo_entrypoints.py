#!/usr/bin/env python3
"""One-file startup, site overrides and reusable backend composition."""
from pathlib import Path
import tempfile
import unittest
import os
from unittest.mock import patch
import yaml
import roslaunch

ROOT = Path(__file__).resolve().parents[1]
DEMOS = sorted(p for p in (ROOT / 'launch/mocap').glob('*.launch')
               if not p.name.startswith('demo_mocap_tf_tt_')
               and 'point_stabilization' not in p.name
               and p.name not in ('demo_mocap_three_mini_circle_tracking.launch','demo_mocap_three_mini_triangle_tracking.launch'))  # Covered by test_point_entrypoints.py.


def load(path, argv=None):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path), config, argv=argv or [], verbose=False)
    return config


class DemoEntrypointTests(unittest.TestCase):
    def test_follower_mode_selects_one_head_controller_and_keeps_roles(self):
        for formation in ('triangle','snake'):
            path=ROOT/('launch/mocap/demo_mocap_three_mini_leader_follower_'+formation+'.launch')
            common=['start_driver:=false','start_vrpn:=false','use_rviz:=false',
                    'robot_name_1:=customer_01','robot_name_2:=customer_02','robot_name_3:=customer_03',
                    'rigid_body_1:=body_01','rigid_body_2:=body_02','rigid_body_3:=body_03']
            manual=load(path,common+['mode:=manual'])
            self.assertEqual(sum(n.type=='teleop_twist_keyboard.py' for n in manual.nodes),1)
            self.assertFalse(any(n.type=='preset_path_tracker.py' for n in manual.nodes))
            self.assertEqual(manual.params['/abc_swarm_demo/mode'].value,'manual')
            auto=load(path,common+['mode:=auto','start_teleop:=true','radius:=.75','laps:=2',
                                  'clockwise:=true','max_linear:=.12','max_angular:=.3','tf_timeout:=.8'])
            self.assertEqual(sum(n.type=='preset_path_tracker.py' for n in auto.nodes),1)
            self.assertFalse(any(n.type=='teleop_twist_keyboard.py' for n in auto.nodes))
            self.assertEqual(sum(n.type=='demo_runtime.py' for n in auto.nodes),1)
            self.assertEqual(sum(n.type=='leader_follower_tracker.py' for n in auto.nodes),2)
            for key,value in [('shape','circle'),('radius',.75),('laps',2),('clockwise',True),
                              ('speed',.12),('max_angular',.3),('tf_timeout',.8),('robot_frame','body_01'),
                              ('cmd_topic','/customer_01/cmd_vel')]:
                self.assertEqual(auto.params['/abc_swarm_path/'+key].value,value)
            self.assertEqual(auto.params['/abc_swarm_demo/mode'].value,'path')
            expected='body_02' if formation=='snake' else 'body_01'
            self.assertEqual(auto.params['/customer_03_leader_follower/leader_frame'].value,expected)
            self.assertEqual(auto.params['/customer_03_leader_follower/max_linear'].value,.12)
            self.assertEqual(auto.params['/abc_swarm_path/edges'].value[-1]['leader'],expected)
            for bad in ['mannual','AUTO','invalid']:
                with self.subTest(formation=formation,mode=bad),self.assertRaises(roslaunch.xmlloader.XmlParseException):
                    load(path,common+['mode:='+bad])
            # roslaunch treats an empty CLI assignment as an omitted argument.
            self.assertEqual(load(path,common+['mode:=']).params['/abc_swarm_demo/mode'].value,'manual')

    def setUp(self):
        env = patch.dict(os.environ, {'ABC_SWARM_VRPN_SERVER': ''})
        env.start()
        self.addCleanup(env.stop)

    def test_ai_generated_customer_launch_resolves_roles_and_initial_readiness_gate(self):
        import copy
        import json
        import runpy
        harness = runpy.run_path(str(ROOT / 'ai/generate_launch.py'))
        example = json.loads((ROOT / 'ai/experiment.example.json').read_text())
        for experiment, cap in harness['CATALOG']['experiments'].items():
            if cap.get('point') or cap.get('coordinated'):
                continue  # Its non-following role mapping is checked in test_point_entrypoints.py.
            with self.subTest(experiment=experiment), tempfile.TemporaryDirectory() as folder:
                spec = copy.deepcopy(example)
                spec['experiment'] = experiment
                spec['robots'] = spec['robots'][:cap['robots']]
                for i, robot in enumerate(spec['robots']):
                    robot['name'] = 'customer_car_' + str(i)
                    robot['rigid_body'] = 'customer_body_' + str(i)
                if not cap['path']:
                    spec['path'] = None
                    spec['startup']['start_teleop'] = True
                path = Path(folder) / 'experiment.launch'
                path.write_text(harness['render'](spec))
                config = load(path)
                self.assertFalse(config.params['/abc_swarm_demo/enabled'].value)
                self.assertEqual(config.params['/abc_swarm_demo/robot_frames'].value,
                                 [r['rigid_body'] for r in spec['robots']])
                self.assertEqual(config.params['/vrpn_client_node/server'].value, spec['vrpn']['server'])
                self.assertEqual(sum(n.type == 'leader_follower_tracker.py' for n in config.nodes),
                                 cap['robots'] - 1)
                if cap['path']:
                    self.assertEqual(config.params['/abc_swarm_path/cmd_topic'].value,
                                     '/customer_car_0/cmd_vel')
                if cap['robots'] == 3:
                    # Check actual follower TF mapping, not just generated XML syntax.
                    expected = 'customer_body_1' if cap['formation'] == 'snake' else 'customer_body_0'
                    self.assertEqual(config.params['/customer_car_2_leader_follower/leader_frame'].value,
                                     expected)
                config = load(path, ['start_driver:=false', 'start_vrpn:=false',
                                     'use_rviz:=false', 'start_teleop:=false'])
                self.assertFalse(any(n.package in ('rosserial_server', 'vrpn_client_ros',
                                                  'rviz', 'teleop_twist_keyboard') for n in config.nodes))

    def test_public_demo_starts_one_backend_and_one_runtime(self):
        profile = yaml.safe_load((ROOT / 'config/mocap.yaml').read_text())
        for path in DEMOS:
            if path.name == 'mocap_backend.launch':
                continue
            with self.subTest(demo=path.name):
                config = load(path)
                self.assertEqual(sum(n.package == 'vrpn_client_ros' for n in config.nodes), 1)
                self.assertEqual(sum(n.package == 'rosserial_server' for n in config.nodes), 1)
                self.assertEqual(sum(n.type == 'demo_runtime.py' for n in config.nodes), 1)
                self.assertFalse(config.params['/abc_swarm_demo/enabled'].value)
                self.assertEqual(config.params['/vrpn_client_node/server'].value, profile['server'])
                self.assertEqual(config.params['/vrpn_client_node/port'].value, profile['port'])
                manual = 'path_tracking' not in path.name
                self.assertEqual(sum(n.package == 'teleop_twist_keyboard' for n in config.nodes),
                                 1 if manual else 0)
                if manual:
                    teleop = next(n for n in config.nodes if n.package == 'teleop_twist_keyboard')
                    self.assertIn(['cmd_vel', '/abc_swarm_demo/teleop_cmd'], teleop.remap_args)
                    self.assertEqual(teleop.launch_prefix, 'xterm -e')

    def test_site_file_and_cli_override_have_single_precedence(self):
        with tempfile.TemporaryDirectory() as folder:
            site = Path(folder) / 'site.yaml'
            site.write_text('server: 192.0.2.10\nport: 3990\n')
            for path in DEMOS:
                with self.subTest(demo=path.name):
                    config = load(path, ['site_config:=' + str(site)])
                    self.assertEqual(config.params['/vrpn_client_node/server'].value, '192.0.2.10')
                    self.assertEqual(config.params['/vrpn_client_node/port'].value, 3990)
                    config = load(path, ['site_config:=' + str(site),
                                         'vrpn_server_ip:=192.0.2.11', 'vrpn_port:=3991'])
                    self.assertEqual(config.params['/vrpn_client_node/server'].value, '192.0.2.11')
                    self.assertEqual(config.params['/vrpn_client_node/port'].value, 3991)

    def test_gui_can_reuse_backend_without_ui_or_duplicate_services(self):
        for path in DEMOS:
            if path.name == 'mocap_backend.launch':
                continue
            args = ['start_driver:=false', 'start_vrpn:=false', 'use_rviz:=false']
            if 'path_tracking' not in path.name:
                args.append('start_teleop:=false')
            config = load(path, args)
            self.assertFalse(any(n.package in ('vrpn_client_ros', 'rosserial_server',
                                               'teleop_twist_keyboard', 'rviz')
                                 for n in config.nodes))
            self.assertEqual(sum(n.type == 'demo_runtime.py' for n in config.nodes), 1)

    def test_default_linear_speed_is_consistent_through_the_command_chain(self):
        for path in DEMOS:
            if path.name=='mocap_backend.launch':continue
            config=load(path)
            if 'path_tracking' in path.name:
                self.assertEqual(config.params['/abc_swarm_path/speed'].value,0.15)
            else:
                self.assertEqual(config.params['/leader_teleop/speed'].value,0.15)
                self.assertEqual(config.params['/abc_swarm_demo/max_linear'].value,0.15)
            followers=[p.value for name,p in config.params.items()
                       if name.endswith(('_leader_follower/max_linear','_tracker/max_linear'))]
            self.assertTrue(all(value==0.15 for value in followers))
        config=load(ROOT/'launch/mocap/demo_mocap_three_mini_leader_follower_triangle.launch',
                    ['max_linear:=0.12'])
        self.assertEqual(config.params['/leader_teleop/speed'].value,0.12)
        self.assertEqual(config.params['/abc_swarm_demo/max_linear'].value,0.12)
        self.assertEqual(config.params['/tianbot_mini_02_leader_follower/max_linear'].value,0.12)

    def test_controller_fragment_has_no_backend_or_lifecycle(self):
        config = load(ROOT / 'launch/common/three_mini_leader_follower.launch.xml',
                      ['formation:=snake'])
        self.assertEqual(sum(n.type == 'leader_follower_tracker.py' for n in config.nodes), 2)
        self.assertFalse(any(n.type in ('demo_runtime.py', 'socket_node', 'vrpn_client_node')
                             for n in config.nodes))
        self.assertNotIn('/abc_swarm_demo/enabled', config.params)
        head=load(ROOT/'launch/common/leader_path_tracker.launch.xml',['formation:=triangle'])
        self.assertEqual(len(head.nodes),1)
        self.assertEqual(head.nodes[0].type,'preset_path_tracker.py')
        self.assertNotIn('/abc_swarm_demo/enabled',head.params)

    def test_path_and_followers_share_configured_front_distance(self):
        for path in DEMOS:
            if 'path_tracking' not in path.name:
                continue
            config = load(path, ['front_distance:=0.23', 'start_driver:=false',
                                 'start_vrpn:=false', 'use_rviz:=false'])
            self.assertEqual(config.params['/abc_swarm_path/front_distance'].value, 0.23)
            followers = [p.value for name, p in config.params.items()
                         if name.endswith('_leader_follower/front_distance')]
            self.assertEqual(followers, [] if path.name == 'demo_mocap_path_tracking.launch'
                             else [0.23, 0.23])

    def test_algorithm_config_is_not_shadowed_by_demo_defaults(self):
        defaults = yaml.safe_load((ROOT / 'config/path_tracking.yaml').read_text())
        for path in DEMOS:
            if 'path_tracking' not in path.name:
                continue
            config = load(path, ['radius:=0.95', 'speed:=0.04', 'laps:=2',
                                 'start_driver:=false', 'start_vrpn:=false', 'use_rviz:=false'])
            for key, value in defaults.items():
                self.assertEqual(config.params['/abc_swarm_path/' + key].value, value)
            for key, value in [('radius', 0.95), ('speed', 0.04), ('laps', 2)]:
                self.assertEqual(config.params['/abc_swarm_path/' + key].value, value)
