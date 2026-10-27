#!/usr/bin/env python3
import copy
import json
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import patch

import roslaunch
import yaml

ROOT=Path(__file__).resolve().parents[1]


def load(path,args=None):
    cfg=roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path),cfg,argv=args or [],verbose=False)
    return cfg


def node_params(cfg):
    params={}
    for key,p in cfg.params.items():
        if not key.startswith('/abc_swarm_demo/'):continue
        parts=key.split('/abc_swarm_demo/')[1].split('/')
        tree=params
        for part in parts[:-1]:tree=tree.setdefault(part,{})
        tree[parts[-1]]=p.value
    return {'~'+k:v for k,v in params.items()}


class FormationEntrypointTests(unittest.TestCase):
    def test_yaml_values_survive_defaults_and_cli_only_changes_its_owner(self):
        path=ROOT/'launch/mocap/demo_mocap_three_mini_leader_follower_triangle.launch'
        with tempfile.TemporaryDirectory() as folder:
            files={}
            for key,name,changes in [('control','point_control.yaml',dict(max_linear=.07,max_angular=.2,k_heading=1.1)),
                                      ('follow','leader_follower.yaml',dict(max_linear=.11,max_angular=1.2,k_x=.8)),
                                      ('feedback','feedback.yaml',dict(tf_timeout=.9,odom_timeout=1.5))]:
                data=yaml.safe_load((ROOT/'config'/name).read_text());data.update(changes)
                p=Path(folder)/name;p.write_text(yaml.safe_dump(data));files[key]=p
            args=[k+'_config:='+str(p) for k,p in files.items()]
            p=node_params(load(path,args))
            self.assertEqual(p['~max_linear'],.07);self.assertEqual(p['~max_angular'],.2)
            self.assertEqual(p['~follow']['max_linear'],.11);self.assertEqual(p['~follow']['max_angular'],1.2)
            self.assertEqual(p['~tf_timeout'],.9);self.assertEqual(p['~odom_timeout'],1.5)
            p=node_params(load(path,args+['max_linear:=.13','point_max_angular:=.3']))
            self.assertEqual(p['~follow']['max_linear'],.13);self.assertEqual(p['~max_linear'],.07)
            self.assertEqual(p['~max_angular'],.3);self.assertEqual(p['~follow']['max_angular'],1.2)

    def test_default_pipeline_has_one_owner_and_reuses_backend_in_both_modes(self):
        settings=yaml.safe_load((ROOT/'config/admission.yaml').read_text())
        for shape in ['triangle','snake']:
            path=ROOT/('launch/mocap/demo_mocap_three_mini_leader_follower_'+shape+'.launch')
            for mode in ['manual','auto']:
                cfg=load(path,['mode:='+mode,'start_driver:=false','start_vrpn:=false','use_rviz:=false'])
                self.assertEqual(sum(n.type=='point_stabilization.py' for n in cfg.nodes),1)
                self.assertFalse(any(n.type in ('demo_runtime.py','leader_follower_tracker.py','preset_path_tracker.py') for n in cfg.nodes))
                self.assertEqual(sum(n.type=='teleop_twist_keyboard.py' for n in cfg.nodes),mode=='manual')
                p=node_params(cfg)
                self.assertEqual(p['~task'],'leader_follower');self.assertEqual(p['~velocity_source'],'mocap')
                self.assertEqual(p['~max_angular'],.5)
                self.assertEqual(p['~follow']['max_angular'],.6)
                self.assertEqual(p['~admission'],settings)
                self.assertEqual(p['~follow']['mode'],mode)
                expected='tianbot_mini_02' if shape=='snake' else 'tianbot_mini_01'
                self.assertEqual(p['~follow']['edges'][-1]['leader'],expected)
                raw=load(path,['startup:=false','mode:='+mode,'start_driver:=false','start_vrpn:=false','use_rviz:=false'])
                self.assertEqual(sum(n.type=='point_stabilization.py' for n in raw.nodes),1)
                self.assertFalse(node_params(raw)['~startup'])

    def test_field_profile_and_custom_vehicle_mapping_are_explicit(self):
        path=ROOT/'launch/mocap/demo_mocap_three_mini_leader_follower_triangle.launch'
        cfg=load(path,['control_config:='+str(ROOT/'config/mocap_field_control.yaml'),'max_linear:=.12'])
        params=node_params(cfg)
        self.assertEqual(params['~position_tolerance'],.08)
        self.assertGreater(params['~mocap_calibration']['tianbot_mini_02']['yaw_offset'],1.9)
        self.assertEqual(params['~max_linear'],.15)
        self.assertEqual(params['~follow']['max_linear'],.12)
        self.assertEqual(params['~max_angular'],.5)
        cfg=load(path,['robot_name_2:=client_02','rigid_body_2:=body_02'])
        params=node_params(cfg)
        self.assertEqual(params['~robots'][1],{'name':'client_02','frame':'body_02'})
        self.assertEqual(params['~follow']['edges'][0]['follower'],'client_02')

    def test_invalid_mission_configuration_fails_before_publishers(self):
        module=runpy.run_path(str(ROOT/'nodes/point_stabilization.py'))
        cfg=load(ROOT/'launch/mocap/demo_mocap_three_mini_leader_follower_triangle.launch')
        base=node_params(cfg)
        for change in [lambda p:p['~follow'].update(mode='mannual'),
                       lambda p:p['~follow'].update(edges=[]),
                       lambda p:p['~follow']['edges'][0].update(leader='missing'),
                       lambda p:p['~admission'].update(settle_time=-1),
                       lambda p:p.update({'~velocity_source':'cmd_vel'}),
                       lambda p:p.update({'~avoidance':'invalid'})]:
            p=copy.deepcopy(base);change(p)
            with patch('rospy.get_param',side_effect=lambda key,default=None:p.get(key,default)),patch('rospy.Publisher') as pub:
                with self.assertRaises(ValueError):module['PointStabilization']()
                pub.assert_not_called()

    def test_ai_following_experiments_select_pipeline_and_expand_calibration(self):
        harness=runpy.run_path(str(ROOT/'ai/generate_launch.py'))
        for experiment in ['manual_triangle','manual_snake','path_triangle','path_snake']:
            spec=json.loads((ROOT/'ai/experiment.example.json').read_text());spec['experiment']=experiment
            if experiment.startswith('manual'):spec['path']=None;spec['startup']['start_teleop']=True
            spec['mocap_control']={'velocity_source':'mocap','position_tolerance':.04,
                                  'calibration':{r['name']:{'yaw_offset':.2,'marker_offset':[.03,0]} for r in spec['robots']}}
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'delivery.launch';path.write_text(harness['render'](spec))
                cfg=load(path,['start_driver:=false','start_vrpn:=false','use_rviz:=false','start_teleop:=false'])
                self.assertEqual(len(cfg.nodes),1)
                params=node_params(cfg)
                self.assertEqual(params['~task'],'leader_follower')
                self.assertTrue(params['~planning'])
                self.assertEqual(params['~follow']['mode'],'manual' if experiment.startswith('manual') else 'auto')
                self.assertEqual(params['~mocap_calibration']['tianbot_mini_02']['yaw_offset'],.2)


if __name__=='__main__':unittest.main()
