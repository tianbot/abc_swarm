#!/usr/bin/env python3
import copy
import json
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import patch
import yaml
import roslaunch

ROOT=Path(__file__).resolve().parents[1]
POINT=ROOT/'launch/mocap/demo_mocap_three_mini_point_stabilization.launch'
ORCA=ROOT/'launch/mocap/demo_mocap_three_mini_orca_point_stabilization.launch'


def load(path,args=None):
    config=roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path),config,argv=args or [],verbose=False)
    return config


class OrcaEntrypointTests(unittest.TestCase):
    def test_regular_point_stays_unmodified_and_orca_is_explicit(self):
        base=load(POINT)
        self.assertEqual(base.params['/abc_swarm_demo/avoidance'].value,'none')
        self.assertFalse(any(key.startswith('/abc_swarm_demo/orca/') for key in base.params))
        config=load(ORCA)
        self.assertEqual(config.params['/abc_swarm_demo/avoidance'].value,'orca')
        self.assertEqual(sum(n.type=='point_stabilization.py' for n in config.nodes),1)
        self.assertEqual(sum(n.package=='rosserial_server' for n in config.nodes),1)
        self.assertEqual(sum(n.package=='vrpn_client_ros' for n in config.nodes),1)
        defaults=yaml.safe_load((ROOT/'config/orca.yaml').read_text())
        for key,value in defaults.items():self.assertEqual(config.params['/abc_swarm_demo/orca/'+key].value,value)
        self.assertNotIn('/abc_swarm_demo/enabled',config.params)
        config=load(ORCA,['start_driver:=false','start_vrpn:=false','use_rviz:=false','formation:=snake'])
        self.assertEqual(len(config.nodes),1)
        self.assertEqual(config.params['/abc_swarm_demo/formation'].value,'snake')

    def test_custom_orca_file_and_goal_file_are_independent(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'orca.yaml'
            settings=yaml.safe_load((ROOT/'config/orca.yaml').read_text());settings['robot_radius']=.16
            p.write_text(yaml.safe_dump(settings))
            config=load(ORCA,['orca_config:='+str(p),'vrpn_server_ip:=192.0.2.50'])
            self.assertEqual(config.params['/abc_swarm_demo/orca/robot_radius'].value,.16)
            self.assertEqual(config.params['/vrpn_client_node/server'].value,'192.0.2.50')
            self.assertEqual(config.params['/abc_swarm_demo/target_reference'].value,'initial_leader')

    def test_ai_geometry_overrides_defaults_and_keeps_mocap_backend(self):
        harness=runpy.run_path(str(ROOT/'ai/generate_launch.py'))
        spec=json.loads((ROOT/'ai/orca.example.json').read_text());spec['orca']['robot_radius']=.14
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'demo.launch';p.write_text(harness['render'](spec))
            config=load(p,['start_driver:=false','start_vrpn:=false','use_rviz:=false'])
            self.assertEqual(config.params['/abc_swarm_demo/avoidance'].value,'orca')
            self.assertEqual(config.params['/abc_swarm_demo/orca/robot_radius'].value,.14)
            self.assertEqual(config.params['/abc_swarm_demo/target_poses'].value,spec['targets']['poses'])
            self.assertEqual(len(config.nodes),1)

    def test_invalid_orca_settings_fail_before_publishers(self):
        module=runpy.run_path(str(ROOT/'nodes/point_stabilization.py'))
        defaults=yaml.safe_load((ROOT/'config/orca.yaml').read_text())
        base={'~robots':[{'name':'a','frame':'a_body'},{'name':'b','frame':'b_body'},{'name':'c','frame':'c_body'}],
              '~target_poses':[[0,0,0],[-.5,.5,0],[-.5,-.5,0]],'~avoidance':'orca','~orca':defaults}
        for change in [lambda p:p.update({'~avoidance':'unknown'}),
                       lambda p:p['~orca'].pop('robot_radius'),
                       lambda p:p['~orca'].update(robot_radius=-1),
                       lambda p:p['~orca'].update(front_distance=True),
                       lambda p:p['~orca'].update(stall_timeout=0)]:
            params=copy.deepcopy(base);change(params)
            with patch('rospy.get_param',side_effect=lambda key,default=None:params.get(key,default)),patch('rospy.Publisher') as pub:
                with self.assertRaises(ValueError):module['PointStabilization']()
                pub.assert_not_called()


if __name__=='__main__':unittest.main()
