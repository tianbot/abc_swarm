#!/usr/bin/env python3
from pathlib import Path
import copy
import json
import runpy
import tempfile
import unittest
from unittest.mock import patch
import roslaunch

ROOT=Path(__file__).resolve().parents[1]


def load(path,args=None):
    config=roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path),config,argv=args or [],verbose=False)
    return config


class CoordinatedEntrypointTests(unittest.TestCase):
    def test_ai_mocap_feedback_and_calibration_expand_without_extra_nodes(self):
        harness=runpy.run_path(str(ROOT/'ai/generate_launch.py'))
        for experiment in ['point_stabilization','point_stabilization_orca','coordinated_circle','coordinated_triangle']:
            example=('coordinated-'+experiment.split('_')[1]+'.example.json' if experiment.startswith('coordinated')
                     else 'orca.example.json' if experiment.endswith('_orca') else 'point-stabilization.example.json')
            spec=json.loads((ROOT/'ai'/example).read_text())
            spec['mocap_control']={'velocity_source':'mocap','position_tolerance':.08,
                                  'calibration':{r['name']:{'yaw_offset':.3,'marker_offset':[.03,.01]} for r in spec['robots']}}
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'demo.launch';path.write_text(harness['render'](spec))
                config=load(path,['start_driver:=false','start_vrpn:=false','use_rviz:=false'])
                self.assertEqual(len(config.nodes),1)
                self.assertEqual(config.params['/abc_swarm_demo/velocity_source'].value,'mocap')
                self.assertEqual(config.params['/abc_swarm_demo/position_tolerance'].value,.08)
                self.assertEqual(config.params['/abc_swarm_demo/mocap_calibration/tianbot_mini_02/yaw_offset'].value,.3)

    def test_both_demos_have_one_owner_default_size_speed_and_optional_backend(self):
        for shape in ('circle','triangle'):
            p=ROOT/('launch/mocap/demo_mocap_three_mini_'+shape+'_tracking.launch')
            config=load(p)
            self.assertEqual(sum(n.type=='point_stabilization.py' for n in config.nodes),1)
            self.assertEqual(sum(n.package=='rosserial_server' for n in config.nodes),1)
            self.assertEqual(sum(n.package=='vrpn_client_ros' for n in config.nodes),1)
            self.assertEqual(config.params['/abc_swarm_demo/task'].value,shape)
            self.assertEqual(config.params['/abc_swarm_demo/coordinated/diameter'].value,2)
            self.assertEqual(config.params['/abc_swarm_demo/coordinated/side_length'].value,2)
            self.assertEqual(config.params['/abc_swarm_demo/coordinated/speed'].value,.15)
            self.assertNotIn('/abc_swarm_demo/enabled',config.params)
            view=next(n for n in config.nodes if n.package=='rviz')
            self.assertIn('coordinated_tracking.rviz',view.args)
            self.assertEqual(len(view.remap_args),3)
            size='diameter' if shape=='circle' else 'side_length'
            config=load(p,[size+':=1.5','start_driver:=false','start_vrpn:=false','use_rviz:=false','speed:=0.12',
                           'reference:=world','origin_x:=1.2','origin_y:=2.1','origin_yaw:=0.4',
                           'robot_name_2:=customer_02','rigid_body_2:=body_02','avoidance:=orca'])
            self.assertEqual(len(config.nodes),1)
            self.assertEqual(config.params['/abc_swarm_demo/coordinated/'+size].value,1.5)
            self.assertEqual(config.params['/abc_swarm_demo/max_linear'].value,.12)
            self.assertEqual(config.params['/abc_swarm_demo/coordinated/origin'].value,[1.2,2.1,.4])
            self.assertEqual(config.params['/abc_swarm_demo/robots'].value[1],{'name':'customer_02','frame':'body_02'})
            self.assertEqual(config.params['/abc_swarm_demo/orca/robot_radius'].value,.13)

    def test_ai_tasks_resolve_geometry_and_optional_assembly_orca(self):
        harness=runpy.run_path(str(ROOT/'ai/generate_launch.py'))
        orca=json.loads((ROOT/'ai/orca.example.json').read_text())['orca']
        for shape in ('circle','triangle'):
            spec=json.loads((ROOT/('ai/coordinated-'+shape+'.example.json')).read_text())
            spec['orca']=orca
            spec['trajectory'].update(reference='world',origin=[1,2,.4])
            with tempfile.TemporaryDirectory() as folder:
                p=Path(folder)/'demo.launch';p.write_text(harness['render'](spec))
                config=load(p,['start_driver:=false','start_vrpn:=false','use_rviz:=false'])
                self.assertEqual(len(config.nodes),1)
                self.assertEqual(config.params['/abc_swarm_demo/task'].value,shape)
                self.assertEqual(config.params['/abc_swarm_demo/coordinated/origin'].value,[1,2,.4])
                self.assertEqual(config.params['/abc_swarm_demo/avoidance'].value,'orca')

    def test_bad_loop_config_is_rejected_before_publishers(self):
        module=runpy.run_path(str(ROOT/'nodes/point_stabilization.py'))
        config=load(ROOT/'launch/mocap/demo_mocap_three_mini_circle_tracking.launch')
        base={'~'+key.split('/abc_swarm_demo/')[1]:p.value for key,p in config.params.items()
              if key.startswith('/abc_swarm_demo/') and not key.startswith('/abc_swarm_demo/coordinated/')}
        base['~coordinated']={key.split('/abc_swarm_demo/coordinated/')[1]:p.value for key,p in config.params.items()
                             if key.startswith('/abc_swarm_demo/coordinated/')}
        for change in [lambda p:p['~coordinated'].update(diameter=1),lambda p:p['~coordinated'].update(laps=True),
                       lambda p:p['~coordinated'].update(reference='bad'),lambda p:p['~coordinated'].update(speed=.2),
                       lambda p:p['~robots'].pop(),lambda p:p.update({'~mocap_calibration':{'unknown':{}}})]:
            params=copy.deepcopy(base);change(params)
            with patch('rospy.get_param',side_effect=lambda key,default=None:params.get(key,default)),patch('rospy.Publisher') as publisher:
                with self.assertRaises(ValueError):module['PointStabilization']()
                publisher.assert_not_called()


if __name__=='__main__':unittest.main()
