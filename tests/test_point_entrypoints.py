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

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / 'launch/mocap/demo_mocap_three_mini_point_stabilization.launch'


def load(path, args=None):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path), config, argv=args or [], verbose=False)
    return config


class PointEntrypointTests(unittest.TestCase):
    def test_one_file_backend_target_config_and_no_followers_or_keyboard(self):
        config = load(DEMO)
        self.assertEqual(sum(n.package=='rosserial_server' for n in config.nodes),1)
        self.assertEqual(sum(n.package=='vrpn_client_ros' for n in config.nodes),1)
        self.assertEqual(sum(n.type=='point_stabilization.py' for n in config.nodes),1)
        self.assertFalse(any(n.type in ('demo_runtime.py','leader_follower_tracker.py','preset_path_tracker.py') for n in config.nodes))
        self.assertFalse(any(n.package=='teleop_twist_keyboard' for n in config.nodes))
        self.assertNotIn('/abc_swarm_demo/enabled',config.params)
        expected=yaml.safe_load((ROOT/'config/point_stabilization.yaml').read_text())
        for key,value in expected.items():self.assertEqual(config.params['/abc_swarm_demo/'+key].value,value)

    def test_custom_goal_file_device_mapping_and_existing_backend(self):
        with tempfile.TemporaryDirectory() as folder:
            goal=Path(folder)/'targets.yaml'
            data=yaml.safe_load((ROOT/'config/point_stabilization.yaml').read_text())
            data.update(target_reference='world',target_poses=[[1,2,0],[2,3,1],[3,4,2]],max_linear=0.03)
            goal.write_text(yaml.safe_dump(data))
            config=load(DEMO,['goal_config:='+str(goal),'start_driver:=false','start_vrpn:=false','use_rviz:=false',
                              'robot_name_2:=customer_02','rigid_body_2:=body_02'])
            self.assertEqual(len(config.nodes),1)
            self.assertEqual(config.params['/abc_swarm_demo/target_poses'].value,data['target_poses'])
            self.assertEqual(config.params['/abc_swarm_demo/max_linear'].value,0.03)
            self.assertEqual(config.params['/abc_swarm_demo/robots'].value[1],{'name':'customer_02','frame':'body_02'})

    def test_ai_generated_targets_override_packaged_default_without_initial_pose_confusion(self):
        harness=runpy.run_path(str(ROOT/'ai/generate_launch.py'))
        spec=json.loads((ROOT/'ai/point-stabilization.example.json').read_text())
        spec['targets']={'reference':'world','poses':[[1,2,0],[2,3,1],[3,4,2]]}
        spec['robots'][0]['initial_pose']=[100,200,3]
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'experiment.launch';p.write_text(harness['render'](spec))
            config=load(p,['start_driver:=false','start_vrpn:=false','use_rviz:=false'])
            self.assertEqual(config.params['/abc_swarm_demo/target_reference'].value,'world')
            self.assertEqual(config.params['/abc_swarm_demo/target_poses'].value,spec['targets']['poses'])
            self.assertEqual(len(config.nodes),1)

    def test_invalid_node_config_is_rejected_before_publishers_exist(self):
        module=runpy.run_path(str(ROOT/'nodes/point_stabilization.py'))
        base={'~robots':[{'name':'a','frame':'body_a'},{'name':'b','frame':'body_b'},{'name':'c','frame':'body_c'}],
              '~target_poses':[[0,0,0],[-0.5,0.5,0],[-0.5,-0.5,0]]}
        cases=[lambda p:p.update({'~max_linear':True}),
               lambda p:p.update({'~max_linear':-0.1}),
               lambda p:p.update({'~tf_timeout':float('nan')}),
               lambda p:p.update({'~target_reference':'map'}),
               lambda p:p.update({'~target_poses':[[0,0,0]]}),
               lambda p:p['~target_poses'][0].__setitem__(0,float('nan')),
               lambda p:p['~robots'][1].update(name='a'),
               lambda p:p['~robots'][1].update(frame='body_a')]
        for change in cases:
            params=copy.deepcopy(base);change(params)
            with patch('rospy.get_param',side_effect=lambda key,default=None:params.get(key,default)), \
                 patch('tf.TransformListener') as listener, patch('rospy.Publisher') as publisher:
                with self.assertRaises(ValueError):module['PointStabilization']()
                listener.assert_not_called();publisher.assert_not_called()

    def test_site_override_and_config_priority_are_shared(self):
        config=load(DEMO,['vrpn_server_ip:=192.0.2.31','vrpn_port:=3999'])
        self.assertEqual(config.params['/vrpn_client_node/server'].value,'192.0.2.31')
        self.assertEqual(config.params['/vrpn_client_node/port'].value,3999)


if __name__=='__main__':unittest.main()
