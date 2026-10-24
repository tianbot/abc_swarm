#!/usr/bin/env python3
"""Vendored PID provenance and legacy import API, without global/site packages."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]


class VendoredPIDTests(unittest.TestCase):
    def test_upstream_code_and_license_match_recorded_copy(self):
        folder=ROOT/'src/simple_pid'
        metadata=json.loads((folder/'UPSTREAM.json').read_text())
        self.assertEqual(metadata['version'],'2.0.0')
        self.assertEqual(metadata['license'],'MIT')
        for name,digest in metadata['sha256'].items():
            self.assertEqual(hashlib.sha256((folder/name).read_bytes()).hexdigest(),digest,name)
        self.assertIn('Martin Lundberg',(folder/'LICENSE.md').read_text())

    def test_legacy_import_and_used_api_work_with_site_packages_disabled(self):
        code='''
import json
import simple_pid
from simple_pid import PID
pid=PID(1,0,0,setpoint=0,sample_time=None,output_limits=(-0.15,0.15))
assert pid(-1,dt=0.1)==0.15
assert pid(1,dt=0.1)==-0.15
pid.tunings=(2,1,0)
assert pid.tunings==(2,1,0)
pid.output_limits=(-0.1,0.1)
for _ in range(20):assert pid(-1,dt=0.1)<=0.1
assert abs(pid.components[1])<=0.1
pid.reset()
assert pid.components==(0,0,0)
print(json.dumps({'file':simple_pid.__file__}))
'''
        env=dict(os.environ,PYTHONPATH=str(ROOT/'src'))
        output=subprocess.check_output([sys.executable,'-S','-c',code],env=env,text=True)
        self.assertEqual(Path(json.loads(output)['file']).resolve(),ROOT/'src/simple_pid/__init__.py')


if __name__=='__main__':unittest.main()
