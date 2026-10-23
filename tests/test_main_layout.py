#!/usr/bin/env python3
"""Verify the origin/main contract, not the discarded refactor layout."""
import ast
import hashlib
import json
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BASELINE = json.loads((ROOT / 'tests/main_baseline.json').read_text())


class MainLayoutTests(unittest.TestCase):
    def test_original_files_are_preserved_except_two_explicit_mocap_moves(self):
        self.assertEqual(len(BASELINE['renames']), 2)
        for original, spec in BASELINE['originals'].items():
            with self.subTest(original=original):
                path = ROOT / spec['current_path']
                self.assertTrue(path.is_file(), path)
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), spec['sha256'])
                if original in BASELINE['renames']:
                    self.assertFalse((ROOT / original).exists())

    def test_nodes_and_includes_have_no_additional_files(self):
        for directory in ('nodes', 'launch/includes'):
            expected = {p for p in BASELINE['originals'] if p.startswith(directory + '/')}
            actual = {str(p.relative_to(ROOT)) for p in (ROOT / directory).rglob('*')
                      if p.is_file() and '__pycache__' not in p.parts
                      and not any(part.startswith('.') for part in p.relative_to(ROOT).parts)}
            self.assertEqual(actual, expected)

    def test_new_layout_is_flat_and_does_not_carry_refactor_directories(self):
        self.assertFalse(any(p.is_dir() for p in (ROOT / 'config').iterdir()))
        self.assertEqual({p.name for p in (ROOT / 'scripts').glob('*.py')},
                         {'demo_runtime.py', 'preset_path_tracker.py',
                          'leader_follower_tracker.py', 'pid_tracker.py'})
        for p in (ROOT / 'scripts').glob('*.py'):
            self.assertTrue(os.access(str(p), os.X_OK), p)
            ast.parse(p.read_text(), filename=str(p))
        for directory in ('demo', 'legacy', 'examples', 'launch/simulation',
                          'launch/sim_demo', 'launch/experiments', 'src/target_tracker'):
            self.assertFalse((ROOT / directory).exists(), directory)
        self.assertEqual(len(list((ROOT / 'launch/mocap').rglob('*.launch'))), 9)
        for p in (ROOT / 'launch/mocap').glob('*.launch'):
            self.assertTrue(p.name.startswith('demo_mocap_') or p.name == 'mocap_backend.launch')
        self.assertFalse(any(p.is_dir() for p in (ROOT / 'launch/mocap').iterdir()))

    def test_new_launches_parse_and_unique_names_resolve(self):
        from roslaunch.rlutil import resolve_launch_arguments
        for directory in ('launch/mocap', 'launch/common'):
            for path in (ROOT / directory).iterdir():
                if path.suffix not in ('.launch', '.xml'):
                    continue
                ET.parse(str(path))
                if path.suffix == '.launch':
                    result = resolve_launch_arguments(['abc_swarm', path.name])[0]
                    self.assertEqual(Path(result).resolve(), path)


if __name__ == '__main__':
    unittest.main()
