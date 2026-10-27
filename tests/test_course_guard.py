#!/usr/bin/env python3
"""Prove the guard rejects regressions, using disposable copies only."""
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('course_guard', ROOT / 'tests/check_course_compatibility.py')
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class CourseGuardTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='abc_course_guard_')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.manifest = json.loads((ROOT / 'tests/main_baseline.json').read_text())
        for entry in self.manifest['originals'].values():
            relative = entry['current_path']
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / relative, target)
        (self.root / 'tests').mkdir()
        self.save_manifest()

    def save_manifest(self):
        (self.root / 'tests/main_baseline.json').write_text(json.dumps(self.manifest))

    def violations(self):
        return guard.check(self.root, ROOT)

    def test_intact_course_and_new_algorithm_are_allowed(self):
        (self.root / 'nodes/new_algorithm.py').write_text('# New algorithm\n')
        self.assertEqual(self.violations(), [])

    def test_changed_old_node_is_rejected_even_if_manifest_is_rewritten(self):
        import hashlib
        relative = 'nodes/leader_follower.py'
        path = self.root / relative
        path.write_text(path.read_text() + '\n# Accidental course edit\n')
        self.manifest['originals'][relative]['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.save_manifest()
        errors = self.violations()
        self.assertTrue(any('disagrees with frozen Git' in error for error in errors), errors)
        self.assertTrue(any('Original content changed' in error for error in errors), errors)

    def test_moved_course_launch_is_rejected(self):
        path = self.root / 'launch/demo/demo_sim_tf.launch'
        path.rename(path.with_name('renamed_demo.launch'))
        self.assertTrue(any('Missing original' in error for error in self.violations()))

    def test_duplicate_course_filename_is_rejected(self):
        folder = self.root / 'launch/duplicate'
        folder.mkdir()
        (folder / 'demo_sim_tf.launch').write_text('<launch/>')
        self.assertTrue(any('Ambiguous original roslaunch name' in error for error in self.violations()))

    def test_omitting_a_resource_from_manifest_is_rejected(self):
        del self.manifest['originals']['nodes/leader_follower.py']
        self.save_manifest()
        self.assertTrue(any('Manifest omitted' in error for error in self.violations()))

    def test_lost_executable_permission_is_rejected(self):
        (self.root / 'nodes/leader_follower.py').chmod(0o644)
        self.assertTrue(any('permission lost' in error for error in self.violations()))

    def test_baseline_and_rename_policy_cannot_silently_move(self):
        self.manifest['base_commit'] = '0' * 40
        self.manifest['renames'] = {}
        self.save_manifest()
        errors = self.violations()
        self.assertTrue(any('Frozen baseline changed' in error for error in errors))
        self.assertTrue(any('rename policy changed' in error for error in errors))
