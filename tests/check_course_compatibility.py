#!/usr/bin/env python3
"""Compare protected course resources with the immutable pre-refactor main.

Standard library + Git only; no ROS, hardware, simulator or network required.
The manifest is audited against Git, not accepted as its own source of truth.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE_COMMIT = '1708f332ae900384b9e84889b7ab3979aaae9f5f'
PROTECTED_PREFIXES = ('nodes/', 'launch/', 'config/', 'rviz/', 'docs/')
APPROVED_RENAMES = {
    'launch/demo/demo_mc_tf_tt_circle_mini.launch': 'launch/mocap/demo_mocap_tf_tt_circle_mini.launch',
    'launch/demo/demo_mc_tf_tt_follow_mini.launch': 'launch/mocap/demo_mocap_tf_tt_follow_mini.launch',
}


def baseline_files(repo):
    """Read authoritative blobs directly from the frozen Git commit."""
    try:
        listing = subprocess.check_output(
            ['git', '-C', str(repo), 'ls-tree', '-r', '-z', BASE_COMMIT], stderr=subprocess.PIPE)
        files = {}
        for entry in listing.split(b'\0'):
            if not entry:
                continue
            header, raw_path = entry.split(b'\t', 1)
            mode, kind, oid = header.decode().split()
            path = raw_path.decode('utf-8')
            if not path.startswith(PROTECTED_PREFIXES):
                continue
            content = subprocess.check_output(
                ['git', '-C', str(repo), 'cat-file', 'blob', oid], stderr=subprocess.PIPE)
            files[path] = (mode, hashlib.sha256(content).hexdigest())
        return files
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError('Cannot read frozen main ' + BASE_COMMIT +
                           '; fetch full Git history (CI checkout fetch-depth: 0).') from exc


def check(root=ROOT, repo=ROOT):
    """Return all contract violations; root can be an isolated test copy."""
    root = Path(root)
    errors = []
    originals = baseline_files(repo)
    try:
        manifest = json.loads((root / 'tests/main_baseline.json').read_text())
    except (OSError, ValueError) as exc:
        return ['Cannot read course manifest: ' + str(exc)]
    if manifest.get('base_commit') != BASE_COMMIT:
        errors.append('Frozen baseline changed; do not advance it to make a test pass')
    if manifest.get('renames') != APPROVED_RENAMES:
        errors.append('Approved rename policy changed; maintainer review is required')
    if manifest.get('allowed_original_content_fixes') != {}:
        errors.append('Original-content exceptions require a separate reviewed policy change')
    records = manifest.get('originals', {})
    if set(records) != set(originals):
        for name in sorted(set(originals) - set(records)):
            errors.append('Manifest omitted protected resource: ' + name)
        for name in sorted(set(records) - set(originals)):
            errors.append('Manifest invents baseline resource: ' + name)
    public_names = {}
    for folder, directories, files in os.walk(str(root)):
        directories[:] = [d for d in directories if not d.startswith('.') and d != '__pycache__']
        for name in files:
            if name.endswith('.launch'):
                public_names.setdefault(name, []).append(str(Path(folder) / name))
    for original, (mode, digest) in sorted(originals.items()):
        target = APPROVED_RENAMES.get(original, original)
        record = records.get(original, {})
        if record.get('sha256') != digest or record.get('current_path') != target:
            errors.append('Manifest disagrees with frozen Git resource: ' + original)
        path = root / target
        if not path.is_file():
            errors.append('Missing original resource: ' + target)
            continue
        if path.is_symlink():
            errors.append('Original resource replaced by symlink: ' + target)
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append('Original content changed: ' + target)
        if mode == '100755' and not os.access(str(path), os.X_OK):
            errors.append('Original executable permission lost: ' + target)
        if original in APPROVED_RENAMES and (root / original).exists():
            errors.append('Renamed original still exists: ' + original)
        if path.suffix == '.launch' and len(public_names.get(path.name, [])) != 1:
            errors.append('Ambiguous original roslaunch name: ' + path.name)
    return errors


def main():
    try:
        errors = check()
    except RuntimeError as exc:
        print('FAIL: ' + str(exc), file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print('FAIL: ' + error, file=sys.stderr)
        return 1
    print('PASS: frozen main ' + BASE_COMMIT[:7] +
          ', 48 original resources unchanged, 2 approved renames, original launch names unique')
    return 0


if __name__ == '__main__':
    sys.exit(main())
