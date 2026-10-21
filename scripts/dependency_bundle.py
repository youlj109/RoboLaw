#!/usr/bin/env python3
"""Verified ModelScope dependency bundles (Python standard library only)."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'configs/dependency_mirrors.json'


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def download(url, target, expected_hash, expected_bytes=None):
    if target.is_file() and digest(target) == expected_hash:
        if expected_bytes is None or target.stat().st_size == expected_bytes:
            return
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + '.partial')
    for attempt in range(3):
        try:
            print('Downloading: ' + url, flush=True)
            with urlopen(Request(url, headers={'User-Agent': 'RoboLaw-dependencies/2'}), timeout=120) as response:
                with partial.open('wb') as output:
                    shutil.copyfileobj(response, output, length=1024 * 1024)
            if expected_bytes is not None and partial.stat().st_size != expected_bytes:
                raise RuntimeError('Download size mismatch: ' + target.name)
            if digest(partial) != expected_hash:
                raise RuntimeError('Download SHA-256 mismatch: ' + target.name)
            partial.replace(target)
            return
        except Exception:
            partial.unlink(missing_ok=True)
            if attempt == 2:
                raise
            time.sleep(attempt + 1)


def target_path(root, name, allowed_roots):
    rel = PurePosixPath(name)
    if rel.is_absolute() or '..' in rel.parts or str(rel) != name or '.git' in rel.parts:
        raise ValueError('Unsafe bundle path: ' + name)
    if not any(name.startswith(prefix + '/') for prefix in allowed_roots):
        raise ValueError('Unexpected bundle path: ' + name)
    target = root / name
    # Do not write through existing directory symlinks, even inside the root.
    parent = target.parent
    while parent != root:
        if parent.is_symlink():
            raise ValueError('Symlink parent: ' + str(parent))
        parent = parent.parent
    return target


def valid_file(root, row, spec):
    path = target_path(root, row['path'], spec['allowed_roots'])
    if row['type'] == 'symlink':
        return path.is_symlink() and os.readlink(path) == row['target']
    return (not path.is_symlink() and path.is_file()
            and path.stat().st_size == row['bytes'] and digest(path) == row['sha256'])


def restore_bundle(key, root=ROOT, verify_only=False):
    root = root.resolve()
    spec = json.loads(CONFIG.read_text())['bundles'][key]
    cache = root / 'assets/.downloads/dependencies'
    manifest_path = cache / (key + '.manifest.json')
    if verify_only:
        if not manifest_path.is_file() or digest(manifest_path) != spec['manifest_sha256']:
            raise RuntimeError('Verified manifest unavailable for ' + key + '; restore first')
    else:
        download(spec['manifest_url'], manifest_path, spec['manifest_sha256'])
    manifest = json.loads(manifest_path.read_text())
    rows = manifest['files']
    by_name = {row['path']: row for row in rows}
    if len(rows) != spec['entry_count'] or len(by_name) != len(rows):
        raise RuntimeError('Invalid manifest entry count: ' + key)
    for row in rows:
        path = target_path(root, row['path'], spec['allowed_roots'])
        if row['type'] == 'symlink':
            destination = (path.parent / row['target']).resolve()
            allowed = any(os.path.commonpath([destination, root / p]) == str(root / p)
                          for p in spec['allowed_roots'])
            if Path(row['target']).is_absolute() or not allowed:
                raise RuntimeError('Unsafe symlink: ' + row['path'])
        elif row['type'] != 'file':
            raise RuntimeError('Unexpected manifest entry type')
    missing = [row for row in rows if not valid_file(root, row, spec)]
    if not missing:
        print(f'{key}: {len(rows)} entries verified', flush=True)
        return
    if verify_only:
        raise RuntimeError(f'{key}: {len(missing)} missing or changed entries')
    # Preserve user edits. A failed partial restore can be resumed safely.
    for row in missing:
        path = target_path(root, row['path'], spec['allowed_roots'])
        if path.exists() or path.is_symlink():
            raise RuntimeError('Existing file differs from pinned bundle; move it aside first: ' + str(path))
    archive = cache / spec['archive_url'].rsplit('/', 1)[1]
    download(spec['archive_url'], archive, spec['archive_sha256'], spec['archive_bytes'])
    with tarfile.open(archive, 'r:gz') as bundle:
        members = bundle.getmembers()
        if len(members) != len(rows) or {m.name for m in members} != set(by_name):
            raise RuntimeError('Bundle contents differ from verified manifest: ' + key)
        # Validate all members before writing any of them.
        for member in members:
            row = by_name[member.name]
            if row['type'] == 'file':
                if not member.isfile() or member.size != row['bytes']:
                    raise RuntimeError('Unexpected archive file: ' + member.name)
            elif not member.issym() or member.linkname != row['target']:
                raise RuntimeError('Unexpected archive symlink: ' + member.name)
        for member in members:
            row = by_name[member.name]
            if valid_file(root, row, spec):
                continue
            path = target_path(root, member.name, spec['allowed_roots'])
            path.parent.mkdir(parents=True, exist_ok=True)
            if row['type'] == 'symlink':
                path.symlink_to(row['target'])
            else:
                partial = path.with_name(path.name + '.partial')
                with bundle.extractfile(member) as source, partial.open('wb') as output:
                    shutil.copyfileobj(source, output, length=1024 * 1024)
                if digest(partial) != row['sha256']:
                    partial.unlink(missing_ok=True)
                    raise RuntimeError('Extracted checksum mismatch: ' + member.name)
                partial.chmod(row['mode'] & 0o777)
                partial.replace(path)
    if any(not valid_file(root, row, spec) for row in rows):
        raise RuntimeError('Post-restore verification failed: ' + key)
    print(f'{key}: restored and verified {len(rows)} entries', flush=True)
