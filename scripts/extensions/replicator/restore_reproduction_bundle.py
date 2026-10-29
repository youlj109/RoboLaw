#!/usr/bin/env python3
"""Restore pinned Replicator reproduction parts without replacing local files.

Python standard library only. No simulation, Git operation or authentication.
--verify-only reads existing manifests and target files, with no network/writes.
--offline restores only from verified cached manifests and archives.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import secrets
import stat
import sys
import tarfile
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'configs/extensions/replicator/reproduction_bundle_20261029.json'
CHUNK = 1024 * 1024


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(CHUNK), b''):
            h.update(block)
    return h.hexdigest()


def safe_relative(name):
    require(isinstance(name, str) and bool(name), 'Missing relative path')
    p = PurePosixPath(name)
    require(not p.is_absolute() and not PureWindowsPath(name).drive
            and '\\' not in name and '\x00' not in name
            and str(p) == name and name != '.' and '..' not in p.parts
            and '.git' not in p.parts, 'Unsafe relative path: ' + name)
    return p


def identifier(value):
    require(isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', value),
            'Invalid bundle/part identifier')
    return value


def hash_spec(row):
    require(isinstance(row, dict) and type(row.get('bytes')) is int and row['bytes'] >= 0
            and re.fullmatch('[0-9a-f]{64}', str(row.get('sha256', ''))), 'Invalid size/SHA binding')


def read_json(path):
    # Reject repeated keys rather than allowing a later manifest value to hide one.
    def unique(pairs):
        output = {}
        for key, value in pairs:
            require(key not in output, 'Duplicate JSON key: ' + key)
            output[key] = value
        return output
    return json.loads(Path(path).read_text(encoding='utf-8'), object_pairs_hook=unique)


def load_lock(path):
    data = read_json(path)
    require(data.get('schema_version') == 1 and data.get('kind') == 'robolaw_reproduction_bundle',
            'Unsupported reproduction bundle schema')
    identifier(data.get('bundle_id'))
    roots = data.get('allowed_roots')
    require(isinstance(roots, list) and roots and len(set(roots)) == len(roots), 'Invalid allowed_roots')
    for name in roots:
        safe_relative(name)
    parts = data.get('parts')
    require(isinstance(parts, list) and parts, 'Bundle has no parts')
    indexed = {}
    for part in parts:
        name = identifier(part.get('id'))
        require(name not in indexed, 'Duplicate part: ' + name)
        require(isinstance(part.get('group'), str) and part['group'], 'Missing part group')
        hash_spec(part); hash_spec(part.get('manifest'))
        require(part.get('format', 'tar.gz') == 'tar.gz', 'Only tar.gz regular-file parts are supported')
        for value in (part, part['manifest']):
            require(isinstance(value.get('url'), str) and value['url'].startswith(('https://', 'http://')),
                    'Part/manifest requires an HTTP(S) URL')
        indexed[name] = part
    profiles = data.get('profiles')
    require(isinstance(profiles, dict) and profiles, 'Missing profiles')
    for name, members in profiles.items():
        identifier(name)
        require(isinstance(members, list) and members and len(set(members)) == len(members)
                and set(members) <= set(indexed), 'Invalid profile: ' + name)
    require(data.get('default_profile') in profiles, 'Unknown default profile')
    require(set(profiles.get('all', [])) == set(indexed), 'The all profile must include every part')
    return data, indexed


def select_parts(lock, indexed, profile=None, parts=None):
    require(not (profile is not None and parts is not None), '--profile and --parts are mutually exclusive')
    if parts is not None:
        names = parts
        require(names and len(set(names)) == len(names) and set(names) <= set(indexed), 'Unknown/duplicate selected part')
    else:
        profile = profile or lock['default_profile']
        require(profile in lock['profiles'], 'Unknown profile: ' + profile)
        names = lock['profiles'][profile]
    return [indexed[name] for name in names]


def cache_file_valid(path, spec):
    if path.is_symlink():
        raise ValueError('Symlink cache entry is not allowed: ' + str(path))
    return path.is_file() and path.stat().st_size == spec['bytes'] and digest(path) == spec['sha256']


def download(spec, target, offline=False, resume=True):
    """Resume only a correctly ranged response, then validate the complete SHA."""
    if cache_file_valid(target, spec):
        return
    require(not offline, 'Verified cache file unavailable: ' + str(target))
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + '.partial')
    require(not partial.is_symlink() and (not partial.exists() or partial.is_file()), 'Unsafe download partial')
    size = partial.stat().st_size if partial.exists() else 0
    if size == spec['bytes'] and digest(partial) == spec['sha256']:
        os.replace(partial, target)
        return
    if not resume or size >= spec['bytes']:
        size = 0
    headers = {'User-Agent': 'RoboLaw-reproduction-restore/1', 'Accept-Encoding': 'identity'}
    if size:
        headers['Range'] = f'bytes={size}-'
    print('Downloading ' + target.name, flush=True)
    with urlopen(Request(spec['url'], headers=headers), timeout=120) as response:
        status_code = response.status
        append = size > 0 and status_code == 206
        if status_code == 206:
            match = re.fullmatch(r'bytes (\d+)-(\d+)/(\d+)', response.headers.get('Content-Range', ''))
            require(match and int(match[1]) == size and int(match[2]) == spec['bytes'] - 1
                    and int(match[3]) == spec['bytes'], 'Incorrect resumed Content-Range')
        else:
            require(status_code == 200, 'Unexpected download status: ' + str(status_code))
        # Servers ignoring Range return 200; restart instead of appending twice.
        received = size if append else 0
        with partial.open('ab' if append else 'wb') as output:
            while block := response.read(CHUNK):
                received += len(block)
                require(received <= spec['bytes'], 'Download exceeds pinned size')
                output.write(block)
    if received != spec['bytes']:
        raise ValueError('Incomplete download; verified resume can continue: ' + target.name)
    if digest(partial) != spec['sha256']:
        partial.unlink()
        raise ValueError('Download SHA-256 mismatch: ' + target.name)
    os.replace(partial, target)


def load_part_manifest(path, part, lock):
    require(cache_file_valid(path, part['manifest']), 'Part manifest size/SHA mismatch: ' + part['id'])
    manifest = read_json(path)
    require(manifest.get('kind') == 'robolaw_reproduction_part'
            and manifest.get('bundle_id') == lock['bundle_id']
            and manifest.get('part_id') == part['id'], 'Part manifest identity mismatch')
    rows = manifest.get('files')
    require(isinstance(rows, list) and rows, 'Empty/invalid part manifest')
    indexed = {}
    for row in rows:
        hash_spec(row)
        name = row.get('path'); safe_relative(name)
        require(any(name == prefix or name.startswith(prefix + '/') for prefix in lock['allowed_roots']),
                'File outside allowed roots: ' + name)
        require(name not in indexed, 'Duplicate file in manifest: ' + name)
        require(row.get('type', 'file') == 'file', 'Only regular files can be restored')
        mode = row.get('mode', 0o644)
        require(type(mode) is int and 0 <= mode <= 0o777, 'Unsafe file mode: ' + name)
        indexed[name] = row
    return indexed


@contextmanager
def parent_fd(root, name, create=False):
    """Walk directory descriptors, never following a parent symlink."""
    path = safe_relative(name)
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    fd = os.open(root, flags)
    try:
        for component in path.parts[:-1]:
            if create:
                try:
                    os.mkdir(component, mode=0o755, dir_fd=fd)
                except FileExistsError:
                    pass
            nxt = os.open(component, flags, dir_fd=fd)
            os.close(fd); fd = nxt
        yield fd, path.name
    finally:
        os.close(fd)


def target_status(root, row):
    """Return absent/valid; conflicts (including symlinks) are never overwritten."""
    try:
        with parent_fd(root, row['path']) as (fd, name):
            info = os.stat(name, dir_fd=fd, follow_symlinks=False)
            require(stat.S_ISREG(info.st_mode), 'Existing target is not a regular file: ' + row['path'])
            require(info.st_size == row['bytes'], 'Existing file differs; preserve/move it first: ' + row['path'])
            f = os.open(name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=fd)
            with os.fdopen(f, 'rb') as stream:
                h = hashlib.sha256()
                for block in iter(lambda: stream.read(CHUNK), b''):
                    h.update(block)
            require(h.hexdigest() == row['sha256'], 'Existing file differs; preserve/move it first: ' + row['path'])
            return 'valid'
    except FileNotFoundError:
        return 'absent'


def inspect_archive(path, part, indexed):
    require(cache_file_valid(path, part), 'Archive size/SHA mismatch: ' + part['id'])
    seen = set()
    with tarfile.open(path, 'r:gz') as archive:
        for member in archive:
            safe_relative(member.name)
            require(member.name in indexed and member.name not in seen, 'Unexpected/duplicate archive member: ' + member.name)
            row = indexed[member.name]
            require(member.isfile() and member.size == row['bytes'] and not member.sparse,
                    'Archive must contain ordinary files of pinned sizes: ' + member.name)
            seen.add(member.name)
    require(seen == set(indexed), 'Archive members do not match manifest: ' + part['id'])


def install_member(root, row, source):
    # A random sibling is published with link(2), which fails if the destination
    # appears concurrently. Directory descriptors prevent symlink-parent races.
    with parent_fd(root, row['path'], create=True) as (fd, name):
        temporary = '.robolaw-restore-' + secrets.token_hex(12)
        handle = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=fd)
        try:
            count = 0; h = hashlib.sha256()
            with os.fdopen(handle, 'wb') as output:
                while block := source.read(CHUNK):
                    count += len(block)
                    require(count <= row['bytes'], 'Extracted file exceeds pinned size')
                    h.update(block); output.write(block)
                output.flush(); os.fsync(output.fileno())
                os.fchmod(output.fileno(), row.get('mode', 0o644))
            require(count == row['bytes'] and h.hexdigest() == row['sha256'], 'Extracted file SHA/size mismatch: ' + row['path'])
            os.link(temporary, name, src_dir_fd=fd, dst_dir_fd=fd, follow_symlinks=False)
        finally:
            os.unlink(temporary, dir_fd=fd)


def restore(config=CONFIG, root=ROOT, cache=None, profile=None, parts=None, verify_only=False, offline=False, resume=True):
    lock, indexed = load_lock(config)
    selected = select_parts(lock, indexed, profile, parts)
    root = Path(root).absolute()
    require(root.is_dir() and not root.is_symlink(), 'Destination root must be an existing real directory')
    root = root.resolve()
    cache = Path(cache).absolute() if cache else root / 'assets/.downloads/replicator-reproduction' / lock['bundle_id']
    # Cache directory is user-selected, but existing links within its absolute path
    # are rejected before any download or replacement of a cached artifact.
    require(not any(p.is_symlink() for p in (cache, *cache.parents)), 'Symlink cache directory is not allowed')
    loaded = []; all_rows = {}; missing = set()
    for part in selected:
        path = cache / (part['id'] + '.manifest.json')
        if verify_only:
            require(cache_file_valid(path, part['manifest']), 'Verified manifest unavailable; restore first: ' + str(path))
        else:
            download(part['manifest'], path, offline, resume)
        rows = load_part_manifest(path, part, lock)
        duplicates = set(rows) & set(all_rows)
        require(not duplicates, 'Duplicate paths across selected parts: ' + str(sorted(duplicates)[:3]))
        all_rows.update(rows)
        loaded.append((part, rows, cache / (part['id'] + '.tar.gz')))
    # Reject both file/child collisions and existing user edits before installation.
    for name, row in all_rows.items():
        require(not any(str(p) in all_rows for p in PurePosixPath(name).parents if str(p) != '.'),
                'Manifest file is also a parent directory: ' + name)
        if target_status(root, row) == 'absent':
            missing.add(name)
    if verify_only:
        require(not missing, f'{len(missing)} files missing; first: ' + ', '.join(sorted(missing)[:3]))
        return dict(bundle_id=lock['bundle_id'],parts=[p['id'] for p in selected],files=len(all_rows),restored=0,verified=True,network_used=False)
    # Validate every needed archive header before installing any member.
    for part, rows, archive_path in loaded:
        if missing.intersection(rows):
            download(part, archive_path, offline, resume)
            inspect_archive(archive_path, part, rows)
    installed = 0
    for part, rows, archive_path in loaded:
        if not missing.intersection(rows):
            continue
        with tarfile.open(archive_path, 'r:gz') as archive:
            for member in archive:
                if member.name not in missing:
                    continue
                row = rows[member.name]
                if target_status(root, row) == 'valid':
                    continue
                with archive.extractfile(member) as source:
                    install_member(root, row, source)
                installed += 1
    require(all(target_status(root, row) == 'valid' for row in all_rows.values()), 'Post-restore verification failed')
    return dict(bundle_id=lock['bundle_id'],parts=[p['id'] for p in selected],files=len(all_rows),restored=installed,verified=True,offline=offline)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--root', type=Path, default=ROOT, help='Existing destination checkout root')
    parser.add_argument('--cache', type=Path, help='Verified part downloads (default: assets/.downloads/replicator-reproduction/<bundle_id>)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--profile', help='Profile from pinned lock; default is default_profile (rerun)')
    group.add_argument('--parts', nargs='+', help='Explicit part IDs from the lock')
    parser.add_argument('--verify-only', action='store_true', help='No network or writes; verify cached manifests and installed files')
    parser.add_argument('--offline', action='store_true', help='Restore only from already verified local cache')
    parser.add_argument('--no-resume', action='store_true', help='Restart incomplete downloads instead of sending HTTP Range')
    args = parser.parse_args(argv)
    try:
        result = restore(args.config, args.root, args.cache, args.profile, args.parts, args.verify_only, args.offline, not args.no_resume)
    except (ValueError, OSError, tarfile.TarError) as error:
        print('Restore refused: ' + str(error), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
