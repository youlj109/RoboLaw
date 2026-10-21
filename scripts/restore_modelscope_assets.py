#!/usr/bin/env python3
"""Restore the RoboLaw kitchen and default human assets from ModelScope."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
BASE = ('https://www.modelscope.cn/datasets/YouLinjing/'
        'Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/')
MANIFEST_SHA256 = '35b7d1a02cf10b559a3bf9f1081ef915e8178590b835cc1dc2bf3a5535062133'
ARCHIVE_SHA256 = '6d16d4320f8b75d4382edf51fa047e5808907b6352e93a4dc27924e43489530a'
ARCHIVE_BYTES = 79022015


def digest_file(path):
    sha = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(block)
    return sha.hexdigest()


def download(url, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    with urlopen(Request(url, headers={'User-Agent': 'RoboLaw-asset-restore/1'}), timeout=120) as response:
        with partial.open('wb') as output:
            shutil.copyfileobj(response, output, length=1024 * 1024)
    partial.replace(path)


def safe_target(name):
    rel = Path(name)
    if rel.is_absolute() or '..' in rel.parts or not (
        name.startswith('assets/kitchen/') or
        name.startswith('assets/people/mpfb_corridor/')
    ):
        raise ValueError('Unexpected archive path: ' + name)
    return ROOT / rel


def valid_file(row):
    path = safe_target(row['path'])
    return path.is_file() and path.stat().st_size == row['bytes'] and digest_file(path) == row['sha256']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    cache = ROOT / 'assets/.downloads'
    manifest_path = cache / 'robolaw-manifest.json'
    if not manifest_path.exists() or digest_file(manifest_path) != MANIFEST_SHA256:
        if args.verify_only:
            parser.error('Verified manifest is unavailable; run without --verify-only first')
        download(BASE + 'manifest.json', manifest_path)
    if digest_file(manifest_path) != MANIFEST_SHA256:
        raise RuntimeError('ModelScope manifest SHA-256 differs from this release')
    manifest = json.loads(manifest_path.read_text())
    rows = manifest['files']
    expected = {row['path'] for row in rows}
    if len(expected) != len(rows) or len(rows) != 56:
        raise RuntimeError('Unexpected asset manifest')
    for row in rows:
        safe_target(row['path'])
    missing = [row for row in rows if not valid_file(row)]
    if not missing:
        print('RoboLaw assets ready: 56 files, SHA-256 verified')
        return 0
    if args.verify_only:
        print('Missing or invalid asset files:', len(missing))
        return 1
    archive = cache / 'robolaw-assets-v1.tar.gz'
    if not archive.is_file() or archive.stat().st_size != ARCHIVE_BYTES or digest_file(archive) != ARCHIVE_SHA256:
        print('Downloading ModelScope RoboLaw asset bundle (79 MB)', flush=True)
        download(BASE + 'robolaw-assets-v1.tar.gz', archive)
    if archive.stat().st_size != ARCHIVE_BYTES or digest_file(archive) != ARCHIVE_SHA256:
        raise RuntimeError('ModelScope archive SHA-256 or size mismatch')
    with tarfile.open(archive, 'r:gz') as bundle:
        members = bundle.getmembers()
        if {member.name for member in members} != expected or len(members) != len(expected):
            raise RuntimeError('Archive contents differ from the signed manifest')
        by_name = {row['path']: row for row in rows}
        for member in members:
            if not member.isfile():
                raise RuntimeError('Archive contains a non-file entry: ' + member.name)
            row = by_name[member.name]
            target = safe_target(member.name)
            if valid_file(row):
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            partial = target.with_name(target.name + '.partial')
            with bundle.extractfile(member) as source, partial.open('wb') as output:
                shutil.copyfileobj(source, output, length=1024 * 1024)
            if partial.stat().st_size != row['bytes'] or digest_file(partial) != row['sha256']:
                partial.unlink(missing_ok=True)
                raise RuntimeError('Asset checksum mismatch: ' + member.name)
            partial.replace(target)
    invalid = [row['path'] for row in rows if not valid_file(row)]
    if invalid:
        raise RuntimeError('Assets did not verify: ' + ', '.join(invalid[:3]))
    print('RoboLaw assets ready: 56 files, SHA-256 verified')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
