#!/usr/bin/env python3
"""Restore the 13 pinned NVIDIA assets from official URLs or ModelScope.

Both sources are checked against identical per-file SHA-256 checksums.
Upstream licenses remain applicable to mirrored copies.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import time
from urllib.request import Request, urlopen
from dependency_bundle import digest, restore_bundle

ROOT = Path(__file__).resolve().parents[1]


def local_path(root: Path, value: str) -> Path:
    path = (root / value).resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError(f"Asset destination leaves project root: {value}") from error
    return path


def fetch(root: Path, entry: dict, attempts: int = 3) -> dict:
    path = local_path(root, entry['path'])
    if path.is_file() and path.stat().st_size == entry['bytes'] and digest(path) == entry['sha256']:
        return {'path': entry['path'], 'status': 'existing', 'bytes': path.stat().st_size}
    if path.exists():
        return {'path': entry['path'], 'status': 'failed', 'error': 'Existing asset differs from pinned checksum; move it aside before restoring'}
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.part')
    for attempt in range(attempts):
        try:
            request = Request(entry['url'], headers={'User-Agent': 'Isaac-Arena-Kitchen-Safety/asset-restore'})
            with urlopen(request, timeout=60) as response, temporary.open('wb') as output:
                expected = response.headers.get('Content-Length')
                received = 0
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
                    received += len(chunk)
            if not received:
                raise RuntimeError('Server returned an empty asset')
            if expected is not None and received != int(expected):
                raise RuntimeError(f'Incomplete download: {received}/{expected} bytes')
            if received != entry['bytes'] or digest(temporary) != entry['sha256']:
                raise RuntimeError('Asset differs from pinned size or SHA-256')
            os.replace(temporary, path)
            return {'path': entry['path'], 'status': 'downloaded', 'bytes': received}
        except Exception as error:
            if attempt + 1 == attempts:
                return {'path': entry['path'], 'status': 'failed', 'error': str(error)}
            time.sleep(attempt + 1)
    raise AssertionError('unreachable')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=['official', 'modelscope'], default=os.environ.get('ROBOLAW_DOWNLOAD_SOURCE', 'official'))
    parser.add_argument('--root', type=Path, default=ROOT, help='Restored project root; defaults to this script parent project')
    parser.add_argument('--manifest', type=Path, default=ROOT / 'configs/external_assets.json')
    parser.add_argument('--include-optional-nvidia-character', action='store_true', help='Legacy flag; this release contains only the 13 required assets')
    parser.add_argument('--verify-only', action='store_true', help='Verify local file sizes and SHA-256 without downloading')
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    if args.source not in ('official', 'modelscope'):
        parser.error('ROBOLAW_DOWNLOAD_SOURCE must be official or modelscope')
    root = args.root.resolve()
    if args.source == 'modelscope':
        restore_bundle('nvidia_assets', root, args.verify_only)
        return 0
    manifest = json.loads(args.manifest.read_text())
    entries = [entry for entry in manifest['assets'] if entry['required'] or args.include_optional_nvidia_character]
    if args.verify_only:
        results = []
        for entry in entries:
            path = local_path(root, entry['path'])
            present = path.is_file() and path.stat().st_size == entry['bytes'] and digest(path) == entry['sha256']
            results.append({'path': entry['path'], 'status': 'existing' if present else 'missing', 'bytes': path.stat().st_size if present else 0})
    else:
        print('Downloading external assets from NVIDIA. Their upstream terms apply:', flush=True)
        print(manifest['license_url'], flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, min(args.workers, 8))) as pool:
            tasks = [pool.submit(fetch, root, entry) for entry in entries]
            results = []
            for task in concurrent.futures.as_completed(tasks):
                row = task.result()
                results.append(row)
                print(f"{row['status']}: {row['path']}", flush=True)
            order = {entry['path']: index for index, entry in enumerate(entries)}
            results.sort(key=lambda row: order[row['path']])
    failed = [row for row in results if row['status'] in ('failed', 'missing')]
    report = {'complete': not failed, 'source': args.source, 'mode': 'verify_only' if args.verify_only else 'restore', 'selected_files': len(entries), 'optional_character_sources_requested': args.include_optional_nvidia_character, 'license_url': manifest['license_url'], 'results': results}
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if not args.verify_only:
        destination = root / 'reports/external_assets_restore.json'
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
