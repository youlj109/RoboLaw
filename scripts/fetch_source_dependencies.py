#!/usr/bin/env python3
"""Fetch pinned Arena, Isaac Lab and cuRobo source from upstream or ModelScope."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
from dependency_bundle import CONFIG, ROOT, restore_bundle


def git(path, *args, capture=False):
    return subprocess.run(['git', '-C', str(path), *args], check=True,
                          env={**os.environ, 'GIT_TERMINAL_PROMPT': '0'},
                          text=True, stdout=subprocess.PIPE if capture else None).stdout


def official(key, spec, root, verify_only):
    path = root / spec['path']
    if (path / '.git').exists():
        revision = git(path, 'rev-parse', 'HEAD', capture=True).strip()
        if revision != spec['revision']:
            raise RuntimeError(f'{path} is at {revision}; expected {spec["revision"]}. Use a new directory.')
        changed = git(path, 'status', '--porcelain', '--untracked-files=no', '--ignore-submodules=all', capture=True)
        if changed.strip():
            raise RuntimeError('Tracked source files were modified; keep this checkout and use a new directory: ' + str(path))
        print(f'{key}: pinned Git checkout verified ({revision})', flush=True)
        return
    if path.exists() and any(path.iterdir()):
        # A previously verified ModelScope snapshot is equally usable for installation.
        restore_bundle(key, root, verify_only=True)
        return
    if verify_only:
        raise RuntimeError('Missing source: ' + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=key + '-fetch-', dir=path.parent) as temporary:
        checkout = Path(temporary) / 'source'
        checkout.mkdir()
        git(checkout, 'init', '--quiet')
        git(checkout, 'remote', 'add', 'origin', spec['official_url'])
        # Explicit configuration supports older Git versions, including 2.25.
        git(checkout, 'config', 'core.repositoryformatversion', '1')
        git(checkout, 'config', 'extensions.partialClone', 'origin')
        git(checkout, 'config', 'remote.origin.promisor', 'true')
        git(checkout, 'config', 'remote.origin.partialclonefilter', 'blob:none')
        git(checkout, 'fetch', '--depth=1', '--filter=blob:none', 'origin', spec['revision'])
        if spec['excluded_paths']:
            git(checkout, 'config', 'core.sparseCheckout', 'true')
            sparse = checkout / '.git/info/sparse-checkout'
            sparse.write_text('/*\n' + ''.join('!/' + p + '\n' for p in spec['excluded_paths']))
        # Match the canonical Git blob bytes used by the ModelScope snapshot,
        # including Windows batch files with checkout-only CRLF attributes.
        (checkout / '.git/info/attributes').write_text('* -text\n')
        git(checkout, 'checkout', '--detach', spec['revision'])
        if path.exists():
            path.rmdir()  # Only an empty submodule placeholder is removable.
        checkout.rename(path)
    print(f'{key}: downloaded official revision {spec["revision"]}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=['official', 'modelscope'], default=os.environ.get('ROBOLAW_DOWNLOAD_SOURCE', 'official'))
    parser.add_argument('--components', nargs='+', choices=['arena', 'isaaclab', 'curobo'], default=['arena', 'isaaclab', 'curobo'])
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    if args.source not in ('official', 'modelscope'):
        parser.error('ROBOLAW_DOWNLOAD_SOURCE must be official or modelscope')
    specs = json.loads(CONFIG.read_text())['bundles']
    # Arena must be present before its Isaac Lab subdirectory is populated.
    selected = set(args.components)
    if 'isaaclab' in selected:
        selected.add('arena')
    for key in ['arena', 'isaaclab', 'curobo']:
        if key not in selected:
            continue
        if args.source == 'modelscope':
            restore_bundle(key, args.root, args.verify_only)
        else:
            official(key, specs[key], args.root.resolve(), args.verify_only)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
