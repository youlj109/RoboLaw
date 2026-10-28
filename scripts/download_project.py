#!/usr/bin/env python3
"""Download the public project release with stdlib Python; no access token needed."""
import argparse
import json
from pathlib import Path
import shutil
import tarfile
import time
from urllib.parse import quote
from urllib.request import Request, urlopen

DEFAULT_REPO = 'YouLinjing/Isaac-Arena-Kitchen-Safety'


def fetch(url, target, expected_size=None):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and (expected_size is None or target.stat().st_size == expected_size):
        return target
    partial = target.with_name(target.name + '.partial')
    offset = partial.stat().st_size if partial.exists() else 0
    headers = {'User-Agent': 'arena-project-downloader/1'}
    if offset:
        headers['Range'] = f'bytes={offset}-'
    with urlopen(Request(url, headers=headers), timeout=120) as response:
        append = offset > 0 and response.status == 206
        written = offset if append else 0
        last = time.monotonic()
        with partial.open('ab' if append else 'wb') as stream:
            while True:
                block = response.read(8 * 1024 * 1024)
                if not block:
                    break
                stream.write(block)
                written += len(block)
                if time.monotonic() - last >= 10:
                    print(f'DOWNLOAD {target.name}: {written / 1e9:.2f} GB', flush=True)
                    last = time.monotonic()
    if expected_size is not None and partial.stat().st_size != expected_size:
        raise IOError(f'Incomplete download: {target}; rerun to resume the partial file')
    partial.replace(target)
    return target


def extract(archive, destination):
    # Release archives contain regular files/directories, never device nodes or
    # symlinks. Check paths on Python versions without tarfile's data filter.
    with tarfile.open(archive, 'r:gz') as tar:
        for member in tar:
            name = Path(member.name)
            if name.is_absolute() or '..' in name.parts or not (member.isfile() or member.isdir()):
                raise ValueError(f'Unsupported archive entry: {member.name}')
            target = (destination / name).resolve()
            if destination != target and destination not in target.parents:
                raise ValueError(f'Archive entry escapes destination: {member.name}')
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(member) as source, target.open('wb') as output:
                    shutil.copyfileobj(source, output, length=8 * 1024 * 1024)
                target.chmod(member.mode & 0o777)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', default=DEFAULT_REPO)
    parser.add_argument('--dest', default='Isaac-Arena-Kitchen-Safety')
    parser.add_argument('--data', choices=['none', 'human4', 'all'], default='none',
                        help='none: runnable project; human4: its recorded search/accepted runs; all: all historical data')
    parser.add_argument('--data-only', action='store_true', help='Add recorded data without overwriting project source')
    parser.add_argument('--keep-archives', action='store_true', help='Retain archives after successful extraction')
    args = parser.parse_args()
    destination = Path(args.dest).expanduser().resolve()
    if args.data_only and args.data == 'none':
        parser.error('--data-only requires --data human4 or all')
    base = 'https://modelscope.cn/datasets/' + args.repo + '/resolve/master/'
    with urlopen(base + 'release.json', timeout=60) as response:
        release = json.load(response)
    destination.mkdir(parents=True, exist_ok=True)
    cache = destination / '.download-cache'
    state_file = cache / 'extracted.json'
    state = json.loads(state_file.read_text()) if state_file.exists() else []
    started_file = cache / 'started.json'
    expected_start = {'repo': args.repo, 'destination': str(destination)}
    started = json.loads(started_file.read_text()) if started_file.exists() else None
    if started is not None and started != expected_start:
        parser.error('Download progress belongs to another repository or destination; choose a new --dest')
    if not args.data_only:
        nonempty = any(p.name != '.download-cache' for p in destination.iterdir())
        project_extracted = release['project']['path'] in state
        if nonempty and started != expected_start and not project_extracted:
            parser.error('Choose an empty --dest; use --data-only to add data to an existing project')
        # Record ownership only after the empty-directory check. If extraction
        # is interrupted before extracted.json is written, this permits the
        # same download to finish without permitting arbitrary directory writes.
        cache.mkdir(parents=True, exist_ok=True)
        temporary_start = started_file.with_suffix('.tmp')
        temporary_start.write_text(json.dumps(expected_start, indent=2) + '\n')
        temporary_start.replace(started_file)
    selected = [] if args.data_only else [release['project']]
    if args.data == 'all':
        selected.extend(release['data'])
    elif args.data == 'human4':
        selected.extend(item for item in release['data'] if item.get('group') in ('human_lower_first_validation', 'index'))
    for entry in selected:
        name = entry['path']
        if name in state:
            print('ALREADY_EXTRACTED', name, flush=True)
            continue
        archive = cache / Path(name).name
        print('FETCH', name, flush=True)
        for attempt in range(4):
            try:
                fetch(base + quote(name, safe='/'), archive, entry['bytes'])
                break
            except (OSError, TimeoutError) as error:
                if attempt == 3:
                    raise
                print(f'Retrying download ({type(error).__name__})', flush=True)
                time.sleep(2 * (attempt + 1))
        extract(archive, destination)
        state.append(name)
        state_file.parent.mkdir(parents=True, exist_ok=True)
        state_file.write_text(json.dumps(state, indent=2) + '\n')
        if not args.keep_archives:
            archive.unlink()
        print('EXTRACTED', name, flush=True)
    print(f'Project directory: {destination}')
    if not args.data_only:
        print('Next: cd into the project, then bash scripts/setup_arena.sh --accept-eula')
        print('The setup downloads licensed NVIDIA assets from their official source.')


if __name__ == '__main__':
    main()
