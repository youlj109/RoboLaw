#!/usr/bin/env python3
"""Host-safe recursive NVIDIA asset restore for lime task (no Docker).

Works around Sdf.Layer.FindOrOpen raising Tf.ErrorException on this AutoDL image
by using Usd.Stage.Open on repo-relative paths and walking composition/texture refs.
"""
from __future__ import annotations
from pathlib import Path
from urllib.parse import urljoin
from urllib.request import Request, urlopen
import hashlib, json, re, time, os

# Avoid Isaac plugin path poisoning crate format detection.
os.environ.pop('PXR_PLUGINPATH_NAME', None)
os.environ.pop('USD_PLUGIN_PATH', None)

from pxr import Usd  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
BASE = 'https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/'
DEST = ROOT / 'assets/extensions/lightwheel_pick_place/downloads'
START = [
    BASE + 'Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/' + p
    for p in (
        'fruits_veggies/lime01.usd',
        'hot3d/wooden_bowl.usd',
        'objaverse/red_bell_pepper.usd',
        'fruits_veggies/lemon_01.usd',
        'fruits_veggies/orange_01.usd',
        'objaverse/apple_01.usd',
        'ycb/bowl.usd',
        'ycb/banana.usd',
        'vomp/plate_large/plate_large.usd',
    )
]
BUILTIN = {
    'OmniPBR.mdl', 'OmniGlass.mdl', 'OmniSurface.mdl', 'OmniSurfaceLite.mdl',
}


def download(url: str) -> Path:
    assert url.startswith(BASE), url
    path = DEST / url[len(BASE):]
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        return path
    tmp = path.with_suffix(path.suffix + '.part')
    for attempt in range(5):
        try:
            with urlopen(Request(url, headers={'User-Agent': 'RoboLaw-lgd host restore'}), timeout=180) as resp, tmp.open('wb') as out:
                size = int(resp.headers.get('Content-Length') or 0)
                n = 0
                while True:
                    block = resp.read(1024 * 1024)
                    if not block:
                        break
                    out.write(block)
                    n += len(block)
            if size and n != size:
                raise RuntimeError(f'incomplete {n}/{size}')
            tmp.replace(path)
            return path
        except Exception:
            if attempt == 4:
                raise
            time.sleep(attempt + 1)
    return path


def deps_of(path: Path, url: str) -> list[str]:
    rel = path.relative_to(ROOT).as_posix()
    # Prefer Stage.Open; fall back silently if empty.
    stage = Usd.Stage.Open(rel)
    if not stage:
        raise RuntimeError(f'Usd.Stage.Open failed for {rel}')
    layer = stage.GetRootLayer()
    refs = list(layer.GetExternalReferences())
    text = layer.ExportToString()
    refs += [
        x for line in text.splitlines()
        if not line.lstrip().startswith('string ')
        for x in re.findall(r'@([^@]+)@', line)
    ]
    if path.suffix == '.mdl':
        s = path.read_text(errors='replace')
        refs += re.findall(r'texture_2d\(\s*"([^"\n]+)"', s)
        refs += re.findall(r'import\s+\.::(\w+)::', s)
        refs += re.findall(r'using\s+([A-Za-z_]\w*)\s+import', s)
        refs = [x + '.mdl' if '.' not in Path(x).name else x for x in refs]
    out = []
    for x in set(refs):
        if not x or x in BUILTIN:
            continue
        if '/' not in x and '.' not in x:
            continue
        out.append(urljoin(url, x))
    return out


def main() -> int:
    os.chdir(ROOT)
    pending = set(START)
    seen: set[str] = set()
    records = []
    failures = []
    while pending:
        batch = sorted(pending - seen)
        pending = set()
        seen.update(batch)
        for u in batch:
            try:
                path = download(u)
                row = {
                    'url': u,
                    'path': str(path.relative_to(ROOT)),
                    'bytes': path.stat().st_size,
                    'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                }
                records.append(row)
                print('ASSET', row['path'], flush=True)
                if path.suffix.lower() in {'.usd', '.usda', '.usdc', '.mdl'}:
                    pending.update(set(deps_of(path, u)) - seen)
            except Exception as e:
                failures.append({'url': u, 'error': str(e)})
                print('FAILED', u, str(e), flush=True)
        report = {
            'source_registry': 'isaaclab_arena/assets/background_library.py',
            'files': records,
            'failures': failures,
            'complete': not failures,
            'host_restore': True,
        }
        out = ROOT / 'reports/skill_trials/lightwheel_pick_place'
        out.mkdir(parents=True, exist_ok=True)
        (out / 'nvidia_assets.json').write_text(json.dumps(report, indent=2) + '\n')
    print('COMPLETE' if not failures else f'FAILURES={len(failures)}', 'files=', len(records), flush=True)
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
