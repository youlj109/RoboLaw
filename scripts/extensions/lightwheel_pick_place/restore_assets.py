#!/usr/bin/env python3
"""Restore Lime task objects and its dependencies from the Arena registry source."""
from pathlib import Path
from urllib.parse import urljoin, urlparse
from urllib.request import urlopen, Request
import concurrent.futures, hashlib, json, os, re, time
from pxr import Sdf
ROOT = Path(__file__).resolve().parents[3]
BASE = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/"
DEST = ROOT / "assets/extensions/lightwheel_pick_place/downloads"
START = [BASE + "Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/" + p for p in ("fruits_veggies/lime01.usd", "hot3d/wooden_bowl.usd", "objaverse/red_bell_pepper.usd")]

def fetch(url):
    assert url.startswith(BASE), url
    path = DEST / url[len(BASE):]
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        tmp = path.with_suffix(path.suffix + '.part')
        for attempt in range(4):
            try:
                with urlopen(Request(url, headers={'User-Agent': 'RoboLaw asset restore'}), timeout=120) as response, tmp.open('wb') as out:
                    size = int(response.headers.get('Content-Length',0)); n = 0
                    while block := response.read(1024*1024): out.write(block); n += len(block)
                if size and n != size: raise RuntimeError(f'incomplete {n}/{size}')
                tmp.replace(path); break
            except Exception:
                if attempt == 3: raise
                time.sleep(attempt+1)
    deps = []
    if path.suffix in ('.usd','.usda','.usdc'):
        layer = Sdf.Layer.FindOrOpen(str(path))
        if not layer: raise RuntimeError(f'invalid USD {path}')
        deps = list(layer.GetExternalReferences())
        # Includes shader textures as well as composition references.
        text = layer.ExportToString()
        # Asset-valued fields and composition references only; source provenance strings
        # can contain author workstation paths that are not USD dependencies.
        deps += [x for line in text.splitlines() if not line.lstrip().startswith('string ') for x in re.findall(r'@([^@]+)@', line)]
    elif path.suffix == '.mdl':
        s = path.read_text(errors='replace')
        deps += re.findall(r'texture_2d\(\s*"([^"\n]+)"',s)
        deps += re.findall(r'import\s+\.::(\w+)::',s)
        deps += re.findall(r'using\s+([A-Za-z_]\w*)\s+import',s)
        deps = [x+'.mdl' if '.' not in x else x for x in deps]
    builtin = {'OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl','OmniSurfaceLite.mdl'}
    deps = [urljoin(url, x) for x in set(deps) if x not in builtin and (('/' in x) or ('.' in x))]
    return dict(url=url,path=str(path.relative_to(ROOT)),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()), deps

def main():
    pending=set(START); seen=set(); records=[]; failures=[]
    while pending:
        batch=sorted(pending-seen); pending=set(); seen.update(batch)
        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
            futures={pool.submit(fetch,u):u for u in batch}
            for f in concurrent.futures.as_completed(futures):
                u=futures[f]
                try:
                    row,deps=f.result();records.append(row);pending.update(set(deps)-seen);print('ASSET',row['path'],flush=True)
                except Exception as e:
                    failures.append(dict(url=u,error=str(e)));print('FAILED',u,str(e),flush=True)
        report=dict(source_registry='isaaclab_arena/assets/background_library.py',files=records,failures=failures,complete=not failures)
        (ROOT/'reports/skill_trials/lightwheel_pick_place/nvidia_assets.json').write_text(json.dumps(report,indent=2))
    return bool(failures)
if __name__=='__main__':raise SystemExit(main())
