#!/usr/bin/env python3
"""Restore G-shape kitchen and its dependencies from the Arena registry source."""
from pathlib import Path
from urllib.parse import urljoin, urlparse
from urllib.request import urlopen, Request
import concurrent.futures, hashlib, json, os, re, time
from pxr import Sdf
ROOT = Path(__file__).resolve().parents[3]
BASE = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/"
DEST = ROOT / "assets/extensions/replicator/source"
START = BASE + "Isaac/Environments/replicator_kitchen/kitchen_g_shape.usda"

def fetch(url):
    assert url.startswith(BASE), url
    path = DEST / url[len(BASE):]
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        tmp = path.with_suffix(path.suffix + '.part')
        for attempt in range(4):
            try:
                offset=tmp.stat().st_size if tmp.exists() else 0
                headers={'User-Agent':'RoboLaw asset restore'}
                if offset:headers['Range']=f'bytes={offset}-'
                with urlopen(Request(url,headers=headers), timeout=120) as response:
                    resumed=response.status==206 and offset>0
                    if resumed and not response.headers.get('Content-Range','').startswith(f'bytes {offset}-'):
                        raise RuntimeError('Unexpected range offset')
                    expected=int(response.headers.get('Content-Length',0)); n=0
                    with tmp.open('ab' if resumed else 'wb') as out:
                        while block:=response.read(1024*1024):out.write(block);n+=len(block)
                if expected and n != expected:raise RuntimeError(f'incomplete response {n}/{expected}')
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
        # assetInfo.identifier is provenance, not a render/composition dependency.
        text = re.sub(r'asset identifier = @[^@]+@', '', text)
        text='\n'.join(line for line in text.splitlines() if not re.match(r'\s*(?:custom )?string\s',line))
        deps += re.findall(r'@([^@]+)@', text)
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
    pending={START}; seen=set(); records=[]; failures=[]; futures={}
    report_path=ROOT/'reports/skill_trials/replicator/assets.json'
    report_path.parent.mkdir(parents=True,exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        while pending or futures:
            # Start composition layers before large appearance textures.
            ordered=sorted(pending-seen,key=lambda u:(not u.endswith(('.usd','.usda','.usdc','.mdl')),u))
            for u in ordered[:max(0,6-len(futures))]:
                seen.add(u);pending.discard(u);futures[pool.submit(fetch,u)]=u
            pending-=seen
            if not futures:break
            completed,_=concurrent.futures.wait(futures,return_when=concurrent.futures.FIRST_COMPLETED)
            for f in completed:
                u=futures.pop(f)
                try:
                    row,deps=f.result();records.append(row);pending.update(set(deps)-seen);print('ASSET',row['path'],flush=True)
                except Exception as e:
                    failures.append(dict(url=u,error=str(e)));print('FAILED',u,str(e),flush=True)
            report=dict(source_registry='isaaclab_arena/assets/background_library.py',files=records,failures=failures,pending=len(pending)+len(futures),complete=not(failures or pending or futures))
            report_path.write_text(json.dumps(report,indent=2))
    return bool(failures)
if __name__=='__main__':raise SystemExit(main())
