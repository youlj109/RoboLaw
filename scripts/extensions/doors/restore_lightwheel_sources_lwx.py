"""Verify pinned packages; optionally restore missing packages without replacing caches."""
import argparse,hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('--download-missing',action='store_true');a=p.parse_args()
manifest=json.loads((ROOT/'configs/extensions/doors/lightwheel_sources_lwx_01.json').read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for item in manifest['sources']:
 dest=ROOT/item['destination'];scene=dest/'scene.usd'
 if dest.exists():
  if not scene.is_file() or sha(scene)!=item['scene_sha256']:raise RuntimeError('Existing destination differs; preserve it and investigate: '+str(dest))
  print(item['label'],'pinned scene verified');continue
 if not a.download_missing:raise FileNotFoundError(str(scene)+'; use --download-missing in an authenticated SDK environment')
 cache=Path.home()/'.cache/lightwheel_sdk/floorplan'/dest.name
 if cache.exists():
  if not (cache/'scene.usd').is_file() or sha(cache/'scene.usd')!=item['scene_sha256']:raise RuntimeError('Preserving different SDK cache: '+str(cache))
 else:
  from lightwheel_sdk.loader.floorplan import FloorplanLoader
  from lightwheel_sdk.client.client import LightwheelClient
  loader=FloorplanLoader(LightwheelClient())
  path,metadata=loader.get_usd(scene='robocasakitchen',layout_id=item['layout_id'],style_id=item['style_id'],backend='robocasa',version=item['version_id'])
  cache=Path(path).parent
  if metadata.get('version_id')!=item['version_id'] or sha(Path(path))!=item['scene_sha256']:raise RuntimeError('Downloaded package failed pinned identity check')
 if cache.resolve()!=dest.resolve():dest.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(cache,dest)
 if sha(scene)!=item['scene_sha256']:raise RuntimeError('Restored scene hash mismatch')
 print(item['label'],'restored pinned package')
