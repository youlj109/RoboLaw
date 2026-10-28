"""Restore immutable JSON control input files, never simulation states or actions."""
import argparse,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--destination-root',type=Path,default=Path('/root/autodl-tmp'));p.add_argument('--apply',action='store_true');a=p.parse_args();repo=Path(__file__).resolve().parents[3];dest=a.destination_root.resolve()
for family in ['b03_fresh_common_20261028','b04_fresh_common_20261028']:
 index=repo/'configs/extensions/lightwheel_pick_place'/family/'reference_index.json'
 for row in json.loads(index.read_text())['files']:
  source=repo/row['repository_file'];data=source.read_bytes();assert hashlib.sha256(data).hexdigest()==row['sha256']
  relative=Path(row['original_runtime_path']).relative_to('/root/autodl-tmp');target=(dest/relative).resolve();assert target.is_relative_to(dest) and relative.parts[0] in ('B03-review','B04-review')
  if target.exists():assert target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest()==row['sha256'],'Existing different input: '+str(target)
  elif a.apply:target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
  print('MATCH_OR_RESTORE' if a.apply else 'CHECK_ONLY',target)
