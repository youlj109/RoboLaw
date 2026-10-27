"""Package real selected evidence without upgrading any acceptance result."""
import argparse,json,hashlib,zipfile,datetime,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
subprocess.run(['/root/autodl-tmp/conda_envs/robolaw_data/bin/python','scripts/extensions/lightwheel_pick_place/build_delivery_inventory.py'],check=True)
inventory=Path('/root/autodl-tmp/B-group-review/current_delivery_inventory.json');data=json.loads(inventory.read_text());sources={};missing=[]
def add(path,name):
 path=Path(path)
 if path.is_file():sources[name]=path
 else:missing.append(str(path))
add(inventory,'current_delivery_inventory.json')
for task,meta in data['tasks'].items():
 episodes=dict(meta.get('episodes',{}))
 if task=='B03':episodes['baseline']={'episode':'datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_fixedscene_20261026T080313Z'}
 if task=='B04':episodes['historical_baseline']={'episode':'datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_handle_20261025T194846Z'}
 for variant,row in episodes.items():
  episode=Path(row['episode']);prefix=f'{task}/{variant}/{episode.name}'
  for camera in ('external_camera','external_camera_2','wrist_camera'):add(episode/(camera+'.mp4'),prefix+'/'+camera+'.mp4')
  for fn in ('result.json','validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','fixed_base_hazard_audit.json','contacts.json','trajectory.h5','initial_setup.json','cameras.json','collector_source_snapshot.py'):
   add(episode/fn,prefix+'/'+fn)
  for pattern in ('avoidance*json','source_background*json'):
   for file in episode.glob(pattern):add(file,prefix+'/'+file.name)
 if 'manifest' in meta:add(meta['manifest'],task+'/existing_acceptance_manifest.json')
 review=Path('/root/autodl-tmp')/(task+'-review')
 for pattern in ('*existing_baseline_comparison.json','*final_existing_baseline_comparison.json','paired_*/paired_planning.json','acceptance_manifest.json','scope_review_manifest.json','*context_review.json'):
  for file in review.glob(pattern):add(file,task+'/review/'+str(file.relative_to(review)))
# Include exact collected scene configs as provenance, never private handoffs or SSH logs.
for name,path in list(sources.items()):
 if path.name=='result.json':
  r=json.loads(path.read_text());cfg=r.get('variant_config_source',{})
  if cfg.get('path'):
   file=Path(cfg['path'])
   if file.exists() and hashlib.sha256(file.read_bytes()).hexdigest()==cfg['sha256']:add(file,'configs/'+file.name)
size=sum(f.stat().st_size for f in sources.values())
if size>6*1024**3:raise RuntimeError('Review package exceeds6GiB budget; inspect selected inputs')
manifest={'generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Evidence package, not automatic acceptance. Native execution, collision safety, single-baseline necessity, matched paired planning and strict cross-episode comparisons remain separate. Unpassed/absent checks retained. Images HDF5 and scene assets remain on server; original state/action HDF5 and three original camera videos are included.','delivery_complete':data['delivery_complete'],'input_bytes':size,'missing_inputs':missing,'files':{name:{'server_path':str(file.resolve()),'bytes':file.stat().st_size} for name,file in sources.items()}}
a.output.parent.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(a.output,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
 for name,file in sorted(sources.items()):
  z.write(file,name,compress_type=zipfile.ZIP_STORED if file.suffix=='.mp4' else zipfile.ZIP_DEFLATED)
 z.writestr('PACKAGE_INDEX.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(a.output) as z:
 bad=z.testzip()
 if bad:raise RuntimeError('Archive CRC failure: '+bad)
with a.output.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
a.output.with_suffix('.sha256').write_text(digest+'  '+a.output.name+'\n');print('PACKAGE_VERIFIED',a.output,'bytes',a.output.stat().st_size,'missing',len(missing),flush=True)
