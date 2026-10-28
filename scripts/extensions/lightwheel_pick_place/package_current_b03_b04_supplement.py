"""One verified supplemental archive for newly accepted B03/B04 evidence only."""
import argparse,datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();repo=Path.cwd();root=Path('/root/autodl-tmp');sources={}
if a.output.exists():raise FileExistsError(a.output)
def add(path):
 path=Path(path).resolve();assert path.is_file(),str(path);name=str(path.relative_to(root));sources[name]=path
inventory=json.loads((root/'B-group-review/current_delivery_inventory.json').read_text());assert inventory['task_scope_acceptance_complete']
for task in ['B03','B04']:
 j=json.loads((root/(task+'-review')/'current_selected_acceptance_20261028.json').read_text());assert j['passed'] and not j['remaining']
assert json.loads((root/'B04-review/acceptance_manifest.release_20261028.json').read_text())['passed']
names=['base_lgd_b03_matched_property_20261028T063733Z','robot_lgd_b03_standard_stage_property_20261028T074115Z','property_lgd_b03_standard_stage_property_20261028T082633Z','base_lgd_b04_fresh_matched_20261028T102807Z','human_lgd_b04_fresh_matched_20261028T104647Z','property_lgd_b04_fresh_matched_20261028T114946Z','robot_lgd_b04_fresh_matched_20261028T111828Z']
for name in names:
 ep=repo/'datasets/skill_trials/lightwheel_pick_place'/name
 for cam in ['external_camera','external_camera_2','wrist_camera']:add(ep/(cam+'.mp4'))
 for pattern in ['*.json','*.mp4','trajectory.h5','scene.usda','collector_source_snapshot.py']:
  for file in ep.glob(pattern):add(file)
for name in ['robot_lgd_b03_round17_20261026T225657Z','robot_lgd_b03_checkpoint_replay_20261027T033440Z','property_lgd_b03_checkpoint_replay_20261027T064923Z','base_lgd_b04_handle_20261025T194846Z']:
 ep=repo/'datasets/skill_trials/lightwheel_pick_place'/name;add(ep/'result.json')
 if name.startswith('base_lgd_b04'):add(ep/'trajectory.h5')
for task in ['B03','B04']:
 review=root/(task+'-review')
 for pattern in ['current_selected_acceptance_20261028.json','scope_review_manifest.json','acceptance_manifest*20261028.json','*regression*20261028.json','*review*20261028.json','*proof*20261028.json','paired_fresh_*/*.json','paired_fresh_*/*.npy','fresh_*_final/*.png','fresh_*_final/*.pdf','fresh*visual*jpg','fresh_b04_human_overview_review.jpg','task_neutral*/*.json','task_neutral*/*.npz']:
  for file in review.glob(pattern):add(file)
for directory in ['b03_fresh_common_20261028','b04_fresh_common_20261028']:
 for file in (repo/'configs/extensions/lightwheel_pick_place'/directory).glob('*.json'):add(file)
for name in ['B01_B05_DELIVERY_REVIEW_20261028.md','B03_CURRENT_ACCEPTANCE_20261028.md','B04_CURRENT_ACCEPTANCE_20261028.md','ACCEPTANCE_REQUIREMENT_PROVENANCE_20261028.md']:
 add(repo/'docs/extensions/lightwheel_pick_place'/name)
for name in ['current_delivery_inventory.json','github_readiness_check_20261028.json','preserved_preexisting_local_changes_20261028.patch','premerge_preservation_20261028.json']:
 add(root/'B-group-review'/name)
size=sum(x.stat().st_size for x in sources.values());assert size<=6*1024**3,size
index={'generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'New accepted evidence supplement, used with preserved v2 historical package and final lgd Git bundle. No duplicate historical video repack. Original images.h5 remain intact on server, path/hash in selected acceptance manifests; videos/state-action HDF5/scenes/reports/current control inputs included. Raw false reports preserved. Final release SHA/CRC and Git-bundle verification are external receipts produced after archive closes.','all_task_scope_acceptance_complete':True,'full_unexcluded_review_claimed':False,'implementation_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'input_bytes':size,'files':{}}
a.output.parent.mkdir(exist_ok=True)
with zipfile.ZipFile(a.output,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
 for name,path in sorted(sources.items()):
  h=hashlib.sha256();info=zipfile.ZipInfo.from_file(path,name);info.compress_type=zipfile.ZIP_STORED if path.suffix=='.mp4' else zipfile.ZIP_DEFLATED
  with path.open('rb') as source,z.open(info,'w',force_zip64=True) as target:
   for block in iter(lambda:source.read(1024*1024),b''):h.update(block);target.write(block)
  index['files'][name]={'server_path':str(path),'bytes':path.stat().st_size,'sha256':h.hexdigest()}
 z.writestr('PACKAGE_INDEX.json',json.dumps(index,indent=2))
with zipfile.ZipFile(a.output) as z:assert z.testzip() is None
with a.output.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
a.output.with_suffix('.sha256').write_text(digest+'  '+a.output.name+'\n')
receipt={'path':str(a.output),'bytes':a.output.stat().st_size,'sha256':digest,'crc_verified':True,'members':len(sources)+1,'required_missing':0,'implementation_commit':index['implementation_commit']};a.output.with_suffix('.receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
