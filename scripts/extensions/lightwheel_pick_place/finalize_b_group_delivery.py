"""Verify frozen B-group task-scope evidence, source snapshot and handoff artifacts."""
import datetime,hashlib,json,os,subprocess,sys
from pathlib import Path
repo=Path(__file__).resolve().parents[3];os.chdir(repo);root=Path('/root/autodl-tmp');out=root/'B-group-review';scripts=repo/'scripts/extensions/lightwheel_pick_place';head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip();assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='lgd';assert subprocess.run(['git','merge-base','--is-ancestor','origin/ylj','HEAD']).returncode==0
py=sys.executable
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def run(args,log):
 with Path(log).open('a') as f:subprocess.run(args,check=True,stdout=f,stderr=subprocess.STDOUT)
def ep(n):return 'datasets/skill_trials/lightwheel_pick_place/'+n
base=ep('base_lgd_b04_fresh_matched_20261028T102807Z');selected={'human':ep('human_lgd_b04_fresh_matched_20261028T104647Z'),'property':ep('property_lgd_b04_fresh_matched_20261028T114946Z'),'robot':ep('robot_lgd_b04_fresh_matched_20261028T111828Z')};release=root/'B04-review/acceptance_manifest.release_20261028.json'
if not release.exists():
 args=[py,str(scripts/'task_manifest.py'),'--task','kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian','--task-scope','--baseline',base]
 for v,path in selected.items():args+=['--'+v,path,'--'+v+'-avoidance-review',str(Path(path)/('avoidance_audit.fresh_matched.json' if v=='property' else 'avoidance_audit.fresh_matched.policy_review.json'))]
 args+=['--neutral',str(root/'B04-review/task_neutral_90_extendedwindow_20261026/neutral_hold.json'),'--paired',*[str(root/'B04-review'/n/'paired_planning.json') for n in ['paired_fresh_common_human102807_execution_policy_20261028','paired_fresh_common_property_102807_20261028','paired_fresh_common_robot_102807_20261028']],'--output',str(release)]
 run(args,out/'final_manifest_generation.log')
m=json.loads(release.read_text());assert m['passed'] and m['implementation_git_commit']==head
for name,digest in m['implementation_sha256'].items():
 p=repo/name;assert sha(p)==digest,name;blob=subprocess.check_output(['git','show',head+':'+name]);assert hashlib.sha256(blob).hexdigest()==digest,'Uncommitted required source: '+name;
 if p.suffix=='.py':compile(blob,name,'exec')
 elif p.suffix=='.sh':subprocess.run(['bash','-n',str(p)],check=True)
run([py,str(scripts/'restore_control_reference_files.py')],out/'final_control_reference_check.log')
run([py,str(scripts/'build_group_scope_reviews.py')],out/'final_scope_refresh.log')
index=json.loads((out/'current_delivery_inventory.json').read_text());assert index['task_scope_acceptance_complete'] and all(t.get('task_scope_passed') for t in index['tasks'].values())
for task,t in index['tasks'].items():
 for v,row in t.get('episodes',{}).items():
  result=json.loads((Path(row['episode'])/'result.json').read_text());cfg=result.get('variant_config_source',{})
  if cfg.get('path'):
   p=Path(cfg['path']);relative=p.relative_to(repo) if p.is_absolute() else p;assert hashlib.sha256(subprocess.check_output(['git','show',head+':'+str(relative)])).hexdigest()==cfg['sha256']
legacy=out/'B01-B05-evidence-review-20261028-v2.zip';legacy_digest=sha(legacy);assert legacy.stat().st_size==687610850;assert legacy_digest==legacy.with_suffix('.sha256').read_text().split()[0]
archive=out/'B03-B04-current-accepted-supplement-20261028.zip'
if not archive.exists():run([py,str(scripts/'package_current_b03_b04_supplement.py'),'--output',str(archive)],out/'final_supplement_generation.log')
receipt=json.loads(archive.with_suffix('.receipt.json').read_text());assert receipt['crc_verified'] and receipt['required_missing']==0 and receipt['implementation_commit']==head and sha(archive)==receipt['sha256']
bundle=out/'lgd-B01-B05-final-20261028.bundle'
if not bundle.exists():run(['git','bundle','create',str(bundle),'lgd'],out/'final_bundle_verify.log')
run(['git','bundle','verify',str(bundle)],out/'final_bundle_verify.log');heads=subprocess.check_output(['git','bundle','list-heads',str(bundle)],text=True).splitlines();assert head+' refs/heads/lgd' in heads
artifacts={'historical_v2':{'path':str(legacy),'bytes':legacy.stat().st_size,'sha256':legacy_digest,'note':'Unchanged previously CRC-verified historical package; latest selected manifests supersede old remaining-item text.'},'current_supplement':receipt,'full_git_bundle':{'path':str(bundle),'bytes':bundle.stat().st_size,'sha256':sha(bundle),'verified':True,'head':head}}
gate={'passed':True,'as_of_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'implementation_commit':head,'branch':'lgd','origin_ylj':subprocess.check_output(['git','rev-parse','origin/ylj'],text=True).strip(),'known_latest_origin_ylj_is_ancestor':True,'all_task_scope_acceptance_complete':True,'task_scope_passed':{k:v['task_scope_passed'] for k,v in index['tasks'].items()},'full_unexcluded_review_claimed':False,'scope_note':'User exclusions of unrelated global kitchen-fixture joint investigation remain explicit and are not claimed passed. Human additional30mm planning reserve remains false; original20mm execution constraints pass. First-success timestamps for late-observation timeout flags are inferred from original termination control flow, not separately recorded.','required_source_files_committed_and_syntax_checked':len(m['implementation_sha256']),'control_reference_file_hashes_verified':True,'artifacts':artifacts,'github_push_performed':False,'preexisting_unused_work':'Preserved in working tree and tracked-file patch; not silently mixed with accepted task recipes.'}
(out/'final_handoff_gate_20261028.json').write_text(json.dumps(gate,indent=2))
run([py,str(scripts/'build_delivery_inventory.py')],out/'final_scope_refresh.log');final=json.loads((out/'current_delivery_inventory.json').read_text());assert final['delivery_complete']
text='# B-group final handoff\n\nAll five tasks passed the explicitly confirmed task-scope acceptance. B03/B04 selected manifests have no remaining task items. Branch lgd contains latest fetched origin/ylj and the verified source snapshot below. No GitHub push was performed.\n\nCommit: '+head+'\n\n'
for k,v in artifacts.items():text+='- '+k+': '+v['path']+'\n  SHA256: '+v['sha256']+'\n'
text+='\nFinal status: final_handoff_gate_20261028.json and current_delivery_inventory.json. Use historical v2 plus current supplement; historical false reports do not override newly selected accepted reports. Original images.h5 and large assets remain intact at server paths/hashes in selected manifests; videos/state-action trajectories/scenes/control inputs/current and historical reports/plots are in the archive.\n\nScope: '+gate['scope_note']+'\n\nPush only personal lgd after normal authentication, never main or force push. Existing unused local experiments are preserved separately.\n'
(out/'FINAL_RELEASE_NOTE_20261028.md').write_text(text)
print(json.dumps({'passed':True,'branch':'lgd','commit':head,'task_scope_passed':gate['task_scope_passed'],'archive_bytes':receipt['bytes'],'bundle_bytes':bundle.stat().st_size,'delivery_complete':True,'pushed':False}))
