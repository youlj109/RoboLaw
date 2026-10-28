"""Read-only FCL regression for stage-contact registration; never native acceptance."""
import copy,hashlib,importlib.util,json
from pathlib import Path
root=Path('datasets/skill_trials/lightwheel_pick_place')
ep=root/'base_lgd_b03_matched_robot_20261028T023901Z'
source=root/'robot_lgd_b03_checkpoint_replay_20261027T033440Z'
spec=importlib.util.spec_from_file_location('world_audit','scripts/extensions/lightwheel_pick_place/audit_deferred_world.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
r=json.loads((ep/'result.json').read_text());saved=json.loads((source/'result.json').read_text());audit=json.loads((ep/'full_world_audit.json').read_text())
row=next(x for x in reversed(audit['measurements']) if x.get('valid') is False)
assert r['placement_checkpoints']['banana_1']['passed'] and r['placement_checkpoints']['banana_1']['action_index']<row['action']
assert saved['placement_checkpoints']['banana_1']['passed'] and saved['placement_contact_policy']
assert row['phase'] in ('lower','release','retreat','settle_final')
m.initialize(str(ep));before=m.check_batch([copy.deepcopy(row)])[0]
updated=copy.deepcopy(row);updated.pop('valid',None);updated['placed_task_contents']=['banana_1']
m.result['placement_contact_policy']=saved['placement_contact_policy']
after=m.check_batch([updated])[0]
keys=('valid','distance_m','required_m','slack_m','robot_part','obstacle','intersecting')
report={'scope':'Same saved geometry/state and original source collision policy, only verified first-fruit registration differs. Regression only, not native success or retrospective acceptance.','failed_episode':str(ep),'failed_result_sha256':hashlib.sha256((ep/'result.json').read_bytes()).hexdigest(),'original_success_policy_episode':str(source),'original_policy':saved['placement_contact_policy'],'action':row['action'],'phase':row['phase'],'first_native_checkpoint':r['placement_checkpoints']['banana_1'],'original_failed_context':{k:before.get(k) for k in keys},'original_success_contact_context':{k:after.get(k) for k in keys},'regression_passed':before['valid'] is False and after['valid'] is True,'native_acceptance_upgraded':False}
out=Path('/root/autodl-tmp/B03-review/standard_robot_stage_contact_context_review_20261028.json');out.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k in ('action','phase','original_failed_context','original_success_contact_context','regression_passed','native_acceptance_upgraded')}),flush=True)
