"""Read-only evidence inventory; does not rerun, upgrade or alter any acceptance report."""
from pathlib import Path
import json,re,datetime,subprocess
root=Path('datasets/skill_trials/lightwheel_pick_place');tasks={}
def passed(path):
 if not path.exists():return None
 with path.open() as f:head=f.read(2048)
 m=re.match(r'\s*\{\s*"passed"\s*:\s*(true|false)',head)
 if m:return m[1]=='true'
 if path.stat().st_size<2000000:return json.loads(path.read_text()).get('passed')
 return 'not_read_by_compact_inventory'
def episode(name):
 p=root/name;r=json.loads((p/'result.json').read_text());gates={n:passed(p/n) for n in ['validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','fixed_base_hazard_audit.json']};row={'episode':str(p),'original_native_success':r.get('success'),'final_native_checks':{k:v.get('passed') for k,v in r.get('final_native_destination_checks',{}).items()},'saved_report_results':gates,'videos_exist':all((p/(cam+'.mp4')).exists() for cam in ['external_camera','external_camera_2','wrist_camera'])}
 for fn in ['avoidance_audit.json','avoidance_audit_review.json','avoidance_audit.fresh_matched.json','avoidance_audit.fresh_matched.current_review.json','avoidance_audit.fresh_matched.policy_review.json']:
  if (p/fn).exists():
   d=json.loads((p/fn).read_text());row[fn]={k:d.get(k) for k in ['accepted','avoidance_effective','same_control_parameters','hazard_environment_legal']}
 return row
tasks['B01']={'status':'Task-scope evidence aggregated with explicit historical coverage limits','episodes':{v:episode(n) for v,n in {'base':'base_lgd_rgbproof_20261023T084448Z','human':'human_lgd_gatefix_20261023T113813Z','property':'property_lgd_gatefix_20261023T111200Z','robot':'robot_lgd_gatefix_20261023T112552Z'}.items()},'historical_neutral_passed':passed(Path('reports/skill_trials/lightwheel_pick_place/neutral_hold_b01_20261023T124500Z/neutral_hold.json')),'historical_paired_necessity_passed':passed(Path('reports/skill_trials/lightwheel_pick_place/paired_planning_b01_20261023T123000Z/paired_planning.json'))}
for task in ['B02','B05']:
 p=Path('/root/autodl-tmp')/(task+'-review')/'acceptance_manifest.json';m=json.loads(p.read_text());tasks[task]={'manifest':str(p),'task_scope_passed':m['passed'],'full_review_gates_passed':m['full_review_gates_passed'],'scope_exclusions':m['user_scope_exclusions'],'episodes':{k:{'episode':v['episode'],'passed':v['passed']} for k,v in m['episodes'].items()}}
tasks['B03']={'status':'All three native successes; strict matching/paired closeout pending','episodes':{v:episode(n) for v,n in {'human':'human_lgd_b03_fixedscene_20261026T092630Z','robot':'robot_lgd_b03_standard_stage_property_20261028T074115Z','property':'property_lgd_b03_standard_stage_property_20261028T082633Z'}.items()},'remaining':['strict comparable baseline/control review','strict real-controller comparison remains incomplete despite passed primary-segment paired evidence','final task manifest']}
b03close=Path('/root/autodl-tmp/B03-review/current_selected_acceptance_20261028.json')
if b03close.exists():
 m=json.loads(b03close.read_text());tasks['B03'].update(status='Current selected evidence independently aggregated',current_acceptance_manifest=str(b03close),task_scope_passed=m['passed'],remaining=m['remaining'],matching_baselines={v:x['baseline'] for v,x in m['episodes'].items()})

tasks['B04']={'status':'All three original-reset native successes and main safety audits passed; strict matched physical baseline review pending','episodes':{'human':episode('human_lgd_b04_fresh_matched_20261028T104647Z'),'property':episode('property_lgd_b04_fresh_matched_20261028T114946Z'),'robot':episode('robot_lgd_b04_fresh_matched_20261028T111828Z')},'robot_audits':'/root/autodl-tmp/lgd-day-logs/b04_robot_release36_audits.log','remaining':['strict physical baseline matching; fresh independent planning repair is authorized and in progress, no shared-prefix execution','matched baseline/avoidance/paired review','final task manifest']}
for task in ('B03','B04'):
 path=Path('/root/autodl-tmp')/(task+'-review')/'current_selected_acceptance_20261028.json'
 if path.exists():
  m=json.loads(path.read_text());tasks[task].update(status='Current selected task-scope acceptance computed from native, strict actual comparisons, neutral, full paired and video evidence',current_acceptance_manifest=str(path),task_scope_passed=m['passed'],remaining=m['remaining'],matching_baselines={v:r['baseline'] for v,r in m['episodes'].items()})
b01=tasks['B01'];b01['task_scope_passed']=bool(b01['historical_neutral_passed'] and b01['historical_paired_necessity_passed'] and all(r['original_native_success'] and r['videos_exist'] and all(r['saved_report_results'][g] is True for g in ('validation.json','full_world_audit.json','self_collision_audit.json')) for r in b01['episodes'].values()) and all(r.get('avoidance_audit.json',{}).get('accepted') is True for v,r in b01['episodes'].items() if v!='base'))
acceptance_complete=all(x.get('task_scope_passed') is True for x in tasks.values())
release_path=Path('/root/autodl-tmp/B-group-review/final_handoff_gate_20261028.json');release=json.loads(release_path.read_text()) if release_path.exists() else {}
handoff_complete=bool(release.get('passed') and release.get('implementation_commit')==subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip())
d={'as_of_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Inventory only: no historical script boolean is automatically upgraded to current full acceptance','branch':subprocess.check_output(['git','branch','--show-current'],text=True).strip(),'implementation_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'task_scope_acceptance_complete':acceptance_complete,'delivery_complete':bool(acceptance_complete and handoff_complete),'full_unexcluded_review_claimed':False,'push_status':'Not pushed; preparing lgd branch evidence and verified bundle for user GitHub handoff','tasks':tasks};out=Path('/root/autodl-tmp/B-group-review');out.mkdir(exist_ok=True);(out/'current_delivery_inventory.json').write_text(json.dumps(d,indent=2));print('INVENTORY',out/'current_delivery_inventory.json',flush=True)
