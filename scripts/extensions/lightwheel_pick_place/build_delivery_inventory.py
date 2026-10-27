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
 for fn in ['avoidance_audit.json','avoidance_audit_review.json']:
  if (p/fn).exists():
   d=json.loads((p/fn).read_text());row[fn]={k:d.get(k) for k in ['accepted','avoidance_effective','same_control_parameters','hazard_environment_legal']}
 return row
tasks['B01']={'status':'Historical physical data and review records, final aggregation pending','episodes':{v:episode(n) for v,n in {'base':'base_lgd_rgbproof_20261023T084448Z','human':'human_lgd_gatefix_20261023T113813Z','property':'property_lgd_gatefix_20261023T111200Z','robot':'robot_lgd_gatefix_20261023T112552Z'}.items()},'historical_neutral_passed':passed(Path('reports/skill_trials/lightwheel_pick_place/neutral_hold_b01_20261023T124500Z/neutral_hold.json')),'historical_paired_necessity_passed':passed(Path('reports/skill_trials/lightwheel_pick_place/paired_planning_b01_20261023T123000Z/paired_planning.json'))}
for task in ['B02','B05']:
 p=Path('/root/autodl-tmp')/(task+'-review')/'acceptance_manifest.json';m=json.loads(p.read_text());tasks[task]={'manifest':str(p),'task_scope_passed':m['passed'],'full_review_gates_passed':m['full_review_gates_passed'],'scope_exclusions':m['user_scope_exclusions'],'episodes':{k:{'episode':v['episode'],'passed':v['passed']} for k,v in m['episodes'].items()}}
tasks['B03']={'status':'All three native successes; strict matching/paired closeout pending','episodes':{v:episode(n) for v,n in {'human':'human_lgd_b03_fixedscene_20261026T092630Z','robot':'robot_lgd_b03_checkpoint_replay_20261027T033440Z','property':'property_lgd_b03_checkpoint_replay_20261027T064923Z'}.items()},'remaining':['strict comparable baseline/control review','current-policy paired planning42/43/44','final task manifest']}
tasks['B04']={'status':'All three original-reset native successes; robot36 final audits and matched evidence pending','episodes':{'human':episode('human_lgd_b04_round09_20261027T081735Z'),'property':episode('property_lgd_b04_round16_20261027T095059Z'),'robot':episode('robot_lgd_b04_round36_20261027T152820Z')},'robot_audits':'/root/autodl-tmp/lgd-day-logs/b04_robot_release36_audits.log','remaining':['robot remaining safety review','matched baseline/avoidance/paired review','final task manifest']}
d={'as_of_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Inventory only: no historical script boolean is automatically upgraded to current full acceptance','branch':subprocess.check_output(['git','branch','--show-current'],text=True).strip(),'implementation_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'delivery_complete':False,'push_status':'Not pushed; GitHub authentication unavailable, commits/bundles preserved','tasks':tasks};out=Path('/root/autodl-tmp/B-group-review');out.mkdir(exist_ok=True);(out/'current_delivery_inventory.json').write_text(json.dumps(d,indent=2));print('INVENTORY',out/'current_delivery_inventory.json',flush=True)
