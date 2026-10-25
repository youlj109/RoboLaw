"""Bind independent current task evidence; fail closed on missing or failed gates."""
import argparse,hashlib,json,subprocess,platform,sys
from importlib.metadata import version,PackageNotFoundError
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--task',required=True);p.add_argument('--task-scope',action='store_true',help='Explicit user scope: exclude unrelated global kitchen-joint neutral gate, retain task/object/robot checks and record exclusion');p.add_argument('--baseline',type=Path,required=True);p.add_argument('--human',type=Path,required=True);p.add_argument('--property',type=Path,required=True);p.add_argument('--robot',type=Path,required=True);p.add_argument('--neutral',type=Path,required=True);p.add_argument('--paired',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
def read(path):return json.loads(path.read_text()) if path.exists() else {}
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for block in iter(lambda:stream.read(8*1024*1024),b''):h.update(block)
 return h.hexdigest()
paths={k:getattr(a,k) for k in ('baseline','human','property','robot')};episodes={};hashes={};max_duration=0
for name,ep in paths.items():
 r=read(ep/'result.json');max_duration=max(max_duration,float(r.get('frames',0))*float(r.get('dt',0)));gates={'physical_success':r.get('success') is True,'task_and_variant_match':r.get('task')==a.task and r.get('variant')==('base' if name=='baseline' else name)}
 for file in ('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json'):
  gates[file]=read(ep/file).get('passed') is True
 if name!='baseline':
  for file in ('fixed_base_hazard_audit.json','source_background_audit.json'):gates[file]=read(ep/file).get('passed') is True
  gates['effective_avoidance_acceptance']=read(ep/'avoidance_audit.json').get('accepted') is True
 required=['external_camera.mp4','external_camera_2.mp4','wrist_camera.mp4']+(['overview_camera.mp4'] if r.get('overview_camera') else [])
 gates['required_videos_exist']=all((ep/x).is_file() and (ep/x).stat().st_size>0 for x in required)
 episodes[name]={'collector_source_sha256':r.get('collector_source_sha256'),'episode':str(ep),'gates':gates,'passed':all(gates.values()),'duration_seconds':float(r.get('frames',0))*float(r.get('dt',0))}
 for file in ['result.json','trajectory.h5','images.h5','contacts.json','clearances.json','scene.usda','initial_setup.json','validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','avoidance_audit.json','fixed_base_hazard_audit.json','source_background_audit.json']+required:
  if (ep/file).is_file():hashes[str(ep/file)]=digest(ep/file)
 for extra in ep.glob('contacts_*.json'):hashes[str(extra)]=digest(extra)
paired_reports=[read(x) for x in a.paired];paired={}
for v in ('human','property','robot'):
 metadata=read(paths[v]/'result.json')
 rows=[]
 for r in paired_reports:
  bound=(r.get('seeds')==[42,43,44] and r.get('baseline_result_sha256')==digest(a.baseline/'result.json') and r.get('baseline_trajectory_sha256')==digest(a.baseline/'trajectory.h5') and r.get('variant_config_sha256')==metadata.get('variant_config_source',{}).get('sha256'))
  if not bound:continue
  summary=r.get('variant_reports',{}).get(v,{})
  segment=summary.get('primary_segment')
  on=[x for x in r.get('runs',[]) if x.get('variant')==v and x.get('segment')==segment]
  off=[x for x in r.get('runs',[]) if x.get('variant')=='base' and x.get('segment')==segment]
  verified=(len(on)==3 and len(off)==3 and {x['seed'] for x in on}=={42,43,44} and {x['seed'] for x in off}=={42,43,44} and all(x.get('success') and x.get('validation',{}).get('valid') for x in on+off) and all(x.get('true_shape_counterfactual',{}).get('samples',0)>0 and x.get('true_shape_counterfactual',{}).get('violations') and x.get('true_shape_actual',{}).get('samples',0)>0 and x.get('true_shape_actual',{}).get('minimum',{}).get('valid') is True and x.get('true_shape_actual',{}).get('violations')==[] for x in on))
  if verified:rows.append(summary)
 paired[v]={'stable_necessity':any(x.get('stable_avoidance_necessity_in_tested_seeds') is True for x in rows),'all_hazard_on_clear':any(x.get('all_hazard_on_plans_clear') is True for x in rows)}
neutral=read(a.neutral);baseline_metadata=read(a.baseline/'result.json')
neutral_task_gate=(neutral.get('task_objects_passed') is True and neutral.get('distractors_passed') is True and neutral.get('robot_hold_passed') is True and neutral.get('camera_capture_alignment') is True and float(neutral.get('duration_seconds',0))>=max_duration and neutral.get('task')==baseline_metadata.get('task') and neutral.get('fixed_layout_sha256')==baseline_metadata.get('fixed_layout_source',{}).get('sha256'))
neutral_task_gate=bool(neutral_task_gate and all((neutral.get('initial_state_layout_source') or {}).get(k)==baseline_metadata.get('initial_state_layout_source',{}).get(k) for k in ('path','sha256')))
neutral_full_gate=neutral_task_gate and neutral.get('passed') is True and neutral.get('unoperated_joint_state_passed') is True
neutral_gate=neutral_task_gate if a.task_scope else neutral_full_gate

for path in [a.neutral,*a.paired]:
 if path.exists():hashes[str(path)]=digest(path)
passed=all(x['passed'] for x in episodes.values()) and neutral_gate and all(x['stable_necessity'] and x['all_hazard_on_clear'] for x in paired.values())
implementation_files=[Path('scripts/extensions/lightwheel_pick_place')/x for x in ('collect.py','validate.py','audit_avoidance.py','audit_source_support.py','audit_source_background.py','audit_fixed_base.py','audit_self_collision.py','paired_planning.py','task_neutral_hold.py','run_host.sh','task_manifest.py','audit_deferred_world.py','audit_multi_source.py')]
implementation_files += [Path('kitchen_safety/extensions/lightwheel_pick_place')/x for x in ('collision.py','planner.py','lemon_kinematics.py','source_support.py','task_config.py','joint_state.py','b03_strategy.py','contacts.py','multi_evidence.py')]
implementation_files += [Path('kitchen_safety/planning.py'),Path('kitchen_safety/curobo_backend.py')]
runtime={}
for package in ('numpy','scipy','torch','h5py','python-fcl','usd-core','curobo','isaacsim','isaaclab','isaaclab_arena','imageio-ffmpeg'):
 try:runtime[package]=version(package)
 except PackageNotFoundError:runtime[package]=None
report={'acceptance_scope':'user-requested task scope' if a.task_scope else 'full review gates','full_review_gates_passed':bool(passed and neutral_full_gate),'user_scope_exclusions':(['Global unoperated kitchen-fixture joint neutral quality: excluded by explicit user instruction; not inspected or claimed passed'] if a.task_scope else []),'python':platform.python_version(),'python_executable':sys.executable,'runtime_versions':runtime,'implementation_git_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'implementation_sha256':{str(x):digest(x) for x in implementation_files},'passed':passed,'task':a.task,'branch':'lgd','fixed_only':True,'episodes':episodes,'paired':paired,'neutral':{'path':str(a.neutral),'passed_and_covers_longest_episode':neutral_gate,'duration_seconds':neutral.get('duration_seconds'),'required_seconds':max_duration},'sha256':hashes,'scope':'Three planning seeds cover the reported transfer segment, not a full-task success-rate estimate. Physical evidence covers one complete successful episode per variant. Authored source collision geometry is checked; PhysX cooked-shape equivalence is not independently certified.'}
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='sha256'},indent=2));raise SystemExit(0 if passed else 2)
