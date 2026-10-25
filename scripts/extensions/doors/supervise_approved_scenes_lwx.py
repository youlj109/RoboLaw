"""One-shot serial approved-scene run; preserve originals and failed evidence."""
import fcntl,json,math,os,subprocess,time,traceback
from pathlib import Path
R=Path('reports/doors/lwx_approved_run_20261026');D=Path('datasets/doors/lwx_scene_approved_20261026');D.mkdir(exist_ok=True,parents=True)
lock=(R/'supervisor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
statepath=R/'overnight_state01.json'
if statepath.exists():raise FileExistsError('Do not duplicate a supervisor run')
state={'pid':os.getpid(),'started_utc':time.time(),'complete':False,'current':None,'tasks':{},'stages':[],'scope':'One-shot serial work, no periodic automation, no sample batch. Collector success is not independent or visual acceptance.'}
def save():
 tmp=statepath.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(statepath)
def run(label,script,args,gpu=False,host=False):
 state['current']=label;save();print('START',label,flush=True);logfile=R/(label+'.log')
 if logfile.exists():raise FileExistsError(logfile)
 if host:cmd=['python3','-u',script,*map(str,args)]
 else:cmd=['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u',script,*map(str,args)]
 entry={'label':label,'command':cmd,'log':str(logfile),'started_utc':time.time()};state['stages'].append(entry);save()
 with logfile.open('x') as f:code=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 entry.update(exit_code=code,finished_utc=time.time());save();print('FINISH',label,code,flush=True);return code
KEYS={'handle_offset_y':'handle-offset-y','handle_offset_z':'handle-offset-z','grasp_roll_deg':'grasp-roll-deg','grasp_pitch_deg':'grasp-pitch-deg','handle_normal_offset_m':'handle-normal-offset','pull_lead_rad':'pull-lead-rad','pull_steps':'pull-steps','ik_limit_margin_rad':'ik-limit-margin-rad','release_retreat_m':'release-retreat-m','release_tilt_deg':'release-tilt-deg','release_command_policy':'release-command-policy','release_retreat_profile':'release-retreat-profile','release_settle_steps':'release-settle-steps','opening_reserve_rad':'opening-reserve-rad','pregrasp_settle_steps':'pregrasp-settle-steps','pregrasp_shortcut_policy':'pregrasp-shortcut-policy','planner_hazard_extra_padding_m':'planner-hazard-extra-padding-m'}
def parameters(m):
 old=json.loads((Path(m['baseline'])/'metadata.json').read_text());c=dict(old['controller_parameters']);delta=[b-a for a,b in zip(m['old_handle_world_m'],m['new_handle_world_m'])];yaw=math.radians(old['robot_yaw_deg']);approach=[math.cos(yaw),math.sin(yaw)];side=[-approach[1],approach[0]]
 c['handle_offset_z']+=delta[2];c['handle_offset_y']=c.get('handle_offset_y',0.)+sum(delta[i]*side[i] for i in range(2));c['handle_normal_offset_m']=c.get('handle_normal_offset_m',0.)+sum(delta[i]*approach[i] for i in range(2));return old,c
def collect(t,m,c,variant,tag,neutral=False,seconds=40.):
 ep=D/(t+'_'+variant+'_'+tag+('_neutral' if neutral else ''));args=['--task-id',m['task_id'],'--variant',variant,'--variant-config',m['base_config'],'--scene-manifest',str(R/(t+'_released01.json')),'--observer-config',str(R/(t+'_observer01.json')),'--seed','42','--validated-pregrasp','--render-warmup','--max-steps','1800','--output',str(ep)]
 for k,v in KEYS.items():
  if k in c:args+=['--'+v,str(c[k])]
 if c.get('pregrasp_goal_q'):args+=['--pregrasp-goal-q',*map(str,c['pregrasp_goal_q'])]
 if c.get('pull_command_policy'):args+=['--pull-command-policy','lead_backoff_v1']
 if neutral:args+=['--mode','neutral','--neutral-seconds',str(seconds)]
 code=run(ep.name,'scripts/extensions/doors/collect_scene_approved_lwx.py',args,gpu=True)
 result=json.loads((ep/'result.json').read_text()) if (ep/'result.json').exists() else {'accepted':False,'failure':'No result; inspect '+str(R/(ep.name+'.log'))}
 return ep,result,code
def screen(t,m,c,variant,tag,metadata=None):
 if metadata is None:
  old,_=parameters(m);old['initial_q']=m.get('preview_initial_q',old['initial_q']);old['robot_base']=m.get('proposed_robot_base_world_m',old['robot_base']);old['handle_initial']=m['new_handle_world_m'];old['controller_parameters']=c;mp=R/(t+'_'+tag+'_preflight_metadata.json');mp.write_text(json.dumps(old,indent=2))
 else:mp=metadata
 out=R/(t+'_'+variant+'_'+tag+'_preflight.json');code=run(out.stem,'scripts/extensions/doors/screen_approved_scene_lwx.py',['--metadata',mp,'--config',m['base_config'],'--variant',variant,'--output',out,'--reference-episode',m['baseline']],gpu=True)
 return code==0 and out.exists() and json.loads(out.read_text()).get('accepted_geometry',False)
def review(t,ep,neutral,tag):
 code=run(t+'_'+tag+'_reviews','scripts/extensions/doors/supervise_task_reviews_lwx.py',[ep,'--neutral',neutral,'--report-root',R,'--run-label',tag],host=True)
 outputs=[('task_review_audit.json','passed'),('task_validation_full.json','accepted'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
 gates={n:bool(json.loads((ep/n).read_text()).get(k,False)) if (ep/n).exists() else False for n,k in outputs};run(t+'_'+tag+'_fourcam','scripts/extensions/doors/make_four_camera_review_lwx.py',[ep]);return code==0 and all(gates.values()),gates
def process(t):
 m=json.loads((R/(t+'_released01.json')).read_text());static=json.loads((R/(t+'_released_static01.json')).read_text());task=state['tasks'][t]={'variants':{},'static_passed':bool(static['complete'] and all(x['static_rejections_absent'] for x in static['candidates']))};save()
 if not task['static_passed']:task['status']='static_rejected';return
 old,c=parameters(m);base=None
 if t=='C06':
  known=D/'C06_base02';res=json.loads((known/'result.json').read_text())
  if res.get('accepted'):base=known;c=json.loads((base/'metadata.json').read_text())['controller_parameters'];task['baseline_reused']=str(base)
 if base is None:
  if not screen(t,m,c,'base','p01'):task['status']='base_geometry_rejected';return
  base,res,code=collect(t,m,c,'base','p01')
  if not res.get('accepted'):
   failure=res.get('failure') or '';task['baseline_failure']=failure
   if 'release' in failure.lower():
    c['release_retreat_m']=.04;c['release_retreat_profile']='ease_out';c['opening_reserve_rad']=.04;c['release_settle_steps']=20;base,res,code=collect(t,m,c,'base','p02')
   if not res.get('accepted'):task['status']='base_physical_rejected';return
 task['baseline']=str(base);task['controller_parameters']=c;save()
 rows=json.loads((base/'trajectory.json').read_text());duration=max(40.,len(rows)*old['dt']+1.);neutral,nres,_=collect(t,m,c,'base','n01',neutral=True,seconds=duration);task['baseline_neutral']=str(neutral)
 if not nres.get('neutral_completed'):task['status']='base_neutral_rejected';return
 baseok,basegates=review(t,base,neutral,'base01');task['baseline_independent_gates']=basegates
 if not baseok:task['status']='base_independent_rejected';save();return
 for item in m['candidates']:
  v=item['variant'];info=task['variants'][v]={};paired=R/(t+'_'+v+'_pairs01');info['paired_directory']=str(paired);save()
  run(t+'_'+v+'_pairs01','scripts/extensions/doors/paired_tasks_lwx.py',['--baseline',base,'--variant',v,'--config',m['base_config'],'--output',paired],gpu=True)
  pf=paired/'report.json'
  if not pf.exists():info['status']='paired_report_missing';save();continue
  pairs=json.loads(pf.read_text());info['all_seed_fixed_target_necessity']=pairs.get('fixed_endpoint_rerouting_supported',False);seed42=next((x for x in pairs['pairs'] if x['seed']==42),None)
  if not seed42 or not seed42.get('policy_necessity_supported'):info['status']='representative_seed_necessity_not_established';save();continue
  if not screen(t,m,c,v,'p01',base/'metadata.json'):info['status']='variant_geometry_rejected';save();continue
  ep,eres,_=collect(t,m,c,v,'p01');info['episode']=str(ep)
  if not eres.get('accepted'):info['status']='variant_physical_rejected';info['failure']=eres.get('failure');save();continue
  nr=json.loads((ep/'trajectory.json').read_text());seconds=max(40.,len(nr)*old['dt']+1.);ne,nres,_=collect(t,m,c,v,'n01',neutral=True,seconds=seconds);info['neutral']=str(ne)
  if not nres.get('neutral_completed'):info['status']='variant_neutral_rejected';save();continue
  ok,gates=review(t,ep,ne,v+'01');info['independent_gates']=gates
  if ok:
   run(t+'_'+v+'_avoidance01','scripts/extensions/doors/audit_task_avoidance_lwx.py',[ep,'--baseline',base]);ap=ep/'avoidance_audit.json';info['actual_avoidance_accepted']=bool(ap.exists() and json.loads(ap.read_text()).get('accepted',False))
   if ap.exists():
    av=json.loads(ap.read_text());info['actual_tcp_max_deviation_m']=av['path_comparison']['tcp_max_deviation_m'];info['baseline_free_pregrasp_blocked']=av['baseline_counterfactual']['violates_free_pregrasp']
  info['visual_review']='pending agent inspection of actual video';info['status']='independent_pass_visual_pending' if ok and info.get('actual_avoidance_accepted') and info['all_seed_fixed_target_necessity'] else 'partial_evidence_unresolved';save()
 task['status']='pass_pending_video_review' if all(v['status']=='independent_pass_visual_pending' for v in task['variants'].values()) and task['variants'] else 'needs_revision_or_review';save()
try:
 save()
 for t in ['C06','C02','C03','C04','C05','C08','C01']:
  try:process(t)
  except Exception as e:state['tasks'].setdefault(t,{})['supervisor_error']=repr(e);traceback.print_exc();save()
 state['current']=None;state['complete']=True;state['finished_utc']=time.time();save();print('ONE_SHOT_PIPELINE_FINISHED',flush=True)
except BaseException as e:state['fatal_error']=repr(e);save();raise
