"""Run one released variant with a verified matching successful baseline, serially."""
import argparse,hashlib,json,math,subprocess,time,traceback
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--variant',required=True);p.add_argument('--tag',required=True);a=p.parse_args()
M=json.loads(a.manifest.read_text());R=a.manifest.parent;T=M['label'];V=a.variant;BASE=Path(M['baseline']);C=json.loads((BASE/'metadata.json').read_text());POL=C['controller_parameters'];STATE=R/(T+'_'+V+'_'+a.tag+'_state.json');assert not STATE.exists();state={'stages':[],'status':'started','manifest':str(a.manifest),'baseline':str(BASE)}
def save():STATE.write_text(json.dumps(state,indent=2))
def run(label,script,args,gpu=False,host=False):
 log=R/(label+'.log');assert not log.exists();cmd=(['python3','-u',script] if host else ['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u',script])+list(map(str,args));stage={'label':label,'command':cmd,'started':time.time()};state['stages'].append(stage);state['current']=label;save()
 with log.open('x') as f:stage['returncode']=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 stage['finished']=time.time();save();return stage['returncode']
def collect(tag,neutral=False,seconds=40.):
 ep=Path('datasets/doors/lwx_scene_approved_20261026')/(T+'_'+V+'_'+tag);args=['--task-id',M['task_id'],'--variant',V,'--variant-config',M['base_config'],'--scene-manifest',a.manifest,'--observer-config',R/(T+'_observer01.json'),'--seed',POL.get('seed',42),'--validated-pregrasp','--render-warmup','--max-steps',1800,'--output',ep]
 keys={'approach_gripper_position':'approach-gripper-position','approach_preshape_steps':'approach-preshape-steps','gripper_command_policy':'gripper-command-policy','release_open_steps':'release-open-steps','grasp_alignment_start_roll_deg':'grasp-alignment-start-roll-deg','approach_alignment_end_fraction':'approach-alignment-end-fraction','pregrasp_roll_deg':'pregrasp-roll-deg','pregrasp_pitch_deg':'pregrasp-pitch-deg','pregrasp_distance_m':'pregrasp-distance-m','handle_offset_y':'handle-offset-y','handle_offset_z':'handle-offset-z','grasp_roll_deg':'grasp-roll-deg','grasp_pitch_deg':'grasp-pitch-deg','handle_normal_offset_m':'handle-normal-offset','pull_lead_rad':'pull-lead-rad','pull_steps':'pull-steps','ik_limit_margin_rad':'ik-limit-margin-rad','release_retreat_m':'release-retreat-m','release_tilt_deg':'release-tilt-deg','release_yaw_deg':'release-yaw-deg','release_roll_delta_deg':'release-roll-delta-deg','release_completion_fraction':'release-completion-fraction','release_retreat_profile':'release-retreat-profile','release_settle_steps':'release-settle-steps','opening_reserve_rad':'opening-reserve-rad','pregrasp_settle_steps':'pregrasp-settle-steps','pregrasp_shortcut_policy':'pregrasp-shortcut-policy','planner_hazard_extra_padding_m':'planner-hazard-extra-padding-m','release_command_policy':'release-command-policy'}
 for k,v in keys.items():
  if k in POL:args+=['--'+v,POL[k]]
 if POL.get('pregrasp_goal_q'):args+=['--pregrasp-goal-q',*POL['pregrasp_goal_q']]
 if POL.get('pull_command_policy'):args+=['--pull-command-policy','lead_backoff_v1']
 if neutral:args+=['--mode','neutral','--neutral-seconds',seconds]
 run(ep.name,'scripts/extensions/doors/collect_scene_approved_lwx.py',args,gpu=True);res=json.loads((ep/'result.json').read_text()) if (ep/'result.json').exists() else {'accepted':False,'failure':'missing result'};return ep,res
try:
 assert json.loads((BASE/'result.json').read_text())['accepted']
 assert max(abs(a-b) for a,b in zip(M.get('preview_initial_q',C['initial_q']),C['initial_q']))<1e-5
 assert max(abs(a-b) for a,b in zip(M['new_handle_world_m'],C['handle_initial']))<.004
 assert M['task_id']==C['task']['task_id'] if isinstance(C['task'],dict) else M['task_id']==C['task']
 if M.get('proposed_robot_base_world_m'):assert max(abs(a-b) for a,b in zip(M['proposed_robot_base_world_m'],C['robot_base']))<1e-5
 strict=json.loads((R/(T+'_strict_static05.json')).read_text());assert strict['complete'] and strict['manifest_sha256']==hashlib.sha256(a.manifest.read_bytes()).hexdigest();row=next(x for x in strict['candidates'] if x['variant']==V);assert row['static_rejections_absent'] and row['config_sha256']==hashlib.sha256(Path(M['base_config']).read_bytes()).hexdigest()
 paired=R/(T+'_'+V+'_'+a.tag+'_pairs');run(paired.name,'scripts/extensions/doors/paired_tasks_lwx.py',['--baseline',BASE,'--variant',V,'--config',M['base_config'],'--output',paired],gpu=True);pair=json.loads((paired/'report.json').read_text());state['all_seed_necessity']=pair['fixed_endpoint_rerouting_supported'];save()
 if not state['all_seed_necessity']:raise RuntimeError('All seed fixed-target planning not passed')
 screen=R/(T+'_'+V+'_'+a.tag+'_preflight.json');screen_args=['--metadata',BASE/'metadata.json','--config',M['base_config'],'--variant',V,'--output',screen,'--reference-episode',BASE]
 if POL.get('gripper_command_policy')=='continuous_position_v1':screen_args+=['--postgrasp-reference-replay']
 code=run(screen.stem,'scripts/extensions/doors/screen_approved_scene_lwx.py',screen_args,gpu=True)
 if code:raise RuntimeError('Source/control preflight failed')
 ep,res=collect(a.tag);state['episode']=str(ep)
 if not res.get('accepted'):raise RuntimeError('Collection failed: '+str(res.get('failure')))
 rows=json.loads((ep/'trajectory.json').read_text());neutral,nres=collect(a.tag+'_neutral',True,max(40.,len(rows)*C['dt']+1.));state['neutral']=str(neutral)
 if not nres.get('neutral_completed'):raise RuntimeError('Neutral failed')
 run(ep.name+'_reviews','scripts/extensions/doors/supervise_task_reviews_lwx.py',[ep,'--neutral',neutral,'--report-root',R,'--run-label',a.tag],host=True)
 gates={n:json.loads((ep/n).read_text()).get(k,False) for n,k in [('task_review_audit.json','passed'),('task_validation_full.json','accepted'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]};state['independent_gates']=gates
 if not all(gates.values()):raise RuntimeError('Independent gates failed')
 run(ep.name+'_avoidance','scripts/extensions/doors/audit_task_avoidance_lwx.py',[ep,'--baseline',BASE]);av=json.loads((ep/'avoidance_audit.json').read_text());state['avoidance_accepted']=av['accepted'];state['path_comparison']=av['path_comparison'];run(ep.name+'_fourcam','scripts/extensions/doors/make_four_camera_review_lwx.py',[ep]);state['status']='independent_pass_visual_pending' if av['accepted'] else 'avoidance_failed'
except Exception as e:state['status']='rejected';state['failure']=repr(e);traceback.print_exc()
finally:state['finished']=time.time();save();print(json.dumps(state),flush=True)
