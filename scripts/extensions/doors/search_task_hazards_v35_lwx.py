"""Offline source search with calibrated finite candidate pull/approach constraints; no changed safety gate or unsafe execution."""
import argparse,json,itertools,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,fcl
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance,separation
from kitchen_safety.extensions.doors.obstruction_broadphase_lwx import potential_intersection_rows
from kitchen_safety.extensions.doors.source_sphere_screen_lwx import source_spheres_safe
from kitchen_safety.extensions.doors.invariant_envelopes_lwx import invariant_envelopes
p=argparse.ArgumentParser();p.add_argument('--baseline',required=True,type=Path);p.add_argument('--config',required=True,type=Path);p.add_argument('--variant',required=True);p.add_argument('--output',required=True,type=Path);p.add_argument('--pedestal',action='store_true');p.add_argument('--heights');p.add_argument('--min-blocking-samples',type=int,default=3);p.add_argument('--mid-stride',type=int,default=4);p.add_argument('--human-asset');p.add_argument('--refine-from',type=Path);p.add_argument('--candidate-list',type=Path);p.add_argument('--paired-reference',type=Path);p.add_argument('--hazard-asset');p.add_argument('--support-width-m',type=float);p.add_argument('--yaws');p.add_argument('--xy-step-m',type=float,default=.04);p.add_argument('--screen-approach-proxies',action='store_true');p.add_argument('--support-offset-xy',nargs=2,type=float,default=[0.,0.]);p.add_argument('--approach-reference',type=Path,action='append',default=[]);p.add_argument('--paired-endpoint-reference',type=Path,action='append',default=[]);p.add_argument('--paired-endpoint-extra-clearance-m',type=float,default=.005);p.add_argument('--allow-endpoint-posture-adaptation',action='store_true');p.add_argument('--approach-command-reference',type=Path,action='append',default=[]);p.add_argument('--screen-invariant-envelopes',action='store_true');p.add_argument('--measured-contact-reference',type=Path,action='append',default=[]);p.add_argument('--command-sphere-reference',type=Path,action='append',default=[]);p.add_argument('--screen-baseline-contact-proxies',action='store_true');p.add_argument('--candidate-approach-reference',type=Path,action='append',default=[]);p.add_argument('--candidate-pull-reference',type=Path,action='append',default=[]);p.add_argument('--endpoint-proposals-output',type=Path);a=p.parse_args()
if not .01<=a.xy_step_m<=.08:p.error('Discovery xy step must be between .01 and .08m')
if np.linalg.norm(a.support_offset_xy)>.4:p.error('Support offset must be at most .4m')
if a.output.exists():p.error('Evidence exists')
if a.endpoint_proposals_output and a.endpoint_proposals_output.exists():p.error('Endpoint proposal evidence exists')
if a.command_sphere_reference and a.variant not in ['property','robot']:p.error('Command sphere/AABB discovery references support property or robot only')
if a.screen_baseline_contact_proxies and a.variant not in ['property','robot']:p.error('Baseline contact proxy discovery supports property or robot only')
if a.allow_endpoint_posture_adaptation and (a.screen_approach_proxies or a.approach_reference or a.paired_endpoint_reference or a.approach_command_reference):p.error('Posture-adaptation discovery cannot simultaneously constrain old approach or endpoint joint states')
if not 0.<=a.paired_endpoint_extra_clearance_m<=.03:p.error('Paired endpoint discovery extra clearance outside [0,.03]m')
m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text());config=json.loads(a.config.read_text());old=json.loads(Path('configs/extensions/doors/front_variants_lwx_final.json').read_text());spec=old[a.variant].copy();spec.pop('support',None);spec['position']=[0.,0.,0.];spec['yaw_deg']=0.;spec.pop('candidate_evidence',None)
if a.variant=='human':spec['asset']='assets/extensions/doors/human_reach_elbow_up/collidable.usda';spec.pop('baked_yaw_deg',None)
if a.human_asset:spec['asset']=a.human_asset
if a.hazard_asset:spec['asset']=a.hazard_asset;spec['semantic_anchor']='Unscaled original library asset; placement searched against actual source meshes and all reference off paths. Physical stability and same-target small rerouting still require independent acceptance.'
stage,kin,w,g=reconstruct(m,config,spec);key=spec['name'];fixed=[o for o in w.objects if not w._matches(o[0],key)];haz0=physical_hazards(stage,w,key)
print('HAZARD_BOUNDS',a.variant,[(n,lo.tolist(),hi.tolist()) for n,_,lo,hi in haz0],flush=True)
robot=[]
for row in rows:
 q=np.asarray(row['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};robot.append(physical_robot_shapes(w.shapes(q[:7],f)))
pre=[i for i,r in enumerate(rows) if r['phase']=='pregrasp'];mid=pre[5:-5:a.mid_stride];late=list(range(pre[-1],len(rows),6))+[len(rows)-1];margin=w.clearances[key];execmargin=w.execution_clearances[key]
mid_bounds=np.asarray([[[lo,hi] for _,_,lo,hi in robot[i]] for i in mid])
base=np.asarray(m['robot_base']);xs=np.arange(base[0]-.8,base[0]+.81,.08);ys=np.arange(base[1]-.8,base[1]+.81,.08)
if a.variant=='human':candidates=itertools.product(np.arange(base[0]-1.2,base[0]+1.21,.06),np.arange(base[1]-1.2,base[1]+1.21,.06),[0.],range(0,360,15))
else:
 heights=[float(base[2]-.01),float(base[2]+.07),float(base[2]+.15),float(base[2]+.23)] if a.pedestal else [0.,float(base[2]-.01)]
 if a.heights:heights=list(map(float,a.heights.split(',')))
 xs=np.arange(base[0]-.4,base[0]+.57,a.xy_step_m);ys=np.arange(base[1]-.4,base[1]+.65,a.xy_step_m)
 candidates=itertools.product(xs,ys,heights,list(map(float,a.yaws.split(','))) if a.yaws else ([0.] if a.variant=='property' else range(0,360,45)))
if a.refine_from:
 prior=json.loads(a.refine_from.read_text())['candidates'];candidates=[(*[float(c['position'][j]+d[j]) for j in range(3)],c['yaw_deg']) for c in prior for d in itertools.product([-.03,-.02,-.01,0,.01,.02,.03],[ -.03,-.02,-.01,0,.01,.02,.03],[-.01,0,.01])]
if a.candidate_list:candidates=json.loads(a.candidate_list.read_text())
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
goal_metadata=dict(m);goal_metadata['initial_q']=rows[pre[-1]]['next_obs']['q'];goal_spheres=initial_spheres(goal_metadata,kin)
# Same Cartesian target fixes the distal gripper transform while redundant elbows can move.
# Derive descendants from the actual URDF rather than guessing link-name prefixes.
invariant_links={'panda_panda_link7'}
for _ in range(100):
 old_count=len(invariant_links)
 for joint in ET.parse('assets/curobo/droid/urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint'):
  if joint.find('parent').get('link') in invariant_links:invariant_links.add(joint.find('child').get('link'))
 if len(invariant_links)==old_count:break
invariant_goal_spheres=[sphere for sphere in goal_spheres if sphere[0] in invariant_links]
if not invariant_goal_spheres:raise RuntimeError('Missing distal goal sphere model')
spheres=initial_spheres(m,kin)+(invariant_goal_spheres if a.allow_endpoint_posture_adaptation else goal_spheres)
hand_states=[[shape for shape in robot[i] if shape[0].startswith('gripper/')] for i in late]
if any(not shapes for shapes in hand_states):raise RuntimeError('Missing articulated source gripper')
# Discovery-only conservative constraints. Rigid distal envelopes follow the
# fixed Cartesian policy; the rest of the redundant arm remains unconstrained.
invariant_states=[];initial_envelopes=[]
if a.screen_invariant_envelopes:
 for i in [0,*late]:
  q=np.asarray(rows[i]['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
  full_shapes=w.shapes(q[:7],f)
  if i==0:initial_envelopes=full_shapes
  else:invariant_states.append(invariant_envelopes(full_shapes))

if a.screen_approach_proxies:
 for row in rows:
  if row['phase'] not in ['approach_handle','approach']:continue
  approach_metadata=dict(m);approach_metadata['initial_q']=row['next_obs']['q'];spheres+=initial_spheres(approach_metadata,kin)
print('SCREENING_SPHERES',len(spheres),'approach_proxies',a.screen_approach_proxies,flush=True)
sphere_centers=np.asarray([s[2] for s in spheres]);sphere_radii=np.asarray([s[3] for s in spheres])
def initial_safe(bounds,margin):
 for lo,hi in bounds:
  d=np.linalg.norm(np.maximum(np.maximum(lo-margin-sphere_centers,sphere_centers-hi-margin),0),axis=1)-sphere_radii
  if np.min(d)<.003:return False
 return True
approach_reference_shapes=[]
approach_reference_provenance=[]
for reference in a.approach_reference:
 reference_metadata=json.loads((reference/'metadata.json').read_text())
 if reference_metadata['variant_spec']['asset']!=spec['asset']:raise RuntimeError('Approach reference uses a different hazard source asset')
 if reference_metadata['robot_base']!=m['robot_base'] or reference_metadata['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Reference policy/layout mismatch')
 reference_rows=json.loads((reference/'trajectory.json').read_text())
 joints=[r['next_obs']['q'] for r in reference_rows if r['phase'] in ['approach_handle','approach']]
 result=json.loads((reference/'result.json').read_text());failure=result.get('failure') or ''
 if 'EXECUTION_COLLISION ' in failure:joints.append(json.loads(failure.split('EXECUTION_COLLISION ',1)[1])['q'])
 if not joints:raise RuntimeError('No actual approach/reference command states')
 for values in joints:
  q=np.asarray(values);finger={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
  approach_reference_shapes.append(w.shapes(q[:7],finger))
 approach_reference_provenance.append({'episode':str(reference),'states':len(joints),'result_sha256':hashlib.sha256((reference/'result.json').read_bytes()).hexdigest(),'trajectory_sha256':hashlib.sha256((reference/'trajectory.json').read_bytes()).hexdigest()})
for reference in a.approach_command_reference+a.candidate_approach_reference:
 diagnostic=json.loads(reference.read_text())
 if diagnostic['baseline_sha256']!=hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest() or diagnostic['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Predicted approach reference policy mismatch')
 diagnostic_config=Path(diagnostic['config'])
 if hashlib.sha256(diagnostic_config.read_bytes()).hexdigest()!=diagnostic['config_sha256']:raise RuntimeError('Predicted approach reference config changed')
 reference_hazard_asset=json.loads(diagnostic_config.read_text())[a.variant]['asset']
 count=0
 for pair in diagnostic['pairs']:
  for sample in pair.get('samples',[]):
   q=np.asarray(sample['arm_q']);finger={n:m['initial_q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
   approach_reference_shapes.append(w.shapes(q,finger));count+=1
 if not count:raise RuntimeError('Predicted approach reference has no command states')
 approach_reference_provenance.append({'diagnostic':str(reference),'sha256':hashlib.sha256(reference.read_bytes()).hexdigest(),'states':count,'scope':'Only robot configurations reused as conservative discovery constraints; no physical observations or acceptance claim','reference_hazard_asset':reference_hazard_asset,'search_hazard_asset':spec['asset']})
measured_contact_shapes=[]
measured_contact_provenance=[]
for reference in a.measured_contact_reference:
 rm=json.loads((reference/'metadata.json').read_text())
 if rm['variant_spec']['asset']!=spec['asset'] or rm['robot_base']!=m['robot_base'] or rm['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Measured contact reference source/policy/layout mismatch')
 reference_rows=json.loads((reference/'trajectory.json').read_text())
 values=[row['next_obs']['q'] for row in reference_rows if row['phase'] in ['pull_door','success_hold','recover_open','release_handle','released_hold']][::3]
 result=json.loads((reference/'result.json').read_text());failure=result.get('failure') or ''
 if 'EXECUTION_COLLISION ' in failure:values.append(json.loads(failure.split('EXECUTION_COLLISION ',1)[1])['q'])
 if not values:raise RuntimeError('No measured contact motion states')
 for values_q in values:
  q=np.asarray(values_q);fq={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
  measured_contact_shapes.append(w.shapes(q[:7],fq))
 measured_contact_provenance.append({'episode':str(reference),'states':len(values),'trajectory_sha256':hashlib.sha256((reference/'trajectory.json').read_bytes()).hexdigest(),'result_sha256':hashlib.sha256((reference/'result.json').read_bytes()).hexdigest(),'scope':'Optional conservative discovery constraints from actual prior contact motion and its failed transition. Not an invariant or global infeasibility claim.'})
contact_proxy_spheres=[]
if a.screen_baseline_contact_proxies:
 for i in late:
  contact_proxy_spheres.extend(initial_spheres(dict(m,initial_q=rows[i]['next_obs']['q']),kin))
contact_proxy_centers=np.asarray([v[2] for v in contact_proxy_spheres]);contact_proxy_radii=np.asarray([v[3] for v in contact_proxy_spheres])
def baseline_contact_proxy_safe(hazards):
 if not contact_proxy_spheres:return True
 pad=margin+m['controller_parameters'].get('planner_hazard_extra_padding_m',0.)
 for _,_,lo,hi in hazards:
  distances=np.linalg.norm(np.maximum(np.maximum(lo-pad-contact_proxy_centers,contact_proxy_centers-hi-pad),0.),axis=1)-contact_proxy_radii
  if np.any(distances<0.):return False
 return True
command_sphere_provenance=[]
command_spheres=[]
for reference in a.command_sphere_reference:
 gd=json.loads(reference.read_text())
 if not gd.get('complete') or gd.get('valid_with_world') is not False or gd.get('valid_without_world') is not True:raise RuntimeError('Expected a completed external-world GPU rejection diagnostic')
 cdpath=Path(gd['diagnostic'])
 if hashlib.sha256(cdpath.read_bytes()).hexdigest()!=gd['diagnostic_sha256']:raise RuntimeError('GPU diagnostic source changed')
 cd=json.loads(cdpath.read_text());ep=Path(cd['episode']);rm=json.loads((ep/'metadata.json').read_text())
 if hashlib.sha256((ep/'metadata.json').read_bytes()).hexdigest()!=cd['metadata_sha256']:raise RuntimeError('Command metadata changed')
 if rm['variant_spec']['asset']!=spec['asset'] or rm['task']!=m['task'] or rm['source_yaml_sha256']!=m['source_yaml_sha256'] or rm['robot_base']!=m['robot_base'] or rm['robot_yaw_deg']!=m['robot_yaw_deg'] or rm['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Command source/policy/layout mismatch')
 if not np.array_equal(gd['command'],cd['command']):raise RuntimeError('GPU/CPU command mismatch')
 if hashlib.sha256((ep/'trajectory.json').read_bytes()).hexdigest()!=cd['trajectory_sha256']:raise RuntimeError('Command trajectory changed')
 reference_rows=json.loads((ep/'trajectory.json').read_text());full=np.asarray(reference_rows[-1]['next_obs']['q']).copy();full[:7]=np.asarray(gd['command']);refmeta=dict(rm,initial_q=full.tolist());command_spheres.extend(initial_spheres(refmeta,kin))
 command_sphere_provenance.append({'gpu_diagnostic':str(reference),'sha256':hashlib.sha256(reference.read_bytes()).hexdigest(),'cpu_diagnostic':str(cdpath),'cpu_diagnostic_sha256':gd['diagnostic_sha256'],'episode':str(ep),'blocking_obstacles':gd['blocking_obstacles'],'scope':'One reconstructed external-world rejected command as an optional conservative discovery constraint. Full cuRobo spheres including existing 5mm buffer versus original padded per-collider AABBs. Not a physical state, invariant, source-intersection proof or global feasibility claim.'})
# Optional finite CPU candidate pull states, not physical or GPU rejection evidence.
for reference in a.candidate_pull_reference:
 gd=json.loads(reference.read_text())
 if not gd.get('complete') or gd.get('physics_replay') or gd.get('unsafe_path_executed'):raise RuntimeError('Completed offline candidate pull report required')
 if gd['baseline_sha256']!=hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest() or gd['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Candidate pull baseline/policy mismatch')
 cfgpath=Path(gd['config']);cfg=json.loads(cfgpath.read_text())
 if hashlib.sha256(cfgpath.read_bytes()).hexdigest()!=gd['config_sha256'] or cfg[a.variant]['asset']!=spec['asset']:raise RuntimeError('Candidate pull source/config mismatch')
 for attr in ['paired','approach']:
  if hashlib.sha256(Path(gd[attr]).read_bytes()).hexdigest()!=gd[attr+'_sha256']:raise RuntimeError('Candidate pull input changed')
 pad=margin+m['controller_parameters'].get('planner_hazard_extra_padding_m',0.)
 if abs(pad-gd['planning_padding_m'])>1e-12:raise RuntimeError('Candidate pull original padding mismatch')
 reference_spec=cfg[a.variant];R=Rotation.from_euler('z',reference_spec.get('yaw_deg',0.),degrees=True).as_matrix();pos=np.asarray(reference_spec['position']);bounds=[]
 for name,obj,lo,hi in haz0:
  cloud=np.asarray(list(itertools.product(*zip(lo,hi))))@R.T+pos;bounds.append((name,cloud.min(0),cloud.max(0)))
 states=[]
 for pair in gd['pairs']:
  bad=[sample for sample in pair.get('samples',[]) if sample['minimum_proxy']['slack_m']<0.]
  if not bad:continue
  sample=min(bad,key=lambda v:v['minimum_proxy']['slack_m']);full=np.asarray(rows[sample['baseline_action_index']]['obs']['q']).copy();full[:7]=sample['arm_q'];sp=initial_spheres(dict(m,initial_q=full.tolist()),kin);centers=np.asarray([v[2] for v in sp]);radii=np.asarray([v[3] for v in sp]);minimum=None
  for name,lo,hi in bounds:
   values=np.linalg.norm(np.maximum(np.maximum(lo-pad-centers,centers-hi-pad),0.),axis=1)-radii;index=int(np.argmin(values));value=dict(slack_m=float(values[index]),obstacle=name,link=sp[index][0],sphere=index)
   if minimum is None or value['slack_m']<minimum['slack_m']:minimum=value
  if abs(minimum['slack_m']-sample['minimum_proxy']['slack_m'])>1e-10 or minimum['link']!=sample['minimum_proxy']['link']:raise RuntimeError('Candidate pull finite-state calibration failed')
  command_spheres.extend(sp);states.append(dict(seed=pair['seed'],baseline_action_index=sample['baseline_action_index'],arm_q=sample['arm_q'],calibrated_proxy=minimum))
 if not states:raise RuntimeError('No rejected candidate pull states')
 command_sphere_provenance.append(dict(candidate_pull=str(reference),sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),states=states,scope='One worst known CPU-proposed proxy-overlap state per seed, exactly recalibrated at the original pose and padding. Optional finite discovery constraint only; not physical/GPU evidence, not all future commands or a global infeasibility claim. Actual original safety gates remain mandatory.'))

command_centers=np.asarray([v[2] for v in command_spheres]);command_radii=np.asarray([v[3] for v in command_spheres])
def command_proxy_safe(hazards):
 if not command_spheres:return True
 pad=margin+m['controller_parameters'].get('planner_hazard_extra_padding_m',0.)
 for _,_,lo,hi in hazards:
  distance=np.linalg.norm(np.maximum(np.maximum(lo-pad-command_centers,command_centers-hi-pad),0.),axis=1)-command_radii
  if np.any(distance<0.):return False
 return True
paired_endpoint_shapes=[]
paired_endpoint_provenance=[]
for reference in a.paired_endpoint_reference:
 document=json.loads(reference.read_text())
 if document['baseline_sha256']!=hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest() or document['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Paired endpoint baseline policy mismatch')
 count=0
 for pair in document['pairs']:
  plans=[pair['on']]+pair['on'].get('failures',[])
  for plan in plans:
   if not plan.get('positions'):continue
   q=np.asarray(plan['positions'][-1]);finger={n:m['initial_q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
   paired_endpoint_shapes.append(physical_robot_shapes(w.shapes(q,finger)));count+=1
 paired_endpoint_provenance.append({'report':str(reference),'sha256':hashlib.sha256(reference.read_bytes()).hexdigest(),'endpoint_states':count})
paired_robot_paths=[]
if a.paired_reference:
 pairdoc=json.loads(a.paired_reference.read_text())
 if pairdoc['baseline_sha256']!=hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest() or pairdoc['controller_parameters']!=m['controller_parameters']:raise RuntimeError('Reference off-path baseline policy mismatch')
 for pair in pairdoc['pairs']:
  if not pair['off'].get('success'):raise RuntimeError('Reference off-plan failed')
  path=[]
  for joints in pair['off']['positions'][::2]:
   f={n:m['initial_q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
   path.append(physical_robot_shapes(w.shapes(np.asarray(joints),f)))
  paired_robot_paths.append((pair['seed'],path))
refs=[(n,o,np.asarray(o.getRotation()).copy(),np.asarray(o.getTranslation()).copy(),lo.copy(),hi.copy()) for n,o,lo,hi in haz0]
def move(pos,yaw):
 R=Rotation.from_euler('z',yaw,degrees=True).as_matrix();out=[]
 for n,o,rr,tt,lo,hi in refs:
  o.setRotation(R@rr);o.setTranslation(R@tt+pos);corners=np.asarray(list(itertools.product(*zip(lo,hi))));v=corners@R.T+pos;out.append((n,o,v.min(0),v.max(0)))
 return out
report={'task':m['task'],'variant':a.variant,'baseline':str(a.baseline),'scope':'Offline candidates only, not scene or task acceptance','discovery_broadphase':'Vectorized conservative source AABB overlap; every counted intersection still requires exact FCL','endpoint_sphere_screen':'AABB lower bounds only prune safe pairs; close pairs use exact source FCL at unchanged margin+3mm','discovery_min_blocking_samples':a.min_blocking_samples,'discovery_mid_stride':a.mid_stride,'stats':{},'candidates':[]};report['posture_adaptation_discovery']={'enabled':a.allow_endpoint_posture_adaptation,'scope':'Source initial robot and door/environment stay mandatory. Old redundant endpoint and later baseline arm postures are not constrained when enabled. Same Cartesian target, full on-plan endpoint validation, actual approach/pull and all final gates remain mandatory.'};report['additional_paired_endpoint_reference']=paired_endpoint_provenance;report['paired_endpoint_discovery_clearance_m']=margin+a.paired_endpoint_extra_clearance_m;report['additional_approach_reference']=approach_reference_provenance;report['additional_approach_clearance_m']=execmargin+a.paired_endpoint_extra_clearance_m;report['search_parameters']={k:[str(x) for x in v] if k in ['approach_reference','paired_endpoint_reference','approach_command_reference','measured_contact_reference','command_sphere_reference','candidate_approach_reference','candidate_pull_reference'] else str(v) if isinstance(v,Path) else v for k,v in vars(a).items()};report['invariant_gripper_screen']={'enabled':a.allow_endpoint_posture_adaptation,'sampled_actions':late,'goal_sphere_links':sorted(invariant_links),'goal_spheres':len(invariant_goal_spheres),'source_extra_clearance_m':.003,'scope':'Discovery only: fixed Cartesian policy uses measured baseline articulated gripper poses. Redundant arm postures remain unconstrained. Source hand plus original margin+3mm, support+5mm and distal goal proxy checks prune impossible placements; full same-target on-plans and actual dynamics still required.'};report['planning_proxy_discovery_screen']={'enabled':a.allow_endpoint_posture_adaptation and a.variant!='human','scope':'Also check initial and invariant distal goal spheres against the same per-collider padded AABBs used by cuRobo for glass/barrier, including baseline extra padding. This is a conservative discovery filter, not source-intersection evidence or final acceptance.'};report['approach_reference_geometry']='All original source robot shapes and conservative execution envelopes; necessity still uses physical source hulls/primitives only.';report['invariant_conservative_envelope_screen']={'enabled':a.screen_invariant_envelopes,'source_segment':[7,8,.055],'sampled_actions':late,'initial_full_shapes':len(initial_envelopes),'distal_shape_count':len(invariant_states[0]) if invariant_states else 0,'hazard_extra_m':.003,'support_m':.005,'scope':'Conservative placement discovery only: full initial robot plus fixed Cartesian distal arm/gripper/finger proxies. Not source intersection evidence or global infeasibility. Original final margins and all paired/physical acceptance gates unchanged.'};report['measured_contact_discovery_references']=measured_contact_provenance;report['command_sphere_discovery_references']=command_sphere_provenance;stats=report['stats'];stats.update(tested=0,endpoints=0,blocking=0,scene_legal=0)
report['baseline_contact_proxy_screen']={'enabled':a.screen_baseline_contact_proxies,'sampled_actions':late if a.screen_baseline_contact_proxies else [],'sphere_count':len(contact_proxy_spheres),'baseline_trajectory_sha256':hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest(),'scope':'Optional conservative placement discovery: full arm/gripper cuRobo spheres at sampled baseline contact states versus original padded per-collider hazard AABBs. Constrains baseline redundant posture as a finite filter only; not a candidate trajectory, global infeasibility or physical/GPU acceptance.'}
endpoint_proposals=[]
for x,y,z,yaw in candidates:
 stats['tested']+=1
 if stats['tested']%250==0:
  a.output.write_text(json.dumps(report,indent=2));print('SEARCH_PROGRESS',stats,flush=True)
 pos=np.asarray([x,y,z]);hz=move(pos,yaw)
 if not source_spheres_safe(hz,sphere_centers,sphere_radii,margin+.003):continue
 if a.allow_endpoint_posture_adaptation and a.variant!='human':
  planning_padding=margin+float(m['controller_parameters'].get('planner_hazard_extra_padding_m',0.))
  if any(float(np.min(np.linalg.norm(np.maximum(np.maximum(lo-planning_padding-sphere_centers,sphere_centers-hi-planning_padding),0),axis=1)-sphere_radii))<.003 for _,_,lo,hi in hz):continue
 if any(not clearance(robot[i],hz,margin)['valid'] for i in ([0] if a.allow_endpoint_posture_adaptation else [0,pre[-1],len(rows)-1])):continue
 if a.screen_invariant_envelopes:
  if not clearance(initial_envelopes,hz,margin+.003)['valid']:
   stats['initial_conservative_envelope_rejected']=stats.get('initial_conservative_envelope_rejected',0)+1;continue
  if any(not clearance(shapes,hz,margin+.003)['valid'] for shapes in invariant_states):
   stats['invariant_conservative_envelope_rejected']=stats.get('invariant_conservative_envelope_rejected',0)+1;continue

 if a.allow_endpoint_posture_adaptation and any(not clearance(shapes,hz,margin+.003)['valid'] for shapes in hand_states):continue
 if any(not clearance(shapes,hz,execmargin+a.paired_endpoint_extra_clearance_m)['valid'] for shapes in approach_reference_shapes):continue
 if any(not clearance(shapes,hz,margin+a.paired_endpoint_extra_clearance_m)['valid'] for shapes in paired_endpoint_shapes):continue
 if any(not clearance(shapes,hz,execmargin+a.paired_endpoint_extra_clearance_m)['valid'] for shapes in measured_contact_shapes):
  stats['measured_contact_reference_rejected']=stats.get('measured_contact_reference_rejected',0)+1;continue
 if not baseline_contact_proxy_safe(hz):
  stats['baseline_contact_proxy_rejected']=stats.get('baseline_contact_proxy_rejected',0)+1;continue
 if not command_proxy_safe(hz):
  stats['command_sphere_reference_rejected']=stats.get('command_sphere_reference_rejected',0)+1;continue
 stats['endpoints']+=1
 if a.endpoint_proposals_output:endpoint_proposals.append(dict(position=pos.tolist(),yaw_deg=float(yaw)))
 potential=potential_intersection_rows(mid_bounds,[[lo,hi] for _,_,lo,hi in hz])
 if int(np.sum(potential))<a.min_blocking_samples:continue
 hits=[mid[j] for j in np.flatnonzero(potential) if clearance(robot[mid[j]],hz,0.).get('intersecting',False)]
 if len(hits)<a.min_blocking_samples:continue
 stats['blocking']+=1
 if paired_robot_paths:
  paired_hits={str(seed):[j for j,shapes in enumerate(path) if clearance(shapes,hz,0.).get('intersecting',False)] for seed,path in paired_robot_paths}
  if not all(paired_hits.values()):continue
 support_shapes=[];support_info=None
 if a.pedestal:
  stats['shared_off_blocking']=stats.get('shared_off_blocking',0)+1
  width=a.support_width_m or (.10 if a.variant=='property' else .17)
  px,py=np.asarray([x,y])+np.asarray(a.support_offset_xy)
  tops=[(hi[2],n) for n,o,lo,hi in fixed if any(k in n.lower() for k in ['floor','counter','stand_instanceable']) and lo[0]<=px-width/2 and hi[0]>=px+width/2 and lo[1]<=py-width/2 and hi[1]>=py+width/2 and hi[2]<=z+.006]
  if not tops:stats['support_no_surface']=stats.get('support_no_surface',0)+1;continue
  top,support_name=max(tops)
  if z-top>.012:
   offset=bool(np.linalg.norm(a.support_offset_xy)>1e-9)
   low=np.asarray([px-width/2,py-width/2,top+.002]);high=np.asarray([px+width/2,py+width/2,z-(.022 if offset else .002)])
   boxes=[(low,high)]
   if offset:
    boxes.append((np.asarray([min(x,px)-width/2,min(y,py)-width/2,z-.022]),np.asarray([max(x,px)+width/2,max(y,py)+width/2,z-.002])))
   if any(np.any(hi<=lo) for lo,hi in boxes):continue
   support_shapes=[('added_support_'+str(i),fcl.CollisionObject(fcl.Box(*(hi-lo)),fcl.Transform((lo+hi)/2)),lo,hi) for i,(lo,hi) in enumerate(boxes)]
   support_info={'low':np.min([lo for lo,hi in boxes],axis=0).tolist(),'high':np.max([hi for lo,hi in boxes],axis=0).tolist(),'rests_on':support_name}
   if offset:support_info.update(parts=[{'low':lo.tolist(),'high':hi.tolist()} for lo,hi in boxes],design='Source-supported offset column with a short horizontal tray; full source collision checks, dynamic object stability and matched controls remain mandatory.')
   if not initial_safe(boxes,0.):
     stats['support_initial_spheres']=stats.get('support_initial_spheres',0)+1
     report.setdefault('support_rejections',[]).append({'position':pos.tolist(),'yaw_deg':yaw,'reason':'support_initial_spheres','proposed_support':support_info,'baseline_intersection_actions':hits});continue
   support_envelope_failures=[]
   if a.screen_invariant_envelopes:
    for state_index,shapes in enumerate([initial_envelopes,*invariant_states]):
     check=clearance(shapes,support_shapes,.005)
     if not check['valid']:
      support_envelope_failures.append({'state_index':state_index,'baseline_action':0 if state_index==0 else late[state_index-1],'check':check})
   if support_envelope_failures:
    stats['support_conservative_envelope_rejected']=stats.get('support_conservative_envelope_rejected',0)+1
    report.setdefault('support_rejections',[]).append({'position':pos.tolist(),'yaw_deg':float(yaw),'reason':'support_conservative_envelopes','proposed_support':support_info,'baseline_intersection_actions':hits,'source_intersection_in_reference_off_seeds':paired_hits if paired_robot_paths else {},'failures':support_envelope_failures})
    continue

   if a.allow_endpoint_posture_adaptation and any(not clearance(shapes,support_shapes,.005)['valid'] for shapes in hand_states):stats['support_source_hand']=stats.get('support_source_hand',0)+1;continue
   if any(not clearance(robot[i],support_shapes,.002)['valid'] for i in ([0] if a.allow_endpoint_posture_adaptation else [0,*pre[::4],*late])):stats['support_source_robot']=stats.get('support_source_robot',0)+1;continue
 late_bad=[] if a.allow_endpoint_posture_adaptation else [i for i in late if not clearance(robot[i],hz,margin)['valid']]
 if late_bad:
  report.setdefault('late_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'actions':late_bad[:12]});continue
 for body in g['moving_bodies']:w.move_body(body,np.eye(4))
 issues=[];supports=[]
 for hn,ho,hl,hh in hz+support_shapes:
  for n,o,lo,hi in fixed:
   if np.linalg.norm(np.maximum(np.maximum(hl-hi,lo-hh),0))>.008:continue
   d,hit=separation(ho,o);supported=abs(hl[2]-hi[2])<.008 and any(s in n.lower() for s in ['floor','counter','stand_instanceable'])
   if supported and d<.008:supports.append(n)
   elif hit:issues.append([hn,n])
 if issues or not (supports or support_info):
  if True:report.setdefault('scene_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'issues':issues,'supports':supports})
  continue
 stats['scene_legal']+=1
 initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]);door_safe=True
 for row in rows[::6]:
  pose=row['next_obs']['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
  for body in g['moving_bodies']:w.move_body(body,delta)
  door=[o for o in w.objects if any(o[0].startswith(b+'/') for b in g['moving_bodies'])]
  source_check=clearance(door,hz,margin);support_check=clearance(door,support_shapes,.002) if support_shapes else {'valid':True}
  if not source_check['valid'] or not support_check['valid']:door_safe=False;break
 if not door_safe:
  report.setdefault('door_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'action':rows.index(row),'source_check':source_check,'support_check':support_check});continue
 result={**spec,'position':pos.tolist(),'yaw_deg':float(yaw),'baseline_intersection_actions':hits,'source_supports':sorted(set(supports)),'proposed_support':support_info};
 if paired_robot_paths:result['source_intersection_in_reference_off_seeds']=paired_hits
 report['candidates'].append(result);print('CANDIDATE',a.variant,result['position'],yaw,'hits',len(hits),flush=True)
 if len(report['candidates'])>=20:break
 if stats['tested']%500==0:print('SEARCH',stats,flush=True)
report['search_complete']=True;a.output.write_text(json.dumps(report,indent=2));print('SEARCH_DONE',stats,'survivors',len(report['candidates']),flush=True)

if a.endpoint_proposals_output:
 a.endpoint_proposals_output.write_text(json.dumps(dict(scope='Finite endpoint/invariant-admissible discovery poses only; baseline intersection, full source support/scene/door legality and final physical safety are not established by this output.',search_report=str(a.output),search_report_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),baseline_trajectory_sha256=hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest(),endpoint_count=len(endpoint_proposals),search_complete=True,candidates=endpoint_proposals),indent=2)+'\n')
