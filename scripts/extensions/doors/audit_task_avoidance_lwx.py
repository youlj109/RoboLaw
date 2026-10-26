"""Require counterfactual necessity and actual safe rerouting for a door episode."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.search_avoidance_layouts import fixed_geometry,clearance,separation
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes

ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('episode');p.add_argument('--baseline',required=True);p.add_argument('--report-name',default='avoidance_audit.json');a=p.parse_args()
ep=Path(a.episode);base=Path(a.baseline);report_path=ep/a.report_name
if Path(a.report_name).name!=a.report_name:raise ValueError('report name must be a filename')
suffix='' if a.report_name=='avoidance_audit.json' else '_'+Path(a.report_name).stem
plot_path=ep/('path_comparison'+suffix+'.png')
outputs=[report_path,ep/('hazard_clearance'+suffix+'.json'),ep/('fixed_base_hazard_audit'+suffix+'.json'),plot_path]
if any(p.exists() for p in outputs):raise FileExistsError('Refusing to replace evidence; choose a new --report-name')
m=json.loads((ep/'metadata.json').read_text());bm=json.loads((base/'metadata.json').read_text());rows=json.loads((ep/'trajectory.json').read_text());br=json.loads((base/'trajectory.json').read_text());physical=json.loads((ep/'audit.json').read_text())['checks'];spec=m['variant_spec'];kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg'])
config=json.loads(Path(m['variant_config']).read_text())
s,kin,w,geometry=reconstruct(m,config,spec);fixed=[o for o in w.objects if '/stand_instanceable/' in o[0]];margin=w.execution_clearances[spec['name']];reference=transform(bm['initial_door_pose'][:3],bm['initial_door_pose'][3:])
counter_pose=rows[0]['obs'].get('hazard_pose') or m.get('actual_hazard_initial_pose')
if counter_pose is not None:w.set_obstacle_pose(spec['name'],counter_pose)
baseline_kin=DroidKinematics(bm['robot_base'],bm['robot_yaw_deg'])
def shapes(state,baseline=False):
 q=np.array(state['q']);angles=dict(zip(m['robot_joint_names'],q));original=w.kin
 if baseline:w.kin=baseline_kin
 result=w.shapes(q[:7],{n:angles[n] for n in w.GRIPPER_JOINT_NAMES});w.kin=original;return result
def objects(name):return [o for o in w.objects if w._matches(o[0],name)]
hazards=objects(spec['name']);support=objects(spec['support']['name']) if spec.get('support') else []
same=all(np.allclose(m[key],bm[key],atol=2e-6,rtol=0) for key in ['robot_base','robot_yaw_deg','handle_initial','hinge','axis','initial_door_pose']) and m['source_yaml_sha256']==bm['source_yaml_sha256'] and m['seed']==bm['seed']==42 and np.allclose(rows[0]['obs']['q'],br[0]['obs']['q'],atol=1e-5,rtol=0)
expected_controller=bm.get('controller_parameters')
if not isinstance(expected_controller,dict):raise ValueError('Baseline lacks recorded controller_parameters; collect a matching baseline rather than infer legacy defaults')
same_controller=bool(m.get('controller_parameters')==expected_controller)
same=bool(same and same_controller)
def independent_gates(directory):
 gates={}
 for filename,key in [('result.json','accepted'),('task_validation_full.json','accepted'),('self_collision_audit.json','passed')]:
  path=directory/filename
  gates[filename]={'passed':bool(json.loads(path.read_text()).get(key,False)) if path.exists() else False,'sha256':hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None}
 return {'passed':all(x['passed'] for x in gates.values()),'reports':gates}
baseline_gates=independent_gates(base);actual_gates=independent_gates(ep)
counter=[];proxy_counter=[];counter_geometry=physical_hazards(s,w,spec['name'],counter_pose)
for i,row in enumerate(br):
 for side in ['obs','next_obs']:
  observed_shapes=shapes(row[side],True);c=clearance(physical_robot_shapes(observed_shapes),counter_geometry,margin,True);counter.append({'action_index':i,'phase':row['phase'],'side':side,**c});proxy_counter.append(clearance(observed_shapes,hazards,margin,True))
violations=[r for r in counter if not r['valid']];minimum=min(counter,key=lambda r:r['slack_m'])
# The task fixes the TCP pose, not a redundant elbow solution. Check the
# measured new pregrasp endpoint at that same target; never force old q-goal.
goal_row=[r for r in rows if r['phase']=='pregrasp'][-1]
goal_fk=kin.fk(np.asarray(goal_row['next_obs']['q'][:7]));selected_height=m.get('selected_handle_offset_z_m',m['controller_parameters']['handle_offset_z']);allowed=m['controller_parameters'].get('handle_height_candidates_m',[m['controller_parameters']['handle_offset_z']]);selected_legal=any(abs(selected_height-x)<1e-9 for x in allowed)
approach=Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).apply([1.,0.,0.])
target=np.array(bm['handle_initial'])-approach*bm['controller_parameters'].get('pregrasp_distance_m',.13)+[0.,0.,selected_height-bm['controller_parameters']['handle_offset_z']]
goal_position_error=float(np.linalg.norm(goal_fk[:3,3]-target));from kitchen_safety.extensions.doors.task_model import grasp_rotation
target_rotation=grasp_rotation(approach,m['controller_parameters']['grasp_roll_deg'],m['controller_parameters'].get('grasp_pitch_deg',0.))
goal_rotation_error=float(Rotation.from_matrix(goal_fk[:3,:3]@target_rotation.T).magnitude())
endpoint_legal=bool(selected_legal and counter[0]['valid'] and clearance(shapes(goal_row['next_obs']),hazards,margin)['valid'] and goal_position_error<.004 and goal_rotation_error<.03)
handle_geometry=[o for o in w.objects if o[0] in geometry['handle_colliders']]
handle_low=np.min([o[2] for o in handle_geometry],axis=0);handle_high=np.max([o[3] for o in handle_geometry],axis=0)
selected_handle=np.array(m.get('selected_handle_initial',m['handle_initial']))
handle_point_legal=bool(np.all(selected_handle>=handle_low-.003) and np.all(selected_handle<=handle_high+.003) and np.linalg.norm(selected_handle[:2]-np.array(bm['handle_initial'])[:2])<1e-6)
endpoint_legal=bool(endpoint_legal and handle_point_legal and same_controller)
initial_issues=[];floor_support=[]
for hn,ho,hl,hh in hazards+support:
 for on,oo,ol,oh in [o for o in w.objects if '/kitchen/' in o[0]]+fixed:
  bound=float(np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0)))
  if bound>.007:continue
  resting=hl[2]>=oh[2]-.006 and ('/floor' in on.lower() or '/counter_' in on or 'stand' in on.lower())
  d,hit=separation(ho,oo)
  if resting and d<.007:floor_support.append(on)
  elif hit:initial_issues.append({'hazard':hn,'environment':on})
# Every added entity needs its own identifiable physical support.
entity_support=[]
for entity_name,entity_objects,support_objects in [(spec['name'],hazards,support if support else [o for o in w.objects if '/floor' in o[0].lower()])]+([(spec['support']['name'],support,[o for o in w.objects if any(k in o[0].lower() for k in ['/floor','/counter_','/stand_instanceable/'])])] if support else []):
 low=np.min([o[2] for o in entity_objects],axis=0);contacts=[]
 for hn,ho,hl,hh in entity_objects:
  if hl[2]>low[2]+.007:continue
  for on,oo,ol,oh in support_objects:
   if hl[2]<oh[2]-.006 or np.any(np.minimum(hh[:2],oh[:2])-np.maximum(hl[:2],ol[:2])<0):continue
   d,hit=separation(ho,oo)
   if d<.007:contacts.append({'entity_collider':hn,'support_collider':on,'distance_m':d,'bottom_minus_support_top_m':float(hl[2]-oh[2])})
 entity_support.append({'entity':entity_name,'supported':bool(contacts),'contacts':contacts})
actual=[];actual_physical=[];door_checks=[];support_checks=[];fixed_checks=[]
for i,state in enumerate(physical):
 if state.get('hazard_pose') is not None:w.set_obstacle_pose(spec['name'],state['hazard_pose'])
 hazards=objects(spec['name']);robot_shapes=shapes(state)
 pg=physical_hazards(s,w,spec['name'],state.get('hazard_pose') or counter_pose);pc=clearance(physical_robot_shapes(robot_shapes),pg,margin,True);actual_physical.append({'physics_record':i,'action_index':state['action_index'],'phase':state['phase'],**pc})
 c=clearance(robot_shapes,hazards,margin,True);actual.append({'physics_record':i,'action_index':state['action_index'],'phase':state['phase'],**c})
 if support:
  sc=clearance(robot_shapes,support,.002)
  if not sc['valid']:support_checks.append({'physics_record':i,**sc})
 delta=transform(state['door_body_pose'][:3],state['door_body_pose'][3:])@np.linalg.inv(reference);[w.move_body(body,delta) for body in geometry['moving_bodies']]
 door_shapes=[o for o in w.objects if any(o[0].startswith(body+'/') for body in geometry['moving_bodies'])]
 dc=clearance(door_shapes,hazards,margin)
 if not dc['valid']:door_checks.append({'physics_record':i,**dc})
 if support:
  dc=clearance(door_shapes,support,.002)
  if not dc['valid']:door_checks.append({'physics_record':i,**dc})
 fc=clearance(fixed,hazards,0.,True)
 if fc.get('intersecting'):fixed_checks.append({'physics_record':i,**fc})
actual_min=min(actual,key=lambda r:r['slack_m']);actual_physical_min=min(actual_physical,key=lambda r:r['slack_m']);actual_safe=all(r['valid'] for r in actual_physical) and all(r['valid'] for r in actual) and all(r['valid'] for r in physical) and not door_checks and not support_checks and not fixed_checks
def free_path(records):return np.array([r['next_obs']['q'][:7] for r in records if r['phase']=='pregrasp'])
bq=free_path(br);nq=free_path(rows);u=np.linspace(0,1,101)
def interp(q):return np.column_stack([np.interp(u,np.linspace(0,1,len(q)),q[:,j]) for j in range(7)])
bqi=interp(bq);nqi=interp(nq);joint=float(np.max(np.linalg.norm(bqi-nqi,axis=1)));tcp=float(np.max(np.linalg.norm(np.array([kin.fk(q)[:3,3] for q in bqi])-np.array([kin.fk(q)[:3,3] for q in nqi]),axis=1)))
result=json.loads((ep/'result.json').read_text());successful=result['accepted'] and rows[-1]['native_open'];environment_legal=not initial_issues and all(x['supported'] for x in entity_support) and not fixed_checks and not door_checks and not support_checks
property_stability={'required':m['variant']=='property','passed':True}
if m['variant']=='property':
 poses=np.array([state['hazard_pose'] for state in physical]);rots=Rotation.from_quat(poses[:,3:7]);initial_rot=Rotation.from_quat(poses[0,3:7]);translation=float(np.max(np.linalg.norm(poses[:,:3]-poses[0,:3],axis=1)));rotation=float(np.max((rots*initial_rot.inv()).magnitude())*180/np.pi);vertical=float(np.max(np.arccos(np.clip(rots.apply(np.tile([0.,0.,1.],(len(poses),1)))[:,2],-1,1)))*180/np.pi)
 property_stability.update(passed=translation<=.005 and rotation<=2 and vertical<=2,max_translation_m=translation,max_rotation_deg=rotation,max_vertical_tilt_deg=vertical,translation_limit_m=.005,rotation_limit_deg=2.)
report={'accepted':bool(same and endpoint_legal and environment_legal and any(r['phase']=='pregrasp' for r in violations) and actual_safe and successful and joint>1e-3 and baseline_gates['passed'] and actual_gates['passed'] and property_stability['passed']),'baseline_episode':str(base),'baseline_sha256':hashlib.sha256((base/'trajectory.json').read_bytes()).hexdigest(),'same_source_layout':same,'baseline_counterfactual':{'passed':not bool(violations),'violates_safety_margin':bool(violations),'violates_free_pregrasp':bool(any(r['phase']=='pregrasp' for r in violations)),'min_distance_m':minimum['distance_m'],'required_m':margin,'violations':violations,'minimum':minimum,'samples':len(counter),'geometry':'Actual robot collision hulls and articulated gripper meshes; exact USD cylinders/cubes/capsules and original hazard meshes. Coarse arm/finger capsules and primitive world-AABBs cannot establish necessity.','conservative_safety_model_minimum':min(proxy_counter,key=lambda r:r['slack_m'])},'augmented_actual':{'passed':bool(actual_safe),'min_distance_m':actual_physical_min['distance_m'],'required_m':margin,'minimum':actual_physical_min,'conservative_safety_model_minimum':actual_min,'samples':len(actual)},'initial_and_goal_feasible':endpoint_legal,'goal_tcp_error_m':goal_position_error,'goal_tcp_rotation_error_rad':goal_rotation_error,'hazard_environment_legal':environment_legal,'path_comparison':{'free_phase':'pregrasp','joint_max_deviation_rad':joint,'tcp_max_deviation_m':tcp,'baseline_samples':len(bq),'augmented_samples':len(nq),'method':'101 normalized-progress samples; necessity determined by geometric counterfactual, never by path difference alone'},'baseline_independent_gates':baseline_gates,'actual_independent_gates':actual_gates,'same_controller_policy':same_controller,'recorded_controller_parameters':{'baseline':expected_controller,'augmented':m['controller_parameters']},'handle_target_check':{'selected_point':selected_handle.tolist(),'source_handle_bounds':[handle_low.tolist(),handle_high.tolist()],'point_on_source_handle':handle_point_legal,'candidate_offsets_m':allowed,'selected_offset_m':selected_height,'candidate_failures':m.get('handle_candidate_failures',[])},'entity_support':entity_support,'property_stability':property_stability,'source_task_success':bool(successful),'initial_issues':initial_issues,'support_contact_colliders':sorted(set(floor_support)),'door_failures':door_checks,'support_failures':support_checks,'fixed_base_failures':fixed_checks,'sampling':'Baseline original observed states before and after each 15 Hz action; augmented actual robot and moving door recorded at every 120 Hz physics step plus action postchecks. Unsafe baseline never executed with hazards.','variant_spec':spec}
if report['accepted']:
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 bxyz=np.array([baseline_kin.fk(q)[:3,3] for q in bq]);nxyz=np.array([kin.fk(q)[:3,3] for q in nq])
 fig,axes=plt.subplots(2,2,figsize=(12,8));oldlabel='Baseline TCP (15 Hz actions)';newlabel='Augmented TCP (15 Hz actions)'
 for ax,ij,title in [(axes[0,0],(0,1),'Top view'),(axes[0,1],(1,2),'Side view')]:
  ax.plot(bxyz[:,ij[0]],bxyz[:,ij[1]],'--',color='#d1495b',lw=2,label=oldlabel);ax.plot(nxyz[:,ij[0]],nxyz[:,ij[1]],color='#00798c',lw=2,label=newlabel)
  ax.scatter([bxyz[0,ij[0]]],[bxyz[0,ij[1]]],color='black',s=30,label='Shared start');ax.scatter([bxyz[-1,ij[0]],nxyz[-1,ij[0]]],[bxyz[-1,ij[1]],nxyz[-1,ij[1]]],marker='x',s=60,color=['#d1495b','#00798c'])
  ax.set(xlabel='XYZ'[ij[0]]+' world (m)',ylabel='XYZ'[ij[1]]+' world (m)',title=title);ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=7)
 oldfree=[r for r in counter if r['phase']=='pregrasp'];newfree=[r for r in actual_physical if r['phase']=='pregrasp']
 for ax,oldvals,newvals,title in [(axes[1,0],oldfree,newfree,'Free approach: true collision geometry'),(axes[1,1],counter,actual_physical,'Whole episode: true collision geometry')]:
  ax.plot(np.linspace(0,1,len(oldvals)),[r['distance_m']*1000 for r in oldvals],'--',color='#d1495b',lw=1.4,label='Baseline: 15 Hz before/after action')
  ax.plot(np.linspace(0,1,len(newvals)),[r['distance_m']*1000 for r in newvals],color='#00798c',lw=1.4,label='Augmented: every 120 Hz physics step + postcheck')
  ax.axhline(margin*1000,color='black',ls=':',label=f'Unchanged safety margin: {margin*1000:g} mm');ax.set(xlabel='Normalized progress (each measured sequence)',ylabel='Minimum clearance (mm)',title=title);ax.grid(alpha=.25);ax.legend(fontsize=7)
 policy=f"same policy: handle +{bm.get('selected_handle_offset_z_m',.12):.2f} m baseline / +{selected_height:.2f} m augmented"
 fig.suptitle(m['variant'].title()+' source door avoidance — '+policy,fontsize=12);fig.tight_layout();fig.savefig(plot_path,dpi=160);plt.close(fig)
 report['path_comparison']['figure']=plot_path.name
outputs[0].write_text(json.dumps(report,indent=2)+'\n');outputs[1].write_text(json.dumps({'passed':actual_safe,'minimum':actual_min,'samples':len(actual),'variant_spec':spec,'scope':report['sampling']},indent=2)+'\n');outputs[2].write_text(json.dumps({'passed':not fixed_checks and not initial_issues,'samples':len(actual),'violations':fixed_checks,'variant_spec':spec,'scope':'Exact original fixed base/stand against augmented asset and support initially, and measured obstacle at all recorded physics states.'},indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('baseline_counterfactual','variant_spec')},indent=2));raise SystemExit(0 if report['accepted'] else 1)
