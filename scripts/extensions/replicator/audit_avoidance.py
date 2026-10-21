"""Causal geometric evidence: same successful no-hazard task versus augmented execution.
Counterfactual replay only queries geometry and never executes unsafe motion.
"""
import argparse,hashlib,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from audit import audit_episode,hazard_stage,HAZARDS
from audit_fixed_base import fixed_colliders
from search_avoidance import load_baseline,kitchen_world,initial_geometry
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def pose_rows(world,qs,bottles,names,ids,phases,indices,mode):
 rows=[]
 for i,(q,bottle,phase,index) in enumerate(zip(qs,bottles,phases,indices)):
  grip={n:float(q[names.index(n)]) for n in world.GRIPPER_JOINT_NAMES}
  measure=world.clearance(q[ids],mode=mode,gripper_q=grip,object_pose=bottle[:7])
  rows.append(dict(sample_index=i,action_index=int(index),phase=str(phase),**measure))
 return rows

def controls(result):
 g=result['requested_grasp'];effective=result.get('effective_controls',{})
 return dict(planning_strategy=result.get('planning_strategy','not_recorded_before_v1'),seed=result['seed'],planner_seed=effective.get('planner_seed',result['seed']),planner=result['planner'],tilt_deg=g['tilt_deg'],yaw_deg=g['yaw_deg'],offset_z_m=g['offset_z_m'],offset_y_m=g['offset_y_m'],transport_bottle_height_m=g['transport_bottle_height_m'],supported_release=g['supported_release'],transport_tilt_cap_deg=g['transport_tilt_cap_deg'],placement_strategy=result['placement_strategy'],placement_tcp_height_override_m=g['placement_tcp_height_override_m'],transfer_mode=result['transfer_mode'],transport_object_padding_m=effective.get('transport_object_padding_m',result['layout']['planning'].get('transport_object_padding_m_by_variant',{}).get(result['variant'],0.)))
def report_passed(value):
 return value.get('passed') is True if isinstance(value,dict) else bool(value) and all(report_passed(x) for x in value)
def rgb_passed(episode):
 report=ROOT/'reports/avoidance_trials/replicator'/episode.parent.name/episode.name
 return all(report_passed(read(report/(name+'.json'))) for name in ['validation','videos'])
def plot_comparison(out,base,rows,actual,required):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 with h5py.File(base/'trajectory.h5') as bf,h5py.File(out/'trajectory.h5') as af:
  bp=bf['obs/mustard'][:,:3];ap=af['obs/mustard'][:,:3]
  bi=np.isin(bf['phase'].asstr()[:],['lift','above_bowl','lower']);ai=np.isin(af['phase'].asstr()[:],['lift','above_bowl','lower'])
  fig,axes=plt.subplots(1,3,figsize=(16,4.6))
  for ax,(i,j,title) in zip(axes[:2],[(0,1,'Measured carried-bottle route: top view'),(0,2,'Measured carried-bottle route: side view')]):
   ax.plot(bp[bi,i],bp[bi,j],color='#bf3843',label='No-hazard control',lw=2);ax.plot(ap[ai,i],ap[ai,j],color='#187dca',label='With hazard',lw=2);ax.set(xlabel='x (m)',ylabel=('y (m)' if j==1 else 'z (m)'),title=title);ax.grid(alpha=.2);ax.legend()
  axes[2].plot(np.linspace(0,1,len(rows)),[x['distance_m'] for x in rows],color='#bf3843',label='Control replay with hazard (not executed)');axes[2].plot(np.linspace(0,1,len(actual['samples'])),[x['distance_m'] for x in actual['samples']],color='#187dca',label='Actual augmented trajectory');axes[2].axhline(required,color='black',ls='--',label='Unchanged execution margin');axes[2].set(xlabel='Normalized episode progress',ylabel='Nearest envelope separation (m)',title='Causal clearance evidence');axes[2].grid(alpha=.2);axes[2].legend(fontsize=7)
  fig.suptitle('Same source layout, seed and controls; geometry-only unsafe counterfactual');fig.tight_layout();fig.savefig(out/'path_comparison.png',dpi=160);plt.close(fig)

def pose_difference(a,b):
 a=np.array(a,dtype=float,copy=True);b=np.array(b,dtype=float,copy=True)
 translation=np.linalg.norm(a[...,:3]-b[...,:3],axis=-1)
 angle=np.rad2deg((Rotation.from_quat(a[...,3:7].reshape(-1,4))*Rotation.from_quat(b[...,3:7].reshape(-1,4)).inv()).magnitude())
 return dict(max_translation_m=float(translation.max()),max_rotation_deg=float(angle.max()))

def initial_state_comparison(base,out):
 stages=[]
 with h5py.File(base/'trajectory.h5') as bf,h5py.File(out/'trajectory.h5') as af:
  for index,label in [(0,'first_recorded_state'),(30,'settled_before_task_motion')]:
   qdiff=float(np.max(np.abs(af['obs/q'][index]-bf['obs/q'][index])))
   robot=pose_difference(af['obs/robot_body_pose_w'][index],bf['obs/robot_body_pose_w'][index])
   objects={name:pose_difference(af['obs/'+name][index,:7],bf['obs/'+name][index,:7]) for name in ['mustard','bowl']}
   background=pose_difference(af['obs/background_rigid_poses'][index],bf['obs/background_rigid_poses'][index])
   passed=qdiff<=1e-5 and robot['max_translation_m']<=1e-6 and robot['max_rotation_deg']<=.001 and all(v['max_translation_m']<=.0001 and v['max_rotation_deg']<=.01 for v in objects.values())
   stages.append(dict(index=index,stage=label,passed=bool(passed),max_robot_joint_difference_rad=qdiff,robot_body_pose_differences=robot,task_object_pose_differences=objects,background_rigid_pose_differences=background))
 return dict(passed=all(s['passed'] for s in stages),stages=stages,tolerances=dict(robot_joint_rad=1e-5,robot_position_m=1e-6,robot_orientation_deg=.001,task_object_position_m=.0001,task_object_orientation_deg=.01),scope='Compare actual recorded initial and settled robot and original task-object states. Tiny rigid-body settling differences are measured explicitly. Background dynamics differences are reported rather than hidden; authored source reset layout and original task relations are checked separately.')

def hazard_stationarity(out,result,stage,settled_pose):
 from pxr import Usd,UsdPhysics
 with np.load(out/'physics_states.npz') as f:
  has=f['has_hazard_pose'];poses=f['hazard_pose'];count=len(has)
  if bool(has.all()):
   difference=pose_difference(poses,np.broadcast_to(poses[0],poses.shape));height=float(np.max(np.abs(poses[:,2]-poses[0,2])));upright=float(np.rad2deg(np.arccos(np.clip(Rotation.from_quat(poses[:,3:7]).as_matrix()[:,2,2],-1,1))).max())
   return dict(passed=bool(difference['max_translation_m']<.002 and difference['max_rotation_deg']<=2. and height<.002 and upright<=2.),method='Every recorded physics substep from first sample through completion; measured rigid-body xyz + xyzw, including vertical change and absolute upright angle.',samples=count,observed_rigid_pose_samples=int(has.sum()),**difference,max_vertical_displacement_m=height,max_upright_deviation_deg=upright,thresholds=dict(translation_m=.002,vertical_m=.002,rotation_deg=2.,upright_deg=2.),physics_sha256=sha(out/'physics_states.npz'))
  enabled=[];animated=[]
  for prim in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
   if prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get():enabled.append(str(prim.GetPath()))
   for attr in prim.GetAttributes():
    if attr.GetName().startswith('xformOp:') and attr.GetNumTimeSamples():animated.append(str(attr.GetPath()))
  authored=np.r_[result['obstacle']['position'],Rotation.from_euler('z',result['obstacle'].get('yaw_deg',0),degrees=True).as_quat()];difference=pose_difference(np.asarray(settled_pose),authored)
  passed=result['obstacle'].get('static') is True and not bool(has.any()) and not enabled and not animated and difference['max_translation_m']<1e-6 and difference['max_rotation_deg']<.001
  return dict(passed=bool(passed),method='Static non-RigidBody USD collider has no simulated motion state; zero npz placeholders are not measurements. Verify immutable static asset, no enabled rigid bodies or animated transform samples, and measured settled world root equals authored root.',samples=count,observed_rigid_pose_samples=int(has.sum()),enabled_rigid_bodies=enabled,animated_transforms=animated,settled_to_authored_pose_difference=difference,asset=result['obstacle']['usd_path'],asset_sha256=sha(ROOT/result['obstacle']['usd_path']),thresholds=dict(settled_root_translation_m=1e-6,settled_root_rotation_deg=.001),physics_sha256=sha(out/'physics_states.npz'))

def main():
 p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--baseline',type=Path);p.add_argument('--report-name',default='avoidance_audit.json');a=p.parse_args();out=a.episode.resolve();r=read(out/'result.json');base=(a.baseline or ROOT/'datasets/avoidance_trials/replicator/base'/('baseline_human_03' if r['variant']=='human' else 'baseline_01')).resolve()
 target=out/a.report_name
 if target.exists():raise FileExistsError(f'Preserve existing report: {target}; use --report-name for a revision')
 br,data,names,ids=load_baseline(base);layout=r['layout'];same={k:v for k,v in layout.items() if k!='obstacle'}==br['layout'];assert same,'Refuse counterfactual across different original source layouts/settings'
 variant=r['variant'];hazard=HAZARDS[variant];cfg=layout['planning'];kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);stage,_=hazard_stage(layout,variant,r['obstacle']);world=CollisionWorld(stage,kin,clearances=cfg['clearances_m'],execution_clearances=cfg['execution_clearances_m']);pose=np.r_[r['obstacle']['position'],Rotation.from_euler('z',r['obstacle'].get('yaw_deg',0),degrees=True).as_quat()]
 if 'settled_poses' in r:pose=np.array(r['settled_poses']['obstacle'])
 world.set_obstacle_pose(hazard,pose)
 with np.load(base/'physics_states.npz') as f:
  q=f['q'];b=f['mustard'];phases=f['phase'];indices=f['action_index'];pn=f['joint_names'].tolist();pi=f['arm_ids'];rows=pose_rows(world,q,b,pn,pi,phases,indices,'execution')
 required=cfg['execution_clearances_m'][hazard];planning_required=cfg['clearances_m'][hazard];violations=[x for x in rows if not x['valid']];planning_violations=[x for x in rows if x['distance_m']<planning_required or x['intersecting']];middle=[x for x in violations if x['phase'] not in ['settle','release','retreat']];minimum=min(rows,key=lambda x:x['slack_m'])
 endpoints=sorted(set([0,30,len(data['q'])-1]+[max(i for i,p in enumerate(data['phases']) if p==phase) for phase in ['approach','close','lower','release']]))
 endpoint_rows=pose_rows(world,data['q'][endpoints],data['mustard'][endpoints],names,ids,[data['phases'][i] for i in endpoints],endpoints,'planning');feasible=all(x['valid'] for x in endpoint_rows)
 env=kitchen_world(layout,kin,data,br);fixed=fixed_colliders(data['robot_body_pose_w'][0,data['body_names'].index('panda_link0')]);legal,pairs=initial_geometry(world,env,fixed,variant)
 actual=audit_episode(out);fixed_report=read(out/'fixed_base_hazard_audit.json');self_report=read(out/'self_collision_audit.json');source=read(out/'source_relation_audit.json');overview=read(out/'overview_validation.json');base_source=read(base/'source_relation_audit.json');base_self=read(base/'self_collision_audit.json')
 actual_physics=r.get('deferred_safety',{}).get('passed') is True
 comparison=[]
 with h5py.File(base/'trajectory.h5') as bf,h5py.File(out/'trajectory.h5') as af:
  for phase in ['pregrasp','lift','above_bowl']:
   bi=np.flatnonzero(bf['phase'].asstr()[:]==phase);ai=np.flatnonzero(af['phase'].asstr()[:]==phase)
   if len(bi)<2 or len(ai)<2:continue
   values={}
   for name,col in [('q',ids),('tcp',slice(None)),('mustard',slice(0,3))]:
    bv=bf['obs/'+name][:][bi][:,col];av=af['obs/'+name][:][ai][:,col];s=np.linspace(0,1,101)
    bn=np.stack([np.interp(s,np.linspace(0,1,len(bv)),bv[:,i]) for i in range(bv.shape[1])],axis=1);an=np.stack([np.interp(s,np.linspace(0,1,len(av)),av[:,i]) for i in range(av.shape[1])],axis=1);norm=np.linalg.norm(an-bn,axis=1)
    values[name]=dict(mean_norm=float(norm.mean()),max_norm=float(norm.max()),units='rad' if name=='q' else 'm')
   comparison.append(dict(phase=phase,baseline_frames=len(bi),augmented_frames=len(ai),normalized_phase_samples=101,changes=values))
 initial_comparison=initial_state_comparison(base,out)
 stationarity=hazard_stationarity(out,r,stage,pose)
 background_report=read(out/'background_consistency_audit.json') if (out/'background_consistency_audit.json').exists() else {}
 baseline_ok=rgb_passed(base) and br['success'] and br['arena_success'] and br['deferred_safety']['passed'] and base_source['passed'] and base_self['passed']
 actual_ok=rgb_passed(out) and r.get('body_grasp_carried_region',{}).get('passed') is True and r['success'] and r['arena_success'] and r['final_on_destination'] and actual_physics and actual['clearance_passed'] and source['passed'] and fixed_report['passed'] and self_report['passed'] and overview['passed']
 same_controls=controls(r)==controls(br)
 gates=dict(source_background_not_a_route_confound=background_report.get('passed') is True,identical_effective_controls=bool(same_controls),valid_no_hazard_baseline=bool(baseline_ok),same_source_layout=bool(same),actual_initial_and_settled_states_match=initial_comparison['passed'],baseline_intermediate_execution_margin_violated=bool(middle),augmented_physical_and_geometry_passed=bool(actual_ok),initial_and_goal_feasible=bool(feasible),hazard_environment_legal=bool(legal and fixed_report['passed']),hazard_approximately_stationary=stationarity['passed'])
 report=dict(schema_version=3,background_consistency_report='background_consistency_audit.json',background_consistency_sha256=sha(out/'background_consistency_audit.json') if background_report else None,accepted=all(gates.values()),gates=gates,baseline_episode=str(base.relative_to(ROOT)),baseline_sha256={'result':sha(base/'result.json'),'trajectory':sha(base/'trajectory.h5'),'physics_states':sha(base/'physics_states.npz')},same_source_layout=same,same_effective_controls=same_controls,effective_controls={'baseline':controls(br),'augmented':controls(r)},initial_state_comparison=initial_comparison,hazard_stationarity=stationarity,baseline_counterfactual=dict(passed=not violations,violates_safety_margin=bool(violations),baseline_path_safe_with_hazard=not violations,min_distance_m=minimum['distance_m'],required_m=required,planning_required_m=planning_required,violations=violations,planning_violation_count=len(planning_violations),intermediate_violation_count=len(middle),minimum=minimum,samples=len(rows),sampling='Every saved physics substep of the real no-hazard episode; recorded q, fingers and bottle pose; hazard at augmented settled pose. Pure geometry; unsafe trajectory not executed.'),augmented_actual=dict(passed=bool(actual_ok),min_distance_m=actual['minimum_clearance_m'],required_m=required,action_samples=actual['states_checked'],full_physics_substeps_passed=actual_physics,hazard_stationarity_passed=stationarity['passed']),initial_and_goal_feasible=bool(feasible),endpoint_checks=endpoint_rows,hazard_environment_legal=bool(legal and fixed_report['passed']),hazard_environment_intersections=pairs,source_relation_report='source_relation_audit.json',fixed_base_report='fixed_base_hazard_audit.json',path_comparison=dict(method='101 samples per matched free-motion phase, linearly normalized phase progress; route difference alone is not acceptance evidence.',phases=comparison),scope='Source robot, targets, support, asset identities and all clearance settings fixed. Counterfactual baseline violation + safe replanned real physics + independent source/fixed/self/RGB/overview reports required. Distances refer to the established arm/gripper/carried-object safety model (including conservative capsules/hulls), not a claim of inevitable exact physical surface contact. No executed collision counterfactual; one fixed sample, not success-rate evidence.')
 if not (out/'path_comparison.png').exists():plot_comparison(out,base,rows,actual,required)
 report['path_comparison']['artifact']='path_comparison.png';report['path_comparison']['sha256']=sha(out/'path_comparison.png')
 target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(accepted=report['accepted'],gates=gates,baseline_minimum=minimum,augmented_minimum=actual['minimum_clearance_m']),indent=2));return int(not report['accepted'])
if __name__=='__main__':raise SystemExit(main())
