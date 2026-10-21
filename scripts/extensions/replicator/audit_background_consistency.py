"""Exclude source-clutter settling as the cause of a hazard avoidance result.
Uses actual positions/quaternions only; velocity channels are never compared.
"""
import argparse,itertools,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from search_avoidance import ROOT,load_baseline,kitchen_world,DroidKinematics
from audit_avoidance import pose_difference,sha,initial_state_comparison

def trajectory_envelopes(path,world):
 lows=[];highs=[]
 with np.load(path/'physics_states.npz') as f:
  names=f['joint_names'].tolist();ids=f['arm_ids']
  for q,bottle in zip(f['q'],f['mustard']):
   grip={n:float(q[names.index(n)]) for n in world.GRIPPER_JOINT_NAMES}
   for _,_,lo,hi in world.shapes(q[ids],grip,bottle[:7]):lows.append(lo);highs.append(hi)
 return np.asarray(lows),np.asarray(highs)

def main():
 p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--baseline',type=Path);a=p.parse_args();out=a.episode.resolve();r=json.loads((out/'result.json').read_text());base=(a.baseline or ROOT/'datasets/avoidance_trials/replicator/base'/('baseline_human_03' if r['variant']=='human' else 'baseline_01')).resolve();br,data,names,ids=load_baseline(base)
 with np.load(base/'background_physics_states.npz') as bf,np.load(out/'background_physics_states.npz') as af:
  assert bf['paths'].tolist()==af['paths'].tolist();paths=bf['paths'].tolist();bp=bf['poses'].copy();ap=af['poses'].copy()
 layout=r['layout'];kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);world=kitchen_world(layout,kin,data,br)
 differences=[];changed=[]
 for i,path in enumerate(paths):
  diff=pose_difference(ap[0,i],bp[0,i]);significant=diff['max_translation_m']>.0001 or diff['max_rotation_deg']>.01
  differences.append(dict(path=path,**diff,significant=significant))
  if significant:changed.append(i)
 states=initial_state_comparison(base,out);cross=[]
 if changed:
  envelopes={'baseline':trajectory_envelopes(base,world),'augmented':trajectory_envelopes(out,world)}
  for i in changed:
   path=paths[i];reference=world._root_reference_transforms[path];u,_,vh=np.linalg.svd(reference[:3,:3]);ref_rotation=u@vh;corners=[]
   for j,(name,_,_,_) in enumerate(world.objects):
    if world._matches(name,path):
     _,_,lo,hi,_=world._object_reference[j];corners.extend(itertools.product(*zip(lo,hi)))
   if not corners:
    cross.append(dict(path=path,passed=False,reason='No collision bounds found'));continue
   local=(np.asarray(corners)-reference[:3,3])@ref_rotation
   bounds={}
   for label,poses in [('baseline_background',bp[:,i]),('augmented_background',ap[:,i])]:
    transformed=np.einsum('tij,vj->tvi',Rotation.from_quat(poses[:,3:7]).as_matrix(),local)+poses[:,None,:3]
    low=transformed.min(axis=(0,1));high=transformed.max(axis=(0,1));bounds[label]=(low,high)
   checks=[]
   for motion,(low,high) in envelopes.items():
    for background,(lo,hi) in bounds.items():
     gap=np.linalg.norm(np.maximum(0,np.maximum(low-hi,lo-high)),axis=1);minimum=float(gap.min());checks.append(dict(motion=motion,background=background,conservative_minimum_aabb_separation_m=minimum,required_m=.002,passed=minimum>=.002))
   cross.append(dict(path=path,passed=all(c['passed'] for c in checks),checks=checks,method='Every measured robot/gripper/carried-object safety-shape AABB against conservative source-collider swept AABB across EVERY physical background pose; preserves USD scale using polar reference rotation. Both trajectories x both backgrounds. AABB separation proves actual safety shapes cannot approach closer; an inconclusive bound fails rather than claiming clearance.'))
 same_nominal={k:v for k,v in layout.items() if k!='obstacle'}==br['layout'];passed=same_nominal and states['passed'] and all(x['passed'] for x in cross)
 report=dict(passed=bool(passed),baseline_episode=str(base.relative_to(ROOT)),same_nominal_source_layout=same_nominal,actual_robot_and_task_initial_states=states,background_rigid_body_count=len(paths),initial_task_physics_pose_differences=differences,significant_difference_thresholds=dict(translation_m=.0001,rotation_deg=.01),significantly_changed_bodies=len(changed),cross_checks=cross,baseline_physics_samples=len(bp),augmented_physics_samples=len(ap),hashes={label:sha(path/'background_physics_states.npz') for label,path in [('baseline',base),('augmented',out)]},scope='Physics recordings start after the 30 action-frame initial settling, before pregrasp execution; H5 first/settled pose statistics are separate. Nominal layout identity is not exact equality of all source dynamics. Significant source-body settling changes must be shown clear of BOTH real trajectories under BOTH measured background histories. Smaller differences are explicitly listed and are below 0.1mm/0.01deg; original full-world safety checks remain required.')
 target=out/'background_consistency_audit.json';assert not target.exists(),'Preserve earlier report';target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(passed=passed,significantly_changed_bodies=len(changed),cross_checks=[{'path':x['path'],'passed':x['passed'],'min':min([q['conservative_minimum_aabb_separation_m'] for q in x.get('checks',[])],default=None)} for x in cross]),indent=2));return int(not passed)
if __name__=='__main__':raise SystemExit(main())
