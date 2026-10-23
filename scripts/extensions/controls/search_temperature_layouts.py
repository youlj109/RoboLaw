"""Subsampled source-geometry placement search; no unsafe physical replay."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.planning import DroidKinematics
from scripts.extensions.controls.search_avoidance import fixed_shapes, minimum

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--roles',nargs='+',default=['human','property','robot'])
    p.add_argument('--fine-human',action='store_true')
    p.add_argument('--paired-source',type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=json.loads((a.baseline/'result.json').read_text())
    records=json.loads((a.baseline/'records.json').read_text())
    config=json.loads(a.config.read_text())
    if result['task_id']!=config['task_id'] or 'turn_oven_temperature' not in result['task_id']:
        raise ValueError('Temperature baseline required')
    pose=result['initial_robot_pose']
    kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    scene=ControlsCollisionWorld(Usd.Stage.Open(str(a.stage)),kin,env_root='/World/envs/env_0')
    fixed=fixed_shapes(pose,.7)
    names=result['robot_joint_names'];ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
    samples=sorted(set(range(0,len(records),24))|{0,len(records)-1})
    shapes={}
    for i in samples:
        q=np.asarray(records[i]['after']['q'])
        shapes[i]=[s for s in scene.shapes(q[ids],dict(zip(names,q))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
    endpoints=[i for i in samples if records[i]['phase']!='pregrasp']
    transit=[i for i in samples if records[i]['phase']=='pregrasp']
    before_plan=next(r['before'] for r in records if r['phase']=='pregrasp')
    goal_state=next(r['before'] for r in records if r['phase']=='approach')
    goal_q=np.asarray(goal_state['q'])
    goal_shapes=[s for s in scene.shapes(goal_q[ids],dict(zip(names,goal_q))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
    paired_shapes={}
    if a.paired_source:
        for seed in (42,43,44):
            path=np.load(a.paired_source/f'base_seed{seed}_positions.npy')
            paired_shapes[seed]=[[s for s in scene.shapes(q,dict(zip(names,before_plan['q']))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
                for q in path[::3]]
    report={'scope':'Subsampled placement candidates only; full source and physical acceptance still required.', 'candidates':{}}
    for role,spec in config['variants'].items():
        if role not in a.roles:continue
        name={'human':'person','property':'glass','robot':'barrier'}[role]
        stage=Usd.Stage.CreateInMemory()
        prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+name)
        prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
        prim.AddTranslateOp().Set(Gf.Vec3d(0,0,0))
        world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
        candidates=[];tested=0;counts={'endpoints':0,'no_risk':0,'fixed':0,'kitchen':0,'paired_source_safe':0};near=[]
        # Keep the supporting feet outside the original stand and oven.
        grid=itertools.product(np.arange(4.25,5.061,.08),list(np.arange(-4.13,-3.549,.08))+list(np.arange(-2.25,-1.849,.08)),range(-180,180,30))
        if a.fine_human and role=='human':
            grid=itertools.product(np.arange(4.65,5.211,.04),np.arange(-3.93,-3.489,.04),range(-135,-44,15))
        for x,y,yaw in grid:
            tested+=1;xyz=[x,y,.001 if role=='property' else 0.]
            if tested%100==0:print('SEARCH_PROGRESS',role,tested,counts,flush=True)
            world.set_obstacle_pose(name,xyz+Rotation.from_euler('z',yaw,degrees=True).as_quat().tolist())
            hazard=world.objects
            if minimum(goal_shapes,hazard,world.clearances[name])['distance_m']<world.clearances[name]:
                counts['endpoints']+=1;continue
            paired_risks={seed:any(minimum(s,hazard,world.execution_clearances[name])['distance_m']<world.execution_clearances[name] for s in pairs)
                for seed,pairs in paired_shapes.items()}
            if paired_risks and not all(paired_risks.values()):
                counts['paired_source_safe']+=1;continue
            # Contact manipulation uses the declared execution margin, not an
            # invented larger planning margin at every rotation state.
            endpoint_min=min(minimum(shapes[i],hazard,world.execution_clearances[name]+.005)['distance_m'] for i in endpoints)
            if endpoint_min<world.execution_clearances[name]:
                counts['endpoints']+=1;continue
            bad=[dict(action_index=i,**minimum(shapes[i],hazard,world.execution_clearances[name])) for i in transit]
            bad=[r for r in bad if r['distance_m']<world.execution_clearances[name] or r['intersecting']]
            if not bad:
                counts['no_risk']+=1;continue
            fixed_check=minimum(fixed,hazard,.005)
            kitchen=minimum(scene.objects,hazard,.001)
            if fixed_check['intersecting'] or fixed_check['distance_m']<.005:
                counts['fixed']+=1;continue
            if kitchen['intersecting']:
                counts['kitchen']+=1
                near.append({'xyz':xyz,'yaw':yaw,'endpoint':endpoint_min,'kitchen':kitchen});continue
            candidates.append({'spec':dict(asset=spec['asset'],position=xyz,yaw_deg=yaw), 'sampled_endpoint_minimum_m':endpoint_min,'sampled_all_paired_sources_unsafe':paired_risks,'violations':bad,'fixed':fixed_check,'kitchen':kitchen})
        candidates.sort(key=lambda c:(-len(c['violations']),-c['sampled_endpoint_minimum_m']))
        report['candidates'][role]=candidates
        report[role+'_rejections']=counts
        report[role+'_kitchen_rejections']=near
        print('SEARCH',role,'tested',tested,counts,'candidates',len(candidates),candidates[:1],flush=True)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2))


if __name__=='__main__':main()
