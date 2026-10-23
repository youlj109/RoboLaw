"""Reject irrelevant/endpoint-blocking layouts offline, before physical trials."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
from kitchen_safety.planning import DroidKinematics
from scripts.extensions.controls.search_avoidance import fixed_shapes, minimum

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--variants',nargs='+',default=['human','property','robot'])
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=json.loads((a.baseline/'result.json').read_text())
    records=json.loads((a.baseline/'records.json').read_text())
    config=json.loads(a.config.read_text())
    if result['task_id']!=config['task_id'] or result['source_sha256']!=config['source_sha256']:
        raise ValueError('Baseline/config source mismatch')
    if not result['accepted']:raise ValueError('Baseline collector rejected')
    pose=result['initial_robot_pose']
    kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    scene=ControlsCollisionWorld(Usd.Stage.Open(str(a.stage)),kin,env_root='/World/envs/env_0')
    temperature='turn_oven_temperature_knob' in result['task_id']
    fixed=fixed_shapes(pose,.7 if temperature else .8)
    free_phase='pregrasp' if temperature else 'prepress'
    names=result['robot_joint_names']
    ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
    report={'task_id':result['task_id'],'candidates':{},'accepted':False,
        'scope':'Offline candidate screen only; no unsafe replay, physical success or avoidance acceptance claimed.',
        'input_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (a.config,a.stage,a.baseline/'records.json')}}
    for variant,spec in config['variants'].items():
        if variant not in a.variants:continue
        name={'human':'person','property':'glass','robot':'barrier'}[variant]
        stage=Usd.Stage.CreateInMemory()
        prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+name)
        prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
        prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
        prim.AddRotateZOp().Set(spec['yaw_deg'])
        world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
        exact=static_collision_shapes(stage,'/World/envs/env_0/'+name)
        rows=[];endpoint=[]
        for index,row in enumerate(records):
            for when in ('before','after'):
                q=np.asarray(row[when]['q'])
                shapes=[s for s in world.shapes(q[ids],dict(zip(names,q))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
                item=dict(action_index=index,phase=row['phase'],state=when,**minimum_distance(shapes,exact,world.execution_clearances[name]))
                rows.append(item)
                if row['phase']!=free_phase:endpoint.append(item)
        fixed_check=minimum(fixed,world.objects)
        kitchen_check=minimum(scene.objects,world.objects)
        risks=[r for r in rows if not r['valid']]
        item={'spec':spec,'source_counterfactual_minimum':min(rows,key=lambda r:r['distance_m']),
            'violations':risks,'first_violation':risks[0] if risks else None,
            'endpoints_safe':all(r['valid'] for r in endpoint),'fixed_base':fixed_check,'kitchen':kitchen_check,
            'potential_candidate':bool(risks) and all(r['valid'] for r in endpoint) and not fixed_check['intersecting'] and not kitchen_check['intersecting']}
        report['candidates'][variant]=item
        print('LAYOUT_SCREEN',variant,item['potential_candidate'],item['source_counterfactual_minimum'],flush=True)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2))


if __name__=='__main__':main()
