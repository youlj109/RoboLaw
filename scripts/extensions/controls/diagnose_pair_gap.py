"""Read-only exact nearest-point diagnostic for a saved paired path."""
import argparse
import json
from pathlib import Path

import fcl
import numpy as np
from pxr import Gf,Usd,UsdGeom
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.geometry_audit import static_collision_shapes
from kitchen_safety.planning import DroidKinematics

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('paired',type=Path)
    p.add_argument('--seed',type=int,default=43)
    p.add_argument('--variant',default='robot')
    a=p.parse_args()
    report=json.loads((a.paired/'paired_planning.json').read_text())
    config=json.loads((a.paired/'config.json').read_text())
    run=next(r for r in report['runs'] if r['seed']==a.seed and r['variant']==a.variant)
    index=run['true_shape_counterfactual']['minimum']['index']
    q=np.load(a.paired/f'base_seed{a.seed}_positions.npy')[index]
    baseline=Path(report['baseline_state_source'])
    result=json.loads((baseline/'result.json').read_text())
    records=json.loads((baseline/'records.json').read_text())
    state=next(r['before'] for r in records if r['phase'] in ('pregrasp','prepress'))
    pose=result['initial_robot_pose']
    kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    name={'human':'person','property':'glass','robot':'barrier'}[a.variant]
    spec=config['variants'][a.variant]
    stage=Usd.Stage.CreateInMemory();prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+name)
    prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
    prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));prim.AddRotateZOp().Set(spec['yaw_deg'])
    world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
    exact=static_collision_shapes(stage,str(prim.GetPath()))
    best=None
    for first in world.shapes(q,dict(zip(result['robot_joint_names'],state['q']))):
        if first[0]!='robot_hull' and not first[0].startswith('gripper/'):continue
        for second in exact:
            measured=fcl.DistanceResult()
            distance=float(fcl.distance(first[1],second[1],fcl.DistanceRequest(enable_nearest_points=True),measured))
            if best is None or distance<best['distance_m']:
                points=np.asarray(measured.nearest_points)
                best={'index':int(index),'robot_part':first[0],'obstacle':second[0],'distance_m':distance,
                    'nearest_points_world_m':points.tolist(),'gap_vector_m':(points[0]-points[1]).tolist()}
    print(json.dumps(best,indent=2))


if __name__=='__main__':main()
