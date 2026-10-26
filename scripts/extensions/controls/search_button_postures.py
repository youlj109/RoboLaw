"""Screen redundant joint endpoints for the unchanged button TCP and human."""
import argparse
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.extensions.controls.planning import button_prepress_pose

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,required=True)
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--stage',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--samples',type=int,default=128)
p.add_argument('--roll-deg',type=float,default=0.)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
config=json.loads(a.config.read_text());result=json.loads((a.baseline/'result.json').read_text())
records=json.loads((a.baseline/'records.json').read_text())
pose=config['robot_pose_xyzw']
kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
stage=Usd.Stage.CreateInMemory();stage.GetRootLayer().subLayerPaths.append(str(a.stage.resolve()))
spec=config['variants']['human'];prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/person')
prim.GetPrim().GetReferences().AddReference(str(Path(spec['asset']).resolve()))
prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));prim.AddRotateZOp().Set(spec['yaw_deg'])
world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
initial=records[0]['before'];gripper=dict(zip(result['robot_joint_names'],initial['q']))
target=button_prepress_pose(config,roll_deg=a.roll_deg)
old=np.array(config['prepress_planning']['ik_branch_seed']);rng=np.random.default_rng(2026)
report={'scope':'Endpoint diagnostic only. Same original TCP, human and robot base. '
    'No self-collision, path, physical success or avoidance acceptance implied.',
    'config':str(a.config),'roll_deg':a.roll_deg,'samples':[],'candidates':[]}
for index in range(a.samples):
    seed=old if index==0 else rng.uniform(kin.limits[:,0],kin.limits[:,1])
    row={'index':index,'seed':seed.tolist()}
    try:
        q=kin.ik(target,seed,attempts=1)
        row['q']=q.tolist();row['prepress_safe']=world.valid(q,gripper_q=gripper)
        row['prepress_failure']=world.last_clearance
        if row['prepress_safe']:
            states=[]
            for y in np.linspace(-.44,-.39,11):
                t=target.copy();t[1,3]=y;q=kin.ik(t,q,attempts=1)
                if not world.valid(q,gripper_q=gripper,obstacle_filter=('person',)):
                    row['press_failure']=world.last_clearance;break
                states.append(q.tolist())
            if len(states)==11:
                report['candidates'].append({'index':index,'ik_branch_seed':row['q'],
                    'press_q':states,'joint_distance_from_previous':float(np.linalg.norm(np.array(row['q'])-old))})
                print('POSTURE_CANDIDATE',index,flush=True)
    except RuntimeError as exc:row['error']=str(exc)
    report['samples'].append(row)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
print('POSTURE_DONE',len(report['candidates']),flush=True)
