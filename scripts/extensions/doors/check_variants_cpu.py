"""Read-only geometric counterfactual of recorded robot states against proposed hazards."""
import json,argparse
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
parser=argparse.ArgumentParser();parser.add_argument('--episode',default='datasets/skill_trials/doors/base_attempt_04');args=parser.parse_args()
episode=ROOT/args.episode
meta=json.loads((episode/'metadata.json').read_text())
records=json.loads((episode/'trajectory.json').read_text())
names=json.loads((ROOT/'reports/skill_trials/doors/probe/probe.json').read_text())['robot_joint_names']
kin=DroidKinematics(meta['robot_base'],meta['robot_yaw_deg'])
report={}
for variant,spec in json.loads((ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text()).items():
    stage=Usd.Stage.CreateInMemory();x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(spec.get('yaw_deg',0))
    world=DoorCollisionWorld(stage,kin,env_root='/World/envs/env_0')
    best=None
    for i,row in enumerate(records[::4]):
        q=np.array(row['obs']['q']);angles=dict(zip(names,q))
        c=world.clearance(q[:7],gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES},obstacle_filter=[spec['name']])
        if best is None or c['slack_m']<best['slack_m']:best={'frame_index':i*4,**c}
    report[variant]={'baseline_episode':args.episode,'collider_count':len(world.objects),'baseline_recorded_approach_minimum':best,'geometry_bounds':{'min':world.low.min(0).tolist(),'max':world.high.max(0).tolist()}}
(ROOT/'reports/skill_trials/doors/hazard_counterfactual.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
