import json,numpy as np
from pathlib import Path
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3];m=json.loads((ROOT/'datasets/skill_trials/doors/base_attempt_08/metadata.json').read_text());kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg']);report={}
for variant,spec in json.loads((ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text()).items():
    stage=Usd.Stage.CreateInMemory();k=UsdGeom.Xform.Define(stage,'/World/envs/env_0/kitchen');k.GetPrim().GetReferences().AddReference(str(ROOT/'assets/kitchen/scene.usd'))
    x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(spec.get('yaw_deg',0))
    w=DoorCollisionWorld(stage,kin,env_root='/World/envs/env_0');issues=[]
    hazard=[r for r in w.objects if w._matches(r[0],spec['name'])];others=[r for r in w.objects if '/kitchen/' in r[0]]
    for hn,ho,hl,hh in hazard:
        for kn,ko,kl,kh in others:
            if np.any(hl>kh) or np.any(kl>hh):continue
            if hl[2]>=kh[2]-.006:continue
            n=w.fcl.collide(ho,ko,w.fcl.CollisionRequest(),w.fcl.CollisionResult())
            if n:issues.append({'hazard':hn,'kitchen':kn})
    report[variant]={'non_support_intersections':issues,'passed':not issues,'scope':'Authored static initial USD geometry; floor/counter support touching within6mm allowed; source graph independently checks robot layout'}
(ROOT/'reports/skill_trials/doors/initial_geometry.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
