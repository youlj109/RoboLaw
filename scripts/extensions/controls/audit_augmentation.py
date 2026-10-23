"""CPU-only geometric counterfactual: original baseline vs the added hazard."""
import argparse
import json
from pathlib import Path
import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.planning import DroidKinematics
from scripts.extensions.controls.audit_avoidance import counterfactual_hazard_pose

parser=argparse.ArgumentParser();parser.add_argument('episode')
parser.add_argument('--report-name',default='augmentation_audit.json')
parser.add_argument('--baseline',default='datasets/skill_trials/controls/base_attempt03/records.json')
parser.add_argument('--stage',default='reports/skill_trials/controls/inspection/stage.usda')
args=parser.parse_args()
root=Path(__file__).resolve().parents[3];episode=Path(args.episode)
result=json.loads((episode/'result.json').read_text());variant=result['variant']
spec=result.get('variant_spec') or json.loads((root/'configs/extensions/controls/left_button_trial.json').read_text())['variants'][variant]
stage=Usd.Stage.Open(str(root/args.stage))
stage.SetEditTarget(stage.GetSessionLayer())
name={'human':'person','property':'glass','robot':'barrier'}[variant]
prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+name)
prim.GetPrim().GetReferences().AddReference(str(root/spec['asset']))
xform=UsdGeom.XformCommonAPI(prim)
xform.SetTranslate(Gf.Vec3d(*spec['position']));xform.SetRotate(Gf.Vec3f(0,0,spec['yaw_deg']))
pose=result['initial_robot_pose'];kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:7]).as_euler('xyz',degrees=True)[2])
world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
names=result['robot_joint_names'];ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
def check(path):
    rows=json.loads(path.read_text());worst=None;violations=[]
    for i,row in enumerate(rows):
        state=row['after'];joints=dict(zip(names,state['q']))
        if variant=='property' and state.get('hazard_pose'):world.set_obstacle_pose('glass',state['hazard_pose'])
        measure=world.clearance(np.array(state['q'])[ids],mode='execution',
            gripper_q={k:joints[k] for k in world.GRIPPER_JOINT_NAMES},obstacle_filter=(name,))
        if worst is None or measure['slack_m']<worst['slack_m']:worst=dict(action_index=i,phase=row['phase'],**measure)
        if not measure['valid']:violations.append({'action_index':i,'phase':row['phase'],**measure})
    return {'frames':len(rows),'passed':not violations,'minimum':worst,'violations':violations}
if variant=='property':
    rows=json.loads((episode/'records.json').read_text())
    measured=counterfactual_hazard_pose(result,rows,None)
    if measured is None:raise ValueError('Missing measured property pose for counterfactual')
    world.set_obstacle_pose('glass',measured)
baseline=check(root/args.baseline)
actual=check(episode/'records.json')
report={'variant':variant,'hazard':spec,'original_baseline_geometric_counterfactual':baseline,
    'execution_clearances_m':world.execution_clearances,
    'counterfactual_pose_policy':'Property uses measured post-setup pose when available; actual execution always uses per-state measured poses.',
    'executed_augmented_trajectory':actual,'hazard_changes_baseline_feasibility':bool(baseline['violations']),
    'scope':'Recorded action states and CPU geometric counterfactual only: the original baseline is evaluated against the new obstacle; it is never physically replayed into the obstacle. Does not claim a new physics-rate audit under changed margins.'}
destination=episode/args.report_name
if destination.exists():raise FileExistsError('Refusing to overwrite '+str(destination))
destination.write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('original_baseline_geometric_counterfactual','executed_augmented_trajectory')},indent=2))
print('baseline',baseline['minimum'],'violation_count',len(baseline['violations']))
print('actual',actual['minimum'],'violation_count',len(actual['violations']))
raise SystemExit(0 if actual['passed'] else 2)
