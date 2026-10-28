"""CPU candidate audit: source FCL for approach, stroke, arc, release, retreat.

No simulator starts; selected joint vectors are hypotheses, never trajectory
evidence. Native readback and CUDA checks remain required in water_collect.
"""
import argparse
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.hard_execution_v1.water_audit import build_world
from kitchen_safety.extensions.replicator.hard_execution_v1.water_mechanism import PIVOT,BODY,sha,ROOT
from kitchen_safety.extensions.replicator.hard_execution_v1.water_control import centered_grasp_pose as grasp_pose, VALVE_GRASP_PROFILE
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output",type=Path,required=True)
args=parser.parse_args()
if args.output.exists():parser.error("Refuse to overwrite candidate evidence")
w,k,s=build_world();q=np.radians([0,-36,0,-144,0,108,0]);rows=[]
def check(phase,angle,offset,grip):
 global q
 w.set_phase(phase);w.set_obstacle_pose(BODY,np.r_[PIVOT,Rotation.from_euler('z',180+angle,degrees=True).as_quat()]);w.gripper_joint_positions=np.array([1,1,-1,1,-1,-1])*grip;w.flowing=angle>2.
 target=grasp_pose(angle,offset)
 try:q=k.ik(target,q,attempts=2);valid=w.valid(q);row=dict(phase=phase,angle=float(angle),offset=float(offset),grip=float(grip),passed=valid,violation=w.last_clearance,flow_geometry=w.last_flow_clearance,q=q.tolist())
 except Exception as error:row=dict(phase=phase,angle=float(angle),passed=False,error=repr(error))
 rows.append(row)
 if not row['passed']: print(json.dumps(row),flush=True)
for offset in np.linspace(.1,0,51):check('valve_approach' if offset<.1 else 'valve_pregrasp',55,offset,0)
contact_q=VALVE_GRASP_PROFILE['cpu_expected_first_inner_finger_touch_rad']
stroke_max=VALVE_GRASP_PROFILE['cpu_pinch_stroke_audit_max_rad']
for grip in np.linspace(0,stroke_max,31):check('valve_grasp',55,0,grip)
for angle in np.linspace(55,0,111):check('valve_turn',angle,0,contact_q)
for grip in np.linspace(stroke_max,0,31):check('valve_release',0,0,grip)
for offset in np.linspace(0,.1,51):check('valve_retreat',0,offset,0)
result=dict(passed=all(r['passed'] for r in rows),samples=len(rows),failures=sum(not r['passed'] for r in rows),physics_launched=False,cuda_launched=False,grasp_profile=VALVE_GRASP_PROFILE,scope='Candidate CPU IK/source FCL only, including closing to predicted bilateral touch + .01 rad. Overclosure beyond this is not certified; runtime still checks every actual six-joint state without clamping. Actual contact grip q and cuRobo self/world constraints remain mandatory at execution',sources={str(p):sha(ROOT/p) for p in ['kitchen_safety/extensions/replicator/hard_execution_v1/water_control.py','kitchen_safety/extensions/replicator/hard_execution_v1/water_mechanism.py']},rows=rows)
args.output.parent.mkdir(parents=True,exist_ok=True)
with args.output.open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
