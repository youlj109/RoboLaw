import json,numpy as np,itertools
from pathlib import Path
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform
p=Path("datasets/doors/lwx_C07_release_retry07_20261024/C07_base_attempt01");m=json.load(open(p/"metadata.json"));stage,k,w,g=reconstruct(m,json.load(open(m["variant_config"])),None);samples=json.load(open(p/"physics.json"))["samples"];release=[s for s in samples if s["phase"]=="release_handle"];before=next(s for s in reversed(samples) if s["phase"]!="release_handle");q0=np.array(before["state"]["q"][:7]);T0=transform(m["initial_door_pose"][:3],m["initial_door_pose"][3:]);start=k.fk(q0);w.allow_handle_contact=True;results=[]
for j5,j6,j7 in itertools.product([-.1,0.,.1],[-.1,-.2,-.3],[0.]):
 deltaq=np.array([0,0,0,0,j5,j6,j7]);worst=None
 for sample in release:
  state=sample["state"];fraction=max(0.,min(1.,(sample["action_index"]-release[0]["action_index"]+1-1.2)/20));q=q0+deltaq*fraction;pose=state["door_body_pose"];delta=transform(pose[:3],pose[3:])@np.linalg.inv(T0)
  for body in g["moving_bodies"]:w.move_body(body,delta)
  fingers={n:state["q"][m["robot_joint_names"].index(n)] for n in w.GRIPPER_JOINT_NAMES};w.gripper_joint_position=fingers["finger_joint"];detail=dict(w.clearance(q,mode="execution",gripper_q=fingers))
  if worst is None or detail["slack_m"]<worst["slack_m"]:worst=detail
 item=dict(joint_delta_rad=deltaq.tolist(),worst=worst,final_tcp_delta_m=(k.fk(q0+deltaq)[:3,3]-start[:3,3]).tolist(),limit_valid=bool(np.all(q0+deltaq>k.limits[:,0]+.005) and np.all(q0+deltaq<k.limits[:,1]-.005)));results.append(item);print(json.dumps(item),flush=True)
Path("reports/doors/lwx_C07_release_retry07_20261024/C07_joint_release_diagnosis01.json").write_text(json.dumps(dict(scope="CPU screening only at failed recorded poses, with estimated 1.2 action tracking delay. Does not predict altered dynamics or later hold safety.",physics_replay=False,results=results),indent=2))
