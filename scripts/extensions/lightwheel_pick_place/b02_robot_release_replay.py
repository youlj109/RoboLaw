"""Candidate filter using actual failed-release states, never physical replay."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.exact_primitives import static_collision_shapes,minimum_distance
from paired_planning import build_world
root=Path('datasets/skill_trials/lightwheel_pick_place');base=root/'base_lgd_b02_20261024T072827Z';failed=root/'robot_lgd_b02_review_20261025T102903Z'
r=json.load(open(base/'result.json'));b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);variants=json.load(open('configs/extensions/lightwheel_pick_place/variants_lemon_b02_robot_v2.json'));world,stage=build_world(base,kin,'robot',variants,task_spec=episode_spec(r));hazards=static_collision_shapes(stage,'/World/envs/env_0/barrier');report={'scope':'Offline actual-state candidate filter; no unsafe physical replay and no new rollout acceptance','position':variants['robot']['position']}
for label,ep in [('baseline',base),('previous_robot',failed)]:
 rows=json.load(open(ep/'full_world_audit.json'))['measurements'];indices=sorted(set(range(0,len(rows),16))|set(range(max(0,len(rows)-500),len(rows))));checks=[]
 for i in indices:
  row=rows[i];args=(row['q_arm'],('lemon',));kw=dict(object_pose=row['lime_pose'],mode='execution',gripper_q=row['gripper_q'],obstacle_filter=['barrier']);conservative=world.clearance(*args,**kw);exact=minimum_distance(world.shapes(np.asarray(row['q_arm']),row['gripper_q'],np.asarray(row['lime_pose'])),hazards,.01);checks.append({'index':i,'phase':row['phase'],'conservative':conservative,'exact':exact})
 report[label]={'samples':len(checks),'minimum_conservative':min(checks,key=lambda x:x['conservative']['slack_m']),'minimum_exact':min(checks,key=lambda x:x['exact']['slack_m']),'conservative_violations':sum(not x['conservative']['valid'] for x in checks),'exact_violations':sum(not x['exact']['valid'] for x in checks)}
 print(label,json.dumps(report[label]),flush=True)
p=Path('reports/skill_trials/lightwheel_pick_place/b02_review_20261025/robot_release_replay_v2.json');p.write_text(json.dumps(report,indent=2)+'\n')
