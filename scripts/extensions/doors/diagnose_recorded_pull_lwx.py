"""Reconstruct the next rejected pull command from a retained actual state; CPU only."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_model import grasp_rotation,joint_position_from_openness,opening_command,door_target_pose
from kitchen_safety.planning import transform

p=argparse.ArgumentParser();p.add_argument('--episode',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
m=json.loads((a.episode/'metadata.json').read_text());result=json.loads((a.episode/'result.json').read_text());rows=json.loads((a.episode/'trajectory.json').read_text())
if result.get('failure')!='RuntimeError: cuRobo rejected pull segment':raise ValueError('Requires the retained pull-command rejection')
config=json.loads(Path(m['variant_config']).read_text());stage,kin,w,g=reconstruct(m,config,m['variant_spec']);cp=m['controller_parameters'];margin=cp.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin;obs=rows[-1]['next_obs'];q=np.asarray(obs['q'][:7]);lower,upper=g['limits_rad'];threshold=joint_position_from_openness(g['openness_threshold'],lower,upper);limit=max(lower+.01,threshold-.10) if lower<0. else min(upper-.01,threshold+.10);angle=opening_command(obs['door_q'],cp['pull_lead_rad'],limit,lower)
approach=Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).apply([1.,0.,0.]);rotation=grasp_rotation(approach,cp['grasp_roll_deg'],cp.get('grasp_pitch_deg',0.));target=door_target_pose(np.asarray(g['hinge']),np.asarray(g['axis']),np.asarray(m['selected_handle_initial']),rotation,angle,m['door_motion_reference_joint_rad']);command=kin.ik(target,q,attempts=1);delta=command-q;command=q+delta/max(1.,float(np.abs(delta).max())/.025)
pose=obs['door_body_pose'];change=transform(pose[:3],pose[3:])@np.linalg.inv(transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]))
for body in g['moving_bodies']:w.move_body(body,change)
w.allow_handle_contact=True;fingers={n:obs['q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};w.gripper_joint_position=float(obs['q'][m['robot_joint_names'].index('finger_joint')]);checks={mode:w.clearance(command,mode=mode,gripper_q=fingers) for mode in ['execution','planning']}
report={'scope':'Reconstructed command from the last retained actual state and controller parameters. CPU external geometry and joint limits only; no GPU self collision, exact failed-command observation, dynamics, execution or acceptance claim.','episode':str(a.episode),'trajectory_sha256':sha(a.episode/'trajectory.json'),'metadata_sha256':sha(a.episode/'metadata.json'),'result_sha256':sha(a.episode/'result.json'),'implementation_sha256':sha(__file__),'physics_replay':False,'unsafe_path_executed':False,'observed_door_q':obs['door_q'],'requested_angle_rad':angle,'target_pose':target.tolist(),'command':command.tolist(),'maximum_joint_step_rad':float(np.abs(command-q).max()),'joint_limit_slack_rad':float(np.minimum(command-kin.limits[:,0],kin.limits[:,1]-command).min()),'external_checks':checks,'complete':True};a.output.write_text(json.dumps(report,indent=2)+'\n');print('RECONSTRUCTED_PULL',report['joint_limit_slack_rad'],checks)
