"""Declare one knob grasp orientation and solve its nominal reference once."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.temperature_execution import (
    pregrasp_connection, knob_turn_pose, temperature_grasp_rotation, knob_retreat_poses)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path('configs/extensions/controls/visual_20261027/08_oven_property_r05.json'))
    p.add_argument('--approach-result',type=Path,default=Path('reports/controls_planning_quick_20261027/round90_auxiliary_roll_height_20261027/round90/08_oven_temperature_property/result.json'))
    p.add_argument('--tilt-deg',type=float,default=50.)
    p.add_argument('--cone-start-deg',type=float,default=-67.5)
    p.add_argument('--roll-deg',type=float,default=-105.)
    p.add_argument('--retreat-rise-m',type=float,default=0.)
    p.add_argument('--config-output',type=Path,required=True)
    p.add_argument('--report-output',type=Path,required=True)
    args=p.parse_args()
    if args.config_output.exists() or args.report_output.exists():
        raise FileExistsError('Candidate outputs are immutable')
    if not all(np.isfinite([args.tilt_deg,args.cone_start_deg,args.roll_deg])):
        raise ValueError('Orientation parameters must be finite')
    if not 0 <= args.tilt_deg <= 60 or not -90 <= args.cone_start_deg <= 0:
        raise ValueError('Candidate cone parameters outside declared range')
    if not np.isfinite(args.retreat_rise_m) or not 0<=args.retreat_rise_m<=.08:
        raise ValueError('Retreat rise must be within 0 to 8 cm')
    config=json.loads(args.source.read_text())
    approach=json.loads(args.approach_result.read_text())
    source_hash=hashlib.sha256(args.source.read_bytes()).hexdigest()
    if source_hash != approach['config_sha256'] or approach['status']!='generated':
        raise ValueError('Approach/source mismatch')
    if (config['original_target_level'],config['original_reset_level'])!=(3,-1):
        raise ValueError('Original knob task must remain unchanged')
    rotation=(Rotation.from_euler('x',args.cone_start_deg,degrees=True).as_matrix() @
              Rotation.from_euler('y',args.tilt_deg,degrees=True).as_matrix() @
              Rotation.from_euler('x',args.roll_deg,degrees=True).as_matrix())
    config['grasp_rotation_world']=rotation.tolist()
    config['pregrasp_tcp_world_m']=[approach['requested_tcp'][i][3] for i in range(3)]
    config['pregrasp_roll_deg']=-165.
    config['retreat_offset_world_m']=[-.06,0.,args.retreat_rise_m]
    config['prepress_planning']['ik_branch_seed']=approach['goal_q']
    config['grasp_reference_candidate']=dict(source=str(args.source),source_sha256=source_hash,
        tilt_deg=args.tilt_deg,cone_start_deg=args.cone_start_deg,axial_roll_deg=args.roll_deg,
        retreat_rise_m=args.retreat_rise_m,layout_modified=False,actual_grasp_point_modified=False,physics_validated=False)
    args.config_output.parent.mkdir(parents=True,exist_ok=True)
    args.config_output.write_text(json.dumps(config,indent=2)+'\n')
    base=config['robot_pose_xyzw']
    kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:]).as_euler('xyz',degrees=True)[2])
    q=np.asarray(approach['goal_q'])
    start=np.asarray(approach['requested_tcp'])
    grasp_rotation=temperature_grasp_rotation(config)
    report=dict(status='failed',scope='One nominal kinematic reference, not a cuRobo path, '
        'collision audit, physical execution, grasp success or dataset.',
        layout_modified=False,task_goal_level=3,config=str(args.config_output),
        source_sha256=source_hash,ik_calls=0,motiongen_calls=0,solved_samples=0,
        grasp_rotation_world=grasp_rotation.tolist(),actual_grasp_point=config['grasp_tcp_world_m'],
        retreat_offset_world_m=config['retreat_offset_world_m'])
    started=time.monotonic()
    positions=[]

    def solve(phase,pose):
        nonlocal q
        report.update(phase=phase,requested_tcp=pose.tolist())
        report['ik_calls']+=1
        q=kin.ik(pose,q,attempts=1)
        positions.append(q.tolist())
        report['solved_samples']+=1

    try:
        for pose in pregrasp_connection(start,config['grasp_tcp_world_m'],grasp_rotation):
            solve('approach',pose)
        for angle in range(1,int(config['target_angle_interior_deg'])+1):
            solve('turn',knob_turn_pose(config['grasp_tcp_world_m'],grasp_rotation,angle))
        terminal=knob_turn_pose(config['grasp_tcp_world_m'],grasp_rotation,config['target_angle_interior_deg'])
        for pose in knob_retreat_poses(terminal,config['retreat_offset_world_m']):
            solve('retreat',pose)
        report['status']='nominal_reference_solved_not_physically_validated'
    except RuntimeError as exc:
        report.update(error=str(exc),last_solved_q=q.tolist())
    report['elapsed_s']=time.monotonic()-started
    report['source_unchanged']=hashlib.sha256(args.source.read_bytes()).hexdigest()==source_hash
    args.report_output.parent.mkdir(parents=True,exist_ok=True)
    args.report_output.write_text(json.dumps(report,indent=2)+'\n')
    if report['status']!='failed':
        np.save(args.report_output.with_suffix('.npy'),np.asarray(positions))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
