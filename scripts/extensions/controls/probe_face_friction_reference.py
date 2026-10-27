"""One fixed fingertip-circle candidate, checked kinematically without physics."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.temperature_execution import (
    face_friction_pose, pregrasp_connection, knob_retreat_poses)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tilt-deg',type=float,default=45.)
    p.add_argument('--lead-deg',type=float,default=2.)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config-output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists() or args.config_output.exists():
        raise FileExistsError('Reference evidence is immutable')
    source=Path('configs/extensions/controls/visual_20261027/08_oven_property_r05.json')
    config=json.loads(source.read_text())
    rotation=(Rotation.from_euler('y',args.tilt_deg,degrees=True)*
              Rotation.from_euler('x',-90,degrees=True)).as_matrix()
    config.update(grasp_rotation_world=rotation.tolist(),pregrasp_rotation_world=rotation.tolist(),
        pregrasp_tcp_world_m=[5.17,-3.22609,1.32588],knob_contact_mode='front_face_friction',
        friction_radius_m=.02,friction_preload_n=1.5,friction_start_x_offset_m=-.02,
        friction_max_axial_advance_m=.025,friction_lead_deg=args.lead_deg,
        property_physics_recipe='weighted_reading_lamp_v1')
    base=config['robot_pose_xyzw']
    kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:]).as_euler('xyz',degrees=True)[2])
    q=np.array([-1.1143192649,1.0469874423,1.7097610837,-.9823019496,
                -1.1704641944,1.9693056651,-2.8972990878])
    start=np.eye(4);start[:3,:3]=rotation;start[:3,3]=config['pregrasp_tcp_world_m']
    report=dict(scope='Nominal IK only; no physical success, collision or contact claim.',
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                layout_modified=False,status='failed',solved_samples=0)
    positions=[]
    def solve(phase,pose):
        nonlocal q
        report.update(phase=phase,requested_tcp=pose.tolist())
        q=kin.ik(pose,q,attempts=1)
        positions.append(q.tolist());report['solved_samples']+=1
    try:
        solve('pregrasp',start)
        config['prepress_planning']['ik_branch_seed']=q.tolist()
        contact=face_friction_pose(config,0,0)
        for pose in pregrasp_connection(start,contact[:3,3],rotation):solve('approach',pose)
        for advance in np.arange(.0005,.02501,.0005):solve('load_bound',face_friction_pose(config,0,advance))
        for angle in range(1,136):solve('turn',face_friction_pose(config,angle,.025))
        terminal=face_friction_pose(config,135,.025)
        for offset in ([-.03,0.,0.],[0.,0.,.03],[-.03,0.,.03]):
            for pose in knob_retreat_poses(terminal,offset):solve('retreat',pose)
            terminal=pose.copy()
        report['status']='nominal_reference_solved_not_physically_validated'
    except RuntimeError as exc:
        report['error']=str(exc)
    report['last_solved_q']=q.tolist()
    for path,data in ((args.output,report),(args.config_output,config)):
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(data,indent=2)+'\n')
    np.save(args.output.with_suffix('.npy'),np.asarray(positions))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
