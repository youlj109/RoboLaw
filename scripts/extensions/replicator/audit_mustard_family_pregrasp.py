#!/usr/bin/env python3
"""CPU static pregrasp IK/FCL for a prepared original mustard layout.

Retains every predeclared bottle-local orientation. No contact exemptions,
object masks, cuRobo/Kit launch, path, physical grasp or success claim.
"""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,ROBOT_USD
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry,code_evidence
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q,near_failures
from scripts.extensions.replicator.audit_fixed_base import fixed_colliders,ASSEMBLY
from scripts.extensions.replicator.audit_source_layout_geometry import minimum_distance,REQUIRED_M
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch


def audit(preparation_path):
    prepared=load_preparation(preparation_path);layout=prepared['layout']
    files=[Path(__file__),ROOT/'kitchen_safety/extensions/replicator/mustard_family_preparation.py',
        ROOT/'kitchen_safety/extensions/replicator/mustard_family_background.py',
        ROOT/'kitchen_safety/extensions/replicator/background_physics.py',
        ROOT/'scripts/extensions/replicator/variant_geometry.py',
        ROOT/'scripts/extensions/replicator/search_source_layout_candidates.py',
        ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_initial_robot_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_fixed_base.py',
        ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/geometry_audit.py',
        ROOT/'kitchen_safety/extensions/replicator/collision_planner.py']
    evidence=code_evidence(files)
    scene=CandidateSearch(layout)
    base=np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]
    kin=DroidKinematics(base,layout['robot_yaw_deg'])
    world=CollisionWorld(Usd.Stage.CreateInMemory(),kin);robot=SourceRobotGeometry(world)
    values,initial=source_initial_q()
    start=np.array([values[f'panda_joint{i}'] for i in range(1,8)])
    fingers={name:values[name] for name in world.GRIPPER_JOINT_NAMES}
    stand=scene.shapes['droid'].at([0,0,0])
    fixed=fixed_colliders(np.r_[base,Rotation.from_euler('z',layout['robot_yaw_deg'],degrees=True).as_quat()])
    fixed_rows=[(info['path'],shape,*np.asarray(info['bounds'])) for shape,info in fixed]
    obstacles=scene.world+[part for name in layout['objects'] for part in scene.shapes[name].at([0,0,0])]
    rows=[]
    for index,candidate in enumerate(prepared['profile']['nominal_geometry_candidates']['ordered_grasp_candidates']):
        target=np.asarray(candidate['pregrasp_world'])
        row=dict(candidate_index=index,bottle_local_tilt_deg=candidate['bottle_local_tilt_deg'],
            bottle_local_yaw_deg=candidate['bottle_local_yaw_deg'],target_world=target.tolist(),static_pregrasp_pass=False)
        try:q=kin.ik(target,start,attempts=12)
        except RuntimeError as error:
            row.update(ik_passed=False,reason=str(error));rows.append(row)
            print(json.dumps(dict(index=index,passed=False,reason=str(error))),flush=True);continue
        tcp=kin.fk(q);error=float(np.linalg.norm(tcp[:3,3]-target[:3,3]))
        angle=float(Rotation.from_matrix(tcp[:3,:3]@target[:3,:3].T).magnitude())
        ik_ok=bool(error<=.002 and angle<=np.deg2rad(2.) and np.all(q>=kin.limits[:,0]) and np.all(q<=kin.limits[:,1]))
        source_shapes=robot.shapes(q,fingers)+stand
        proxy_shapes=world.shapes(q,gripper_q=fingers)+fixed_rows
        source_min=minimum_distance(source_shapes,obstacles,REQUIRED_M)
        proxy_min=minimum_distance(proxy_shapes,obstacles,REQUIRED_M)
        row.update(ik_passed=ik_ok,q_rad=q.tolist(),actual_tcp_world=tcp.tolist(),
            tcp_position_error_m=error,tcp_rotation_error_deg=float(np.rad2deg(angle)),
            source_robot_and_stand=source_min,conservative_robot_and_stand=proxy_min,
            source_failures=near_failures(source_shapes,obstacles),conservative_failures=near_failures(proxy_shapes,obstacles),
            static_pregrasp_pass=bool(ik_ok and source_min['valid'] and proxy_min['valid']))
        rows.append(row)
        print(json.dumps(dict(index=index,passed=row['static_pregrasp_pass'],source_min_m=source_min['distance_m'],proxy_min_m=proxy_min['distance_m'])),flush=True)
    selected=next((row['candidate_index'] for row in rows if row['static_pregrasp_pass']),None)
    unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    return dict(task_id=layout['task_id'],preparation_path=str(preparation_path),preparation_sha256=sha(preparation_path),
        layout=layout,required_clearance_m=REQUIRED_M,initial_joint_source=initial,
        selected_candidate_index=selected,selection='First passing orientation in preparation declaration; all eight attempts retained',
        static_pregrasp_candidate_found=selected is not None,passed=bool(selected is not None and unchanged),rows=rows,
        source_robot=robot.provenance,source_robot_sha256=sha(ROBOT_USD),source_stand_sha256=sha(ASSEMBLY),
        geometry_layer_sha256={str(Path(layer.realPath)):sha(layer.realPath) for stage in scene.stages
            for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()},
        code_unchanged=unchanged,execution_ready=False,physics_launched=False,accepted=False,
        pending=['Robot self-collision','Continuous approach from original initial q','Selective physical bottle contact',
                 'Actual settled pose adaptation','cuRobo paths','Lift/release/source task predicate','Neutral/mobility and full physics collection'],
        scope='Nominal authored-pose, open-finger static pregrasp. Full source and conservative robot+stand versus all original kitchen/floor/task-object geometry, strict original 2mm with no contact masks. No source-mesh containment, PhysX cooking/contact-offset, self-collision, interpolated path or real grasp certificate.',
        **evidence)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preparation',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    report=audit(args.preparation)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(task=report['task_id'],passed=report['passed'],selected=report['selected_candidate_index'],execution_ready=False,output=str(args.output))),flush=True)
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
