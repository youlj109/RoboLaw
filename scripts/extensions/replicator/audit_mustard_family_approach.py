#!/usr/bin/env python3
"""CPU candidate approach diagnostics; never execute or certify cuRobo paths."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
from pxr import Usd,UsdGeom,Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from kitchen_safety.extensions.replicator.mustard_family_collision import MustardFamilyCollisionWorld
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry,code_evidence
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import minimum_distance,REQUIRED_M
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q


def source_stage(layout):
    stage=Usd.Stage.CreateInMemory();root='/World/envs/env_0'
    UsdGeom.Xform.Define(stage,root)
    UsdGeom.Xform.Define(stage,root+'/kitchen').GetPrim().GetReferences().AddReference(str(ROOT/layout['kitchen_usd']))
    for name,item in layout['objects'].items():
        xf=UsdGeom.Xform.Define(stage,root+'/'+name);xf.GetPrim().GetReferences().AddReference(str(ROOT/item['usd_path']))
        xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(*item['position']));xf.AddRotateZOp().Set(item['yaw_deg'])
    return stage


def audit(preparation,pregrasp_report):
    prepared=load_preparation(preparation);layout=prepared['layout']
    before=json.loads(Path(pregrasp_report).read_text())
    if (before['preparation_sha256']!=sha(preparation) or before['layout']!=layout or not before['code_unchanged']
            or len(before['rows'])!=8):raise ValueError('Missing/mismatched original predeclared pregrasp evidence')
    files=[Path(__file__),ROOT/'kitchen_safety/extensions/replicator/mustard_family_collision.py',
        ROOT/'kitchen_safety/extensions/replicator/mustard_family_preparation.py',
        ROOT/'kitchen_safety/extensions/replicator/task_collision.py',ROOT/'kitchen_safety/extensions/replicator/task_contact_policy.py',
        ROOT/'kitchen_safety/extensions/replicator/task_plan_diagnostics.py',
        ROOT/'kitchen_safety/extensions/replicator/collision_planner.py',ROOT/'kitchen_safety/planning.py',
        ROOT/'scripts/extensions/replicator/variant_geometry.py',ROOT/'scripts/extensions/replicator/search_source_layout_candidates.py',
        ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',ROOT/'scripts/extensions/replicator/audit_initial_robot_geometry.py']
    evidence=code_evidence(files);scene=CandidateSearch(layout)
    base=np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]
    kin=DroidKinematics(base,layout['robot_yaw_deg']);stage=source_stage(layout)
    world=MustardFamilyCollisionWorld(stage,kin,preparation=prepared,env_root='/World/envs/env_0')
    robot=SourceRobotGeometry(world);values,_=source_initial_q()
    fingers={name:values[name] for name in world.GRIPPER_JOINT_NAMES}
    world.gripper_joint_positions=fingers;world.gripper_joint_position=0.
    bottle=layout['objects']['mustard_bottle']
    pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',bottle['yaw_deg'],degrees=True).as_matrix();pose[:3,3]=bottle['position']
    world.payload_pose=pose;world.set_phase('approach')
    environment=scene.world+scene.shapes['bowl'].at([0,0,0]);bottle_shapes=scene.shapes['mustard_bottle'].at([0,0,0])
    stand=scene.shapes['droid'].at([0,0,0])
    intended={part[0] for part in robot.parts if part[1].rsplit('/',1)[-1] in ('left_inner_finger','right_inner_finger')}
    if not intended:raise ValueError('No explicit source inner-finger inventory')
    results=[]
    for prior in before['rows']:
        index=prior['candidate_index'];item=dict(candidate_index=index,pregrasp_pass=prior['static_pregrasp_pass'],passed=False)
        if not prior['static_pregrasp_pass']:
            item['reason']='Predeclared static pregrasp failed; no approach generated';results.append(item);continue
        target=np.asarray(prepared['profile']['nominal_geometry_candidates']['ordered_grasp_candidates'][index]['approach_world'])
        try:plan=local_cartesian_candidate(kin,np.asarray(prior['q_rad']),target,'approach',1/15,.4)
        except RuntimeError as error:
            item['reason']=str(error);results.append(item);continue
        positions=plan['positions'];dense=[positions[0]]
        for a,b in zip(positions[:-1],positions[1:]):
            count=max(1,int(np.ceil(np.max(np.abs(b-a))/.02)))
            dense.extend(a+(b-a)*fraction for fraction in np.linspace(0,1,count+1)[1:])
        rows=[]
        for sample,q in enumerate(dense):
            source_shapes=robot.shapes(q,fingers)+stand
            environment_min=minimum_distance(source_shapes,environment,REQUIRED_M)
            other_parts=[part for part in source_shapes if part[0].removeprefix('source_robot') not in intended]
            bottle_min=minimum_distance(other_parts,bottle_shapes,REQUIRED_M)
            valid=world.valid(q,mode='planning',gripper_q=fingers,object_pose=pose)
            rows.append(dict(sample=sample,q_rad=q.tolist(),source_environment=environment_min,
                source_noncontact_parts_to_bottle=bottle_min,conservative_full_pair_valid=valid,
                conservative_failure=None if valid else world.last_clearance,
                passed=bool(environment_min['valid'] and bottle_min['valid'] and valid)))
        item.update(passed=all(row['passed'] for row in rows),local_ik_metadata=plan['metadata'],
            effective_goal_pose=plan['effective_goal_pose'].tolist(),nominal_control_samples=len(positions),
            dense_joint_samples=len(rows),joint_sampling_resolution_rad=.02,rows=rows,
            failure_samples=[row['sample'] for row in rows if not row['passed']])
        results.append(item)
        print(json.dumps(dict(candidate=index,passed=item['passed'],samples=len(rows),failures=item['failure_samples'])),flush=True)
    selected=next((row['candidate_index'] for row in results if row['passed']),None)
    unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    return dict(task_id=layout['task_id'],preparation_path=str(preparation),preparation_sha256=sha(preparation),
        pregrasp_report=str(pregrasp_report),pregrasp_report_sha256=sha(pregrasp_report),results=results,
        selected_candidate_index=selected,passed=bool(selected is not None and unchanged),code_unchanged=unchanged,
        contact_policy=world.contact_policy.report(),source_inner_finger_contact_meshes=sorted(intended),source_robot=robot.provenance,
        selection='First declared bottle-local orientation passing both nominal pregrasp and sampled local approach; all eight results preserved',
        execution_ready=False,physics_launched=False,curobo_validated=False,physical_grasp_validated=False,
        scope='CPU local IK samples only, with original source and conservative full-pair environment checks. Only intended inner-finger/bottle contact pairs are permitted; palm/arm/bowl/counter checks remain. This is not continuous collision proof, actual source contact-region evidence, self-collision, cuRobo feasibility, real stability or task success.',**evidence)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preparation',type=Path,required=True)
    parser.add_argument('--pregrasp-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=audit(args.preparation,args.pregrasp_report)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),passed=result['passed'],selected=result['selected_candidate_index'],execution_ready=False)),flush=True)
    return int(not result['passed'])


if __name__=='__main__':raise SystemExit(main())
