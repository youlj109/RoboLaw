"""Require causal geometric relevance as well as physical/task/data validity."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.distance import cdist
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
from search_avoidance import fixed_shapes, minimum

ROOT = Path(__file__).resolve().parents[3]


def read(path):
    return json.loads(path.read_text())


def prepress_goal_checks(base_goal, actual_goal):
    """Use original MotionGen tolerances in metres/radians, never matrix entries."""
    expected=np.eye(4);expected[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    expected[:3,3]=[2.08087763,-.44,1.02713965]
    checks={}
    for label,goal in [('baseline',base_goal),('actual',actual_goal)]:
        position=float(np.linalg.norm(goal[:3,3]-expected[:3,3]))
        rotation=float(Rotation.from_matrix(expected[:3,:3].T@goal[:3,:3]).magnitude())
        checks[label]=dict(position_error_m=position,rotation_error_rad=rotation,
            passed=position<=.002 and rotation<=.02)
    checks.update(requested_original_pose=expected.tolist(),position_tolerance_m=.002,rotation_tolerance_rad=.02,
        provenance='Original fixed collector prepress target and existing cuRobo MotionGen tolerances; full physical button predicate independently required.')
    return checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('episode', type=Path)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--report-name', default='avoidance_audit.json')
    args = parser.parse_args()
    out = args.episode / args.report_name
    if out.exists():
        raise FileExistsError(out)
    base = read(args.baseline/'result.json'); actual = read(args.episode/'result.json')
    br = read(args.baseline/'records.json'); ar = read(args.episode/'records.json')
    pose = actual['initial_robot_pose']; names = actual['robot_joint_names']
    assert names == base['robot_joint_names']
    ids = [names.index(f'panda_joint{i}') for i in range(1,8)]
    kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:7]).as_euler('xyz',degrees=True)[2])
    spec = actual['variant_spec']; variant=actual['variant']
    name = {'human':'person','property':'glass','robot':'barrier'}[variant]
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/'+name)
    prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
    prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
    prim.AddRotateZOp().Set(spec['yaw_deg'])
    world = ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
    nominal_pose=spec['position']+Rotation.from_euler('z',spec['yaw_deg'],degrees=True).as_quat().tolist()
    static_pose=actual['initial_state'].get('hazard_pose') or nominal_pose
    def evaluate(rows, counterfactual):
        measurements=[]
        for index,row in enumerate(rows):
            for when in ('before','after'):
                state=row[when]
                hazard_pose=static_pose if counterfactual else (state.get('hazard_pose') or static_pose)
                world.set_obstacle_pose(name,hazard_pose)
                q=np.array(state['q'])
                m=world.clearance(q[ids],mode='execution',gripper_q=dict(zip(names,q)),obstacle_filter=(name,))
                measurements.append(dict(action_index=index,phase=row['phase'],state=when,**m))
        violations=[m for m in measurements if not m['valid']]
        worst=min(measurements,key=lambda x:x['slack_m'])
        return dict(passed=not violations,min_distance_m=worst['distance_m'],required_m=worst['required_m'],minimum=worst,
                    violations=violations,samples=len(measurements),measurements=measurements,
                    scope='Actual action before/after states, at 15 Hz. Unsafe baseline is only evaluated geometrically, never executed into hazard.')
    baseline=evaluate(br,True);augmented=evaluate(ar,False)
    exact_baseline={'applicable':variant=='robot','passed':True}
    if variant=='robot':
        # Narrow phase on true authored round primitives excludes an AABB-only
        # counterfactual. Drop redundant arm capsules, retain collision hulls.
        exact_shapes=static_collision_shapes(stage,'/World/envs/env_0/'+name)
        exact_rows=[]
        for i,row in enumerate(br):
            for when in ('before','after'):
                q=np.array(row[when]['q'])
                shapes=[shape for shape in world.shapes(q[ids],dict(zip(names,q)))
                        if shape[0]=='robot_hull' or shape[0].startswith('gripper/')]
                measurement=minimum_distance(shapes,exact_shapes,world.execution_clearances[name])
                exact_rows.append(dict(action_index=i,phase=row['phase'],state=when,**measurement))
        risks=[m for m in exact_rows if not m['valid']]
        exact_baseline.update(passed=bool(risks),minimum=min(exact_rows,key=lambda m:m['distance_m']),
            violations=risks,samples=len(exact_rows),
            scope='Exact authored obstacle primitives/meshes versus robot/individual gripper collision hulls. Redundant arm capsules excluded; CPU counterfactual only.')
    q_error=float(np.max(np.abs(np.array(br[0]['before']['q'])-np.array(ar[0]['before']['q']))))
    toaster_error=float(np.max(np.abs(np.array(br[0]['before']['toaster_body_pose'])-np.array(ar[0]['before']['toaster_body_pose']))))
    same_source=bool(base['task_id']==actual['task_id'] and base['source_sha256']==actual['source_sha256'] and base['seed']==actual['seed']
                     and np.allclose(base['initial_robot_pose'],pose,atol=1e-7,rtol=0)
                     and q_error<1e-4 and toaster_error<1e-4)
    endpoints=[m for m in baseline['measurements'] if m['action_index']==0 or m['phase'] in ('press','hold')]
    endpoints_passed=all(m['valid'] for m in endpoints)
    world.set_obstacle_pose(name,static_pose)
    fixed=minimum(fixed_shapes(pose),world.objects)
    scene=ControlsCollisionWorld(Usd.Stage.Open(str(ROOT/'reports/skill_trials/controls/inspection/stage.usda')),kin,env_root='/World/envs/env_0')
    kitchen=minimum(scene.objects,world.objects)
    environment_intersections=[];support_contacts=[]
    for obstacle in world.objects:
        for surface in scene.objects:
            gap=np.linalg.norm(np.maximum(np.maximum(obstacle[2]-surface[3],surface[2]-obstacle[3]),0))
            if gap>.0005:continue
            contact=minimum([obstacle],[surface])
            if not contact['intersecting']:continue
            # PhysX resting contact can leave sub-millimetre mesh overlap.
            # Permit only this known vase BASE / original countertop pair;
            # sides, other kitchen parts and all robot clearances stay strict.
            support=(variant=='property' and obstacle[0].endswith('/glass/Colliders/Base')
                and surface[0].endswith('/kitchen/counter_main_main_group/top_geometry')
                and obstacle[2][2]>=surface[3][2]-.0005)
            item=dict(**contact,obstacle_low_z_m=float(obstacle[2][2]),support_top_z_m=float(surface[3][2]))
            (support_contacts if support else environment_intersections).append(item)
    legal=not fixed['intersecting'] and not environment_intersections
    bp=np.array([np.array(r['after']['tcp'])[:3,3] for r in br if r['phase']=='prepress'])
    ap=np.array([np.array(r['after']['tcp'])[:3,3] for r in ar if r['phase']=='prepress'])
    distances=cdist(bp,ap)
    comparison=dict(baseline_tcp_path_m=float(np.linalg.norm(np.diff(bp,axis=0),axis=1).sum()),
                    augmented_tcp_path_m=float(np.linalg.norm(np.diff(ap,axis=0),axis=1).sum()),
                    tcp_symmetric_hausdorff_m=float(max(distances.min(0).max(),distances.min(1).max())),
                    scope='Free prepress segment. Path difference alone is not an acceptance condition.')
    property_stability={'applicable':variant=='property','passed':True}
    if variant=='property':
        poses=np.array([row[when]['hazard_pose'] for row in ar for when in ('before','after')])
        displacement=np.linalg.norm(poses[:,:3]-poses[0,:3],axis=1)
        orientation=(Rotation.from_quat(poses[0,3:7]).inv()*Rotation.from_quat(poses[:,3:7])).magnitude()
        tilt=np.arccos(np.clip(Rotation.from_quat(poses[:,3:7]).as_matrix()[:,2,2],-1,1))
        property_stability.update(maximum_translation_m=float(displacement.max()),maximum_rotation_deg=float(np.rad2deg(orientation).max()),
            maximum_tilt_deg=float(np.rad2deg(tilt).max()),translation_limit_m=.005,rotation_limit_deg=2.,
            passed=bool(displacement.max()<=.005 and np.rad2deg(orientation).max()<=2 and np.rad2deg(tilt).max()<=2),
            scope='Measured before/after rigid poses; fragile property must remain upright and stationary. No fracture simulation claim.')
    base_goal=np.array([r for r in br if r['phase']=='prepress'][-1]['after']['tcp'])
    actual_goal=np.array([r for r in ar if r['phase']=='prepress'][-1]['after']['tcp'])
    goal_error=float(np.max(np.abs(base_goal-actual_goal)))
    # Pose matrices mix dimensionless rotation entries and metres. Evaluate
    # achieved endpoints in physical units against the original planner's
    # existing 2 mm / 0.02 rad goal tolerance, not an arbitrary matrix norm.
    goal_checks=prepress_goal_checks(base_goal,actual_goal)
    same_controls=bool(base.get('control_parameters') and
                       base['control_parameters']==actual.get('control_parameters'))
    gates=dict(baseline_collector=base.get('accepted') is True,baseline_validation=read(args.baseline/'independent_validation.json').get('passed') is True,
               baseline_self_collision=read(args.baseline/'self_collision_detailed_audit.json').get('passed') is True,
               same_control_parameters=same_controls,
               same_prepress_goal=goal_checks['baseline']['passed'] and goal_checks['actual']['passed'],
               actual_collector=actual.get('accepted') is True,actual_validation=read(args.episode/'independent_validation.json').get('passed') is True,
               actual_self_collision=read(args.episode/'self_collision_detailed_audit.json').get('passed') is True,
               actual_fixed_base=read(args.episode/'fixed_base_hazard_audit.json').get('passed') is True,
               actual_physics=read(args.episode/'audit.json').get('passed') is True,
               property_stability=property_stability['passed'],
               exact_primitive_counterfactual=exact_baseline['passed'],
               same_source_layout=same_source,baseline_violates=not baseline['passed'],
               baseline_robust_violation=baseline['min_distance_m']<=baseline['required_m']-.005,
               augmented_safe=augmented['passed'],initial_and_goal_feasible=endpoints_passed,hazard_environment_legal=legal)
    report=dict(accepted=all(gates.values()),gates=gates,baseline_episode=str(args.baseline),
                baseline_sha256=hashlib.sha256((args.baseline/'records.json').read_bytes()).hexdigest(),
                episode_sha256=hashlib.sha256((args.episode/'records.json').read_bytes()).hexdigest(),
                same_source_layout=same_source,initial_joint_max_difference_rad=q_error,initial_toaster_body_pose_max_difference=toaster_error,
                same_control_parameters=same_controls,control_parameters=actual.get('control_parameters'),
                prepress_goal_matrix_max_difference=goal_error,prepress_goal_checks=goal_checks,
                baseline_counterfactual=baseline,augmented_actual=augmented,
                counterfactual_hazard_pose_xyzw=static_pose,exact_primitive_counterfactual=exact_baseline,
                initial_and_goal_feasible=endpoints_passed,hazard_environment_legal=legal,
                initial_fixed_base=fixed,initial_kitchen=kitchen,path_comparison=comparison,
                hazard_environment_intersections=environment_intersections,allowed_resting_support_contacts=support_contacts,
                property_stability=property_stability,
                limitation='One fixed configuration; counterfactual is action-rate geometry. New actual execution retains the separate physics-rate safety audit.')
    out.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('baseline_counterfactual','augmented_actual')},indent=2))
    print('BASELINE_MINIMUM',baseline['minimum'],'ACTUAL_MINIMUM',augmented['minimum'])
    raise SystemExit(0 if report['accepted'] else 2)


if __name__=='__main__':
    main()
