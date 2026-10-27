"""One bounded cuRobo approach query. Never execute or certify the result."""
import argparse
import hashlib
import json
import time
import traceback
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.extensions.controls.planning import button_prepress_pose
from kitchen_safety.extensions.controls.temperature_execution import temperature_pregrasp_pose


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('config', 'baseline', 'stage', 'output'):
        p.add_argument('--' + name, required=True, type=Path)
    p.add_argument('--variant', required=True, choices=['human', 'property', 'robot'])
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--refine-from', type=Path, help='Refine a failed saved endpoint without layout edits')
    p.add_argument('--single-policy', type=Path, help='One modification, one IK solve, no candidate search')
    p.add_argument('--round-number', type=int, default=2)
    p.add_argument('--pose-reference', type=Path, help='Saved candidate pose retained while changing one parameter')
    p.add_argument('--target-kind', choices=['original_precontact','actual_grasp_arrival'], default='original_precontact')
    a = p.parse_args()
    if a.single_policy and not a.refine_from:
        p.error('--single-policy requires --refine-from')
    if (a.output/'result.json').exists():
        raise FileExistsError('Existing planning result: '+str(a.output))
    a.output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[3]
    started = time.monotonic()
    report = dict(status='failed', motiongen_calls=0, seed=a.seed, config=str(a.config),
                  scope='Approach only: start to prepress/pregrasp. No physical execution, '
                        'contact manipulation, independent collision audit or stability test.',
                  safety_validated=False, task_success_validated=False,
                  attempts_limit=1, config_sha256=hashlib.sha256(a.config.read_bytes()).hexdigest())
    preview = None
    try:
        config = json.loads(a.config.read_text())
        baseline = json.loads((a.baseline/'result.json').read_text())
        rows = json.loads((a.baseline/'records.json').read_text())
        temperature = 'turn_oven_temperature_knob' in config['task_id']
        phase = 'pregrasp' if temperature else 'prepress'
        state = next(row['before'] for row in rows if row['phase'] == phase)
        names = baseline['robot_joint_names']
        q = np.asarray(state['q'])[[names.index('panda_joint'+str(i)) for i in range(1, 8)]]
        grip = state['q'][names.index('finger_joint')]
        pose = config['robot_pose_xyzw']
        kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:]).as_euler('xyz', degrees=True)[2])
        if baseline['task_id'] != config['task_id']:
            raise ValueError('Baseline/config task mismatch')
        params = baseline['control_parameters']
        if temperature:
            target = temperature_pregrasp_pose(config)
        else:
            target = button_prepress_pose(config, params['roll_deg'], params['tcp_x_offset_m'], params['tcp_z_offset_m'])
        report['target_kind']=a.target_kind
        if a.target_kind=='actual_grasp_arrival':
            if not temperature:raise ValueError('Grasp-arrival target is only defined for the temperature task')
            report['source_precontact_tcp']=target.tolist()
            target[:3,3]=config['grasp_tcp_world_m']
            report['scope']='Nominal open-gripper arrival at configured grasp TCP. No physical grasp, turn or safety validation.'
        report.update(phase=phase, baseline=str(a.baseline), start_q=q.tolist(),
                      gripper_joint_position=grip, requested_tcp=target.tolist(), stage=str(a.stage))
        stage = Usd.Stage.CreateInMemory()
        stage.GetRootLayer().subLayerPaths.append(str(a.stage.resolve()))
        key = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}[a.variant]
        spec = config['variants'][a.variant]
        asset_file = root/spec['asset']
        protected_hashes = {str(a.config): report['config_sha256'],
                            str(asset_file): hashlib.sha256(asset_file.read_bytes()).hexdigest()}
        prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/' + key)
        prim.GetPrim().GetReferences().AddReference(str(root/spec['asset']))
        prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
        prim.AddRotateZOp().Set(spec['yaw_deg'])
        world = ControlsCollisionWorld(stage, kin, env_root='/World/envs/env_0')
        world.gripper_joint_position = grip
        center = (kin.fk(q)[:3, 3] + target[:3, 3]) / 2
        boxes = []
        for name, _, lo, hi in world.objects:
            hazard = '/' + key + '/' in name
            # Display crop only, never a planner-world or safety filter.
            if hazard or (np.linalg.norm((lo+hi)/2-center) < 1.2 and len(boxes) < 220):
                boxes.append(dict(name=name, low=lo.tolist(), high=hi.tolist(), hazard=hazard))

        def joints(x):
            return [t[:3, 3].tolist() for t in kin.links(x).values()] + [kin.fk(x)[:3, 3].tolist()]

        preview = dict(scope='Kinematic line/AABB preview, not physics playback or collision evidence.',
                       target=target[:3, 3].tolist(), boxes=boxes, frames=[joints(q)],
                       tcp=[kin.fk(q)[:3, 3].tolist()], q=[q.tolist()], dt=baseline['dt'], phase=phase)
        planner = CuroboPlanner(world, kin, dt=baseline['dt'], seed=a.seed, max_joint_speed=.4)
        if a.refine_from:
            from endpoint_candidates import select_endpoint
            original = json.loads(a.refine_from.read_text())
            if original['status'] != 'failed' or original['config_sha256'] != report['config_sha256']:
                raise ValueError('Refinement requires the failed result and its unchanged layout config')
            if not np.array_equal(np.asarray(original['start_q']), q):
                raise ValueError('Refinement cannot replace the saved start configuration')
            report.update(refinement_source=str(a.refine_from), planning_round=a.round_number,
                          layout_modified=False, actual_contact_target_changed=False,
                          original_requested_tcp=target.tolist(), protected_hashes=protected_hashes)
            planner._prepare_path_query((), 0 if temperature else 1, phase)
            ik_initial_seed=np.asarray(original['goal_q'])
            if a.pose_reference:
                reference=json.loads(a.pose_reference.read_text())
                if reference['config_sha256'] != report['config_sha256']:
                    raise ValueError('Pose reference layout differs')
                reference_pose=np.asarray(reference['requested_tcp'],dtype=float)
                allowed_positions=[target[:3,3]]
                if a.target_kind=='actual_grasp_arrival':allowed_positions.append(np.asarray(config['pregrasp_tcp_world_m']))
                if reference_pose.shape!=(4,4) or not any(np.allclose(reference_pose[:3,3],v,atol=1e-9,rtol=0) for v in allowed_positions):
                    raise ValueError('Pose reference may not move the original precontact TCP position')
                target=reference_pose.copy()
                ik_initial_seed=np.asarray(reference.get('ik_seed_reference',reference.get('ik_initial_seed',original['goal_q'])))
                report.update(pose_reference=str(a.pose_reference), reference_requested_tcp=target.tolist(),
                              requires_alignment_before_contact=reference.get('requires_alignment_before_contact',False))
            if a.single_policy:
                from endpoint_candidates import single_policy_endpoint, single_policy_parameters
                policy = json.loads(a.single_policy.read_text())
                report.update(single_change=policy, candidate_count=1, ik_calls_limit=1)
                intended_pose, intended_seed, intended_policy = single_policy_parameters(target, ik_initial_seed, phase, policy)
                if a.target_kind=='actual_grasp_arrival' and not np.allclose(intended_pose[:3,3],config['grasp_tcp_world_m'],atol=1e-9,rtol=0):
                    raise ValueError('Nominal grasp-arrival query may not replace the configured actual grasp point')
                report.update(requested_tcp=intended_pose.tolist(), attempted_endpoint_policy=intended_policy)
                report['ik_initial_seed']=intended_seed.tolist()
                single_started = time.monotonic()
                try:
                    selected = single_policy_endpoint(kin, target, ik_initial_seed, phase, policy)
                except RuntimeError:
                    report['failure_code'] = 'SINGLE_POLICY_IK_FAILED'
                    raise
                search = dict(candidates=[dict(index=0, policy=selected['policy'],
                    requested_tcp=selected['requested_tcp'].tolist(), goal_q=selected['goal_q'].tolist(),
                    status='one_ik_solved_no_path_audit')], seconds=time.monotonic()-single_started, budget_exhausted=False,
                    single_modification=True, ik_calls=1)
            else:
                selected, search = select_endpoint(kin, planner, q, original['goal_q'], target, phase, seed=a.seed)
            save_json(a.output/'endpoint_candidates.json', search)
            report['candidate_count'] = len(search['candidates'])
            report['endpoint_selection_seconds'] = search['seconds']
            if selected is None:
                report['failure_code'] = 'NO_VALID_ENDPOINT_IN_BOUNDED_CANDIDATES'
                raise RuntimeError('No eligible pre-contact endpoint in fixed candidate/time budget; no path query')
            goal_q, target = selected['goal_q'], selected['requested_tcp']
            report.update(selected_endpoint_policy=selected['policy'],
                          selected_candidate_index=selected['candidate_index'], requested_tcp=target.tolist())
            report['requires_alignment_before_contact'] = bool(
                report.get('requires_alignment_before_contact', False) or
                selected['policy'].get('requires_alignment_before_contact', False))
            preview['target'] = target[:3, 3].tolist()
        else:
            goal_q = kin.ik(target, np.asarray(config['prepress_planning']['ik_branch_seed']), attempts=1)
        report['goal_q'] = goal_q.tolist()
        if temperature:
            nominal=np.asarray(config.get('original_pregrasp_tcp_world_m',config['pregrasp_tcp_world_m']))
            report.update(configured_pregrasp_position_reached=bool(np.allclose(
                target[:3,3], config['pregrasp_tcp_world_m'], atol=1e-9, rtol=0)),
                original_pregrasp_position_reached=bool(np.allclose(target[:3,3],nominal,atol=1e-9,rtol=0)),
                nominal_grasp_position_reached=bool(np.allclose(
                target[:3,3], config['grasp_tcp_world_m'], atol=1e-9, rtol=0)),
                remaining_nominal_grasp_distance_m=float(np.linalg.norm(
                target[:3,3]-np.asarray(config['grasp_tcp_world_m']))))
            if report.get('selected_endpoint_policy', {}).get('auxiliary_pregrasp_height_m'):
                report['scope']='Adapted auxiliary pregrasp endpoint only. Original grasp point unchanged; no connection, physical grasp, turn or safety validation.'
            if not report['original_pregrasp_position_reached']:
                report['scope']='Adapted configured pregrasp only; not the original fixed endpoint, contact connection, task execution or safety validation.'
                report['requires_alignment_before_contact']=True
                report.setdefault('selected_endpoint_policy',dict(
                    name='retained_recorded_controller_reference',
                    auxiliary_pregrasp_height_m=float(target[2,3]-nominal[2]),
                    requires_alignment_before_contact=True))
        planner._prepare_path_query((), 0 if temperature else 1, phase)
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        joint_names = ['panda_joint'+str(i) for i in range(1, 8)]
        start = JointState.from_position(planner.args.to_device(q[None]), joint_names=joint_names)
        goal = JointState.from_position(planner.args.to_device(goal_q[None]), joint_names=joint_names)
        report['motiongen_calls'] = 1
        result = planner.motion_gen.plan_single_js(start, goal, MotionGenPlanConfig(
            max_attempts=1, timeout=15., enable_graph=True,
            enable_graph_attempt=None, enable_finetune_trajopt=True, time_dilation_factor=.8))
        planner.torch.cuda.synchronize()
        report['planner_status'] = str(result.status)
        report['valid_query'] = str(result.valid_query)
        if not bool(result.success.all()):
            report['failure_code'] = str(result.status)
            raise RuntimeError('cuRobo returned ' + str(result.status) + '; valid_query=' + str(result.valid_query))
        raw = result.get_interpolated_plan().position.detach().cpu().numpy()
        path = planner.retime_path(raw, source_dt=float(result.interpolation_dt))
        np.save(a.output/'positions.npy', path)
        np.savetxt(a.output/'positions.csv', np.column_stack([np.arange(len(path))*baseline['dt'], path]),
                   delimiter=',', header='time_s,'+','.join(joint_names), comments='')
        report.update(status='generated', samples=len(path), dt=baseline['dt'],
                      duration_s=(len(path)-1)*baseline['dt'], collision_geometry=planner.collision_geometry,
                      note='Planner output only. No validate_path/FCL sweep, counterfactual, physics or task test.')
        preview.update(frames=[joints(x) for x in path], tcp=[kin.fk(x)[:3, 3].tolist() for x in path],
                       q=path.tolist(), status='generated')
    except Exception as exc:
        report['status'] = 'failed'
        report['error'] = str(exc)
        report.setdefault('failure_code', type(exc).__name__)
        traceback.print_exc()
    finally:
        if 'protected_hashes' in locals():
            report['layout_files_unchanged'] = all(
                hashlib.sha256(Path(name).read_bytes()).hexdigest() == value
                for name, value in protected_hashes.items())
            if not report['layout_files_unchanged']:
                report.update(status='failed', failure_code='CONCURRENT_LAYOUT_CHANGE')
        report['elapsed_s'] = time.monotonic()-started
        if preview is not None:
            preview['status'] = report['status']
            save_json(a.output/'preview.json', preview)
        save_json(a.output/'result.json', report)
        print('QUICK_PLAN_RESULT', json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
