#!/usr/bin/env python3
"""CPU audit of fixed schedule and measured release boundary; no execution."""
import argparse
from pathlib import Path
import numpy as np
from scripts.extensions.replicator.review_bowl_supported_episode import read, write, sha, require, ROOT


def audit(episode):
    from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
    from scripts.extensions.replicator.review_task_episode import contact_window
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    ep = Path(episode)
    meta, result, contacts = [read(ep/n) for n in ('physics_metadata.json', 'result.json', 'grasp_contacts.json')]
    data = dict(np.load(ep/'physics_states.npz', allow_pickle=False))
    trajectory = dict(np.load(ep/'trajectory.npz', allow_pickle=False))
    plans = result['plans']; lower = next(p for p in plans if p['phase'] == 'lower')
    retreat = next(p for p in plans if p['phase'] == 'retreat')
    release_ids = np.flatnonzero(data['phase'] == 'release')
    final_ids = np.flatnonzero(data['phase'] == 'final_settle')
    step = int(lower['release_boundary']['physics_step'])
    index = int(np.flatnonzero(data['physics_step'] == step)[0])
    world, _, _ = build_world(meta['source_layout'])
    floor = next(x for x in world.objects if x[0] == '/World/envs/env_0/kitchen/Sink/BasinBottom')
    pose = pose_matrix(data['task_object_states'][index, meta['task_object_names'].index('bowl'), :7])
    vertices = world.payload_geometry.vertices@pose[:3,:3].T+pose[:3,3]
    low, high = vertices.min(0), vertices.max(0)
    source_center = (low+high)/2
    declared_center = (floor[2][:2]+floor[3][:2])/2 + np.array([.05, -.1])
    xy_error = float(np.linalg.norm(source_center[:2]-declared_center))
    gap = float(low[2]-floor[3][2])
    triangle = world.fcl.CollisionObject(world.payload_triangle_model,world.fcl.Transform(pose[:3,:3],pose[:3,3]))
    exact = world._exact_measurement((0.,'payload/bowl',floor[0],triangle,floor[1],.002))
    contact = contact_window(data, meta, contacts, 'bowl', result['grasp_target']['contact_region_local_m'], 'lower')
    plan_index = plans.index(lower)+1
    path_file = ep/f'plan_{plan_index:02d}_lower.npz'
    path = dict(np.load(path_file, allow_pickle=False))
    commanded = trajectory['actions'][trajectory['phase'] == 'lower', :7]
    core = path['positions'][1:]
    # The captured collector explicitly casts each action to float32 before
    # env.step (bowl_supported_collect.py step()); compare exact encoded values,
    # not a newly fitted numeric tolerance against pre-encoding float64 plans.
    encoded = core.astype(np.float32)
    full_path = commanded.dtype == np.float32 and len(commanded) >= len(core) and np.array_equal(commanded[:len(core)], encoded)
    hold = commanded[len(core):]
    retreat_state = int(np.flatnonzero(data['physics_step'] == retreat['start_physics_step'])[0])
    actual_tcp = pose_matrix(data['tcp_pose_w'][retreat_state])
    retreat_target = actual_tcp.copy(); retreat_target[2,3] += .15
    physical_dt, action_dt = meta['physics_dt_s'], meta['action_dt_s']
    checks = dict(original_sampling=physical_dt == 1/120 and action_dt == 1/15,
        complete_original_lower_path=full_path,
        only_bounded_identical_lower_hold=len(hold) <= 5 and (not len(hold) or np.array_equal(hold, np.tile(encoded[-1], (len(hold),1)))),
        lower_state_is_actual_action_boundary=step in trajectory['after_physics_step'].tolist(),
        release_follows_lower_next_physical_step=int(data['physics_step'][release_ids[0]]) == step+1,
        original_lower_2mm=exact['valid'], original_gap_window=.003 <= gap <= .005,
        original_xy_error=xy_error <= .005,
        full_source_xy=bool(np.all(low[:2] >= floor[2][:2]) and np.all(high[:2] <= floor[3][:2])),
        last10_lower_bilateral=contact['passed'] and [x['physics_step'] for x in contact['steps']] == list(range(step-9,step+1)),
        release_exact_2seconds=len(release_ids) == 240 and len(np.flatnonzero(trajectory['phase'] == 'release')) == 30,
        release_continuous=bool(np.all(np.diff(data['physics_step'][release_ids]) == 1)),
        release_commands_open=bool(np.all(trajectory['actions'][trajectory['phase'] == 'release', -1] == 0)),
        retreat_exact_original_15cm=np.array_equal(retreat_target, retreat['target_tcp']),
        retreat_without_attachment=retreat['measured_planning_attachment'] is None,
        final_settle_exact_4seconds=len(final_ids) == 480 and len(np.flatnonzero(trajectory['phase'] == 'final_settle')) == 60,
        final_settle_continuous=bool(np.all(np.diff(data['physics_step'][final_ids]) == 1)),
        final_commands_open=bool(np.all(trajectory['actions'][trajectory['phase'] == 'final_settle', -1] == 0)))
    return dict(passed=all(checks.values()), accepted=False, checks=checks,
        boundary_physics_step=step, source_gap_m=gap, original_source_floor_measurement=exact,
        original_xy_error_m=xy_error, original_landing_center_xy_m=declared_center.tolist(),
        actual_source_aabb_center_xy_m=source_center[:2].tolist(), bilateral_lower_contact=contact,
        lower_path_command_count=len(core), lower_identical_hold_command_count=len(hold),
        action_encoding='Original collector float32 cast, compared exactly; no numeric tolerance',
        release_physics_samples=len(release_ids), final_settle_physics_samples=len(final_ids),
        input_sha256={n:sha(ep/n) for n in ('physics_states.npz','physics_metadata.json','trajectory.npz',
            'result.json','grasp_contacts.json',path_file.name)}, review_source_sha256=sha(__file__),
        scope='Predeclared fixed schedule and observed lower release boundary independently recomputed; no threshold changes or successful task inference.')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();require(not args.output.exists(),'Never overwrite existing audit')
    result=audit(args.episode);write(args.output,result);print(result['checks'])
    raise SystemExit(0 if result['passed'] else 2)
