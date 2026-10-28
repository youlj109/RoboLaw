"""Local Cartesian candidate/evidence; no physical execution or relaxed gates.

The candidate generator preserves the public planner's 2 mm IK sampling,
0.15 rad branch gate, 0.4 rad/s (caller configured) speed limit, minimum
3 seconds, 5 mm / 2 degree column and 2 mm height bounds.
"""
from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation


def local_cartesian_candidate(kin, q_arm7, target, phase, dt, max_joint_speed):
    initial = kin.fk(q_arm7)
    goal = np.array(target, copy=True)
    goal[:3, :3] = initial[:3, :3]
    count = max(2, int(np.ceil(np.linalg.norm(goal[:3, 3]-initial[:3, 3])/.002))+1)
    path = [np.asarray(q_arm7).copy()]
    for fraction in np.linspace(0., 1., count)[1:]:
        tcp = initial.copy()
        tcp[:3, 3] = initial[:3, 3]+fraction*(goal[:3, 3]-initial[:3, 3])
        q = kin.ik(tcp, path[-1], attempts=1)
        if np.max(np.abs(q-path[-1])) > .15:
            raise RuntimeError(f"Local Cartesian IK switched joint branches during {phase}")
        path.append(q)
    path = np.asarray(path)
    segment_times = np.maximum(3./(len(path)-1),
        np.max(np.abs(np.diff(path, axis=0)), axis=1)/max_joint_speed)
    knot_times = np.r_[0., np.cumsum(segment_times)]
    times = np.minimum(np.arange(int(np.ceil(knot_times[-1]/dt))+1)*dt, knot_times[-1])
    positions = np.stack([np.interp(times, knot_times, path[:, j]) for j in range(7)], axis=-1)
    poses = np.array([kin.fk(q) for q in positions])
    xy = np.linalg.norm(poses[:, :2, 3]-goal[:2, 3], axis=1)
    rotation = Rotation.from_matrix(poses[:, :3, :3]@goal[:3, :3].T).magnitude()
    zlo, zhi = sorted([initial[2, 3], goal[2, 3]])
    if xy.max() > .005 or rotation.max() > np.deg2rad(2.):
        raise RuntimeError(f"Cartesian {phase} left the 5 mm / 2 degree vertical column")
    if poses[:, 2, 3].min() < zlo-.002 or poses[:, 2, 3].max() > zhi+.002:
        raise RuntimeError(f"Cartesian {phase} left the requested height interval")
    return dict(positions=positions, ik_waypoint_positions=path, poses=poses,
        effective_goal_pose=goal, dt=dt, metadata=dict(ik_waypoints=count,
            max_xy_deviation_m=float(xy.max()), max_orientation_deviation_deg=float(np.rad2deg(rotation.max())),
            minimum_tcp_z_m=float(poses[:, 2, 3].min()), maximum_tcp_z_m=float(poses[:, 2, 3].max()),
            start_joint_error_rad=float(np.max(np.abs(positions[0]-q_arm7)))))


def constraint_components(planner, positions):
    """Use the exact check_valid dynamics and constraints, preserving all gates."""
    solver = planner.motion_gen.ik_solver
    rollout = solver.rollout_fn
    q = planner.args.to_device(np.asarray(positions))
    state = rollout.dynamics_model.forward(rollout.start_state, q.unsqueeze(1))
    metrics = rollout.constraint_fn(state, use_batch_env=False)
    def values(tensor):
        return tensor.detach().cpu().numpy().reshape(len(positions), -1).max(axis=1)
    if not rollout.primitive_collision_constraint.enabled or not rollout.robot_self_collision_constraint.enabled:
        raise RuntimeError("Required world/self collision constraint is disabled")
    rows = dict(joint_bound=values(rollout.bound_constraint.forward(state.state_seq)),
        world_collision=values(rollout.primitive_collision_constraint.forward(state.robot_spheres, env_query_idx=None)),
        self_collision=values(rollout.robot_self_collision_constraint.forward(state.robot_spheres)))
    feasible = metrics.feasible.detach().cpu().numpy().reshape(-1).astype(bool)
    direct = solver.check_valid(q).detach().cpu().numpy().reshape(-1).astype(bool)
    if not np.array_equal(feasible, direct):
        raise RuntimeError("Diagnostic constraint decomposition differs from check_valid")
    if not all(np.isfinite(row).all() for row in rows.values()):
        raise RuntimeError("Nonfinite cuRobo constraint diagnostic")
    report = dict(valid=bool(feasible.all()), samples=len(positions),
        rejected_samples=np.flatnonzero(~feasible).tolist(),
        components={name: dict(costs=row.tolist(), maximum_cost=float(row.max()),
            nonzero_samples=np.flatnonzero(row != 0.).tolist()) for name, row in rows.items()},
        source="Exact pinned IK rollout dynamics/start_state and enabled original constraint functions; no tolerances changed")
    return report, state


def sphere_pair_diagnostics(planner, state, components):
    """Offline attribution only; never a planner result or execution certificate.

    Isolated-obstacle GPU queries identify which original collider contributes
    to the unchanged full-world rejection. Restore the original candidate mask
    after each diagnostic. No path is accepted from these isolation queries.
    """
    from curobo.geom.sdf.world import CollisionQueryBuffer
    rollout = planner.motion_gen.ik_solver.rollout_fn
    cost = rollout.primitive_collision_constraint
    checker = cost.world_coll_checker
    spheres = state.robot_spheres
    robot = rollout.dynamics_model.robot_model
    config = robot.kinematics_config
    name_by_index = dict(config.link_name_to_idx_map)
    reverse_names = {value: key for key, value in name_by_index.items()}
    sphere_links = [reverse_names[int(i)] for i in config.link_sphere_idx_map.detach().cpu().numpy()]
    cpu_spheres = spheres.detach().cpu().numpy()[:, 0]
    bad = components["components"]["world_collision"]["nonzero_samples"]
    masks = planner.candidate_contact_mask["object_switches"]
    enabled = [name for name in planner.obstacle_names if not planner._ignored(name, masks)]
    world_pairs = []
    if bad:
        # Candidate pruning uses exact padded sphere AABBs against source world
        # bounds. Mesh/cuboid activation distance is retained in the expansion.
        rotation, translation = planner.kin.base[:3, :3], planner.kin.base[:3, 3]
        xyz = cpu_spheres[bad, :, :3]@rotation.T+translation
        radius = cpu_spheres[bad, :, 3]
        activation = float(cost.activation_distance.max().item())
        candidates = []
        for name, _, low, high in planner.world.objects:
            if name not in enabled:
                continue
            gap = np.linalg.norm(np.maximum(np.maximum(np.asarray(low)-xyz, xyz-np.asarray(high)), 0.), axis=-1)
            if np.any((radius > 0.) & (gap <= radius+activation+1e-6)):
                candidates.append(name)
        selected = spheres[bad].contiguous()
        buffer = CollisionQueryBuffer()
        buffer.update_buffer_shape(selected.shape, cost.tensor_args, checker.collision_types)
        try:
            for name in enabled:
                checker.enable_obstacle(name, enable=False)
            for name in candidates:
                checker.enable_obstacle(name, enable=True)
                distances = cost.coll_check_fn(selected, buffer, cost.weight, env_query_idx=None,
                    activation_distance=cost.activation_distance, return_loss=cost.return_loss,
                    sum_collisions=cost.sum_collisions).detach().cpu().numpy()[:, 0]
                checker.enable_obstacle(name, enable=False)
                for local, sphere_id in np.argwhere(distances != 0.):
                    sample = bad[int(local)]
                    world_pairs.append(dict(sample=sample, obstacle=name, sphere_index=int(sphere_id),
                        link=sphere_links[int(sphere_id)], cost=float(distances[local, sphere_id]),
                        sphere_world_xyzr=[*(cpu_spheres[sample, sphere_id, :3]@rotation.T+translation),
                                           float(cpu_spheres[sample, sphere_id, 3])]))
        finally:
            for name in planner.obstacle_names:
                checker.enable_obstacle(name, enable=name in enabled)
    self_cfg = robot.get_self_collision_config()
    if self_cfg.experimental_kernel:
        pairs = self_cfg.thread_location.detach().cpu().numpy().reshape(-1, 2)
        pairs = pairs[(pairs >= 0).all(axis=1)]
    else:
        pairs = np.argwhere(np.triu(self_cfg.collision_matrix.detach().cpu().numpy().astype(bool), 1))
    offsets = self_cfg.offset.detach().cpu().numpy().reshape(-1)
    self_pairs = []
    for sample in components["components"]["self_collision"]["nonzero_samples"]:
        a, b = pairs[:, 0], pairs[:, 1]
        distance = np.linalg.norm(cpu_spheres[sample, a, :3]-cpu_spheres[sample, b, :3], axis=1)
        overlap = cpu_spheres[sample, a, 3]+cpu_spheres[sample, b, 3]+offsets[a]+offsets[b]-distance
        for index in np.flatnonzero(overlap > 0):
            first, second = map(int, pairs[index])
            self_pairs.append(dict(sample=sample, first_sphere=first, second_sphere=second,
                first_link=sphere_links[first], second_link=sphere_links[second], overlap_m=float(overlap[index])))
    return dict(world_sphere_pairs=world_pairs, self_sphere_pairs=self_pairs,
        sphere_links=sphere_links, world_sphere_buffer_m=.005,
        world_activation_distance_m=float(cost.activation_distance.max().item()),
        original_candidate_masks=deepcopy(masks),
        scope="Diagnostic isolated-obstacle cuRobo queries restored to full original mask; exact enabled self-pair sphere geometry. Never used to authorize execution."), cpu_spheres
