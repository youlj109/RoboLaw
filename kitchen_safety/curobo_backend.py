"""GPU cuRobo MotionGen adapter for the existing Arena collector.

The caller independently checks every returned segment with CollisionWorld.
cuRobo's sphere model and soft collision cost do not certify a safety margin.
"""
from copy import deepcopy
from collections import deque
from functools import wraps
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[1]


def ensure_warp_torch_compat():
    """Compatibility for pinned cuRobo, including Arena's native placement IK."""
    import warp as wp
    from types import SimpleNamespace
    if not hasattr(wp, 'torch'):
        wp.torch = SimpleNamespace(device_from_torch=wp.device_from_torch)


def _hierarchical_sample_indices(count):
    """Visit every dense-path index once, endpoints and large intervals first.

    This changes only rejection order: a legal path still checks every point.
    Starting with the interval ends and midpoints finds late-path collisions
    without walking a long, already retimed prefix first.
    """
    if count <= 0:
        return
    yield 0
    if count == 1:
        return
    yield count - 1
    pending = deque([(1, count - 2)])
    while pending:
        low, high = pending.popleft()
        if low > high:
            continue
        middle = (low + high) // 2
        yield middle
        pending.append((middle + 1, high))
        pending.append((low, middle - 1))


def _timed_context_operation(category):
    """Wall-clock accounting only; retain no query results or GPU tensors."""
    def decorate(method):
        @wraps(method)
        def measured(self, *args, **kwargs):
            started = time.perf_counter()
            try:
                return method(self, *args, **kwargs)
            finally:
                if not hasattr(self, 'context_timings'):
                    self.context_timings, self.context_calls = {}, {}
                self.context_timings[category] = self.context_timings.get(category, 0.) + time.perf_counter()-started
                self.context_calls[category] = self.context_calls.get(category, 0) + 1
        return measured
    return decorate


class CuroboPlanner:
    def __init__(self, world, kin, dt=1 / 15, seed=42, max_joint_speed=.4,
                 transport_object_padding_m=0., use_cuda_graph=False, warmup=False):
        constructor_started = time.perf_counter()
        self.context_timings, self.context_calls = {}, {}
        self.use_cuda_graph, self.seed = bool(use_cuda_graph), int(seed)
        self._warmed_up, self._context_reused = False, False
        self._query_signature = None
        import torch
        # Arena's pinned cuRobo uses Warp's old torch submodule. Warp 1.16
        # exposes the identical conversion at the package root.
        ensure_warp_torch_compat()
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.geom.types import Cuboid, Mesh, WorldConfig
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

        self.world, self.kin, self.dt = world, kin, float(dt)
        self.max_joint_speed = float(max_joint_speed)
        self.transport_object_padding_m = float(transport_object_padding_m)
        self.torch = torch
        self.args = TensorDeviceType()
        asset = ROOT / "assets/curobo/droid"
        self.robot_dict = yaml.safe_load((asset / "franka_robotiq_2f_85_zero_curobo.yml").read_text())["robot_cfg"]
        k = self.robot_dict["kinematics"]
        k["urdf_path"] = str(asset / "urdf/franka_robotiq_2f_85_zero.urdf")
        k["collision_sphere_buffer"] = .005
        self.T_ee_tcp = np.array(json.loads((asset / "tcp_calibration.json").read_text())["T_urdfEE_USDtcp"])
        self.T_B_W = np.linalg.inv(kin.base)
        meshes, boxes = [], []
        self.obstacle_names = []
        self._dynamic_box_names = set()
        padded_mesh_counts = {"glass": 0, "barrier": 0}
        if not hasattr(world, "meshes"):
            raise RuntimeError("CollisionWorld.meshes is required; refusing to replace the person with a whole-body AABB")
        for name, _fcl_object, lo, hi in world.objects:
            self.obstacle_names.append(name)
            hazard = next((key for key in padded_mesh_counts if world._matches(name, key)), None)
            if name in world.meshes and hazard is None:
                vertices, faces = world.meshes[name]
                vertices = np.asarray(vertices) @ self.T_B_W[:3, :3].T + self.T_B_W[:3, 3]
                meshes.append(Mesh(name=name, pose=[0., 0., 0., 1., 0., 0., 0.],
                                   vertices=vertices.tolist(), faces=np.asarray(faces).tolist()))
            else:
                # Reserve the same stand-off for primitive and convex-mesh
                # pieces of fragile props / fixtures. Each collider gets its
                # own padded box, preserving compound structure rather than
                # enclosing the entire object. Human and kitchen meshes keep
                # their triangles. These conservative planning boxes never
                # replace the original FCL or PhysX collision geometry.
                padding = world._margin(name, 'planning') if hazard is not None else 0.
                if hazard is not None:
                    self._dynamic_box_names.add(name)
                if name in world.meshes:
                    padded_mesh_counts[hazard] += 1
                pose, dims = self._planning_box(lo, hi, padding)
                boxes.append(Cuboid(name=name, pose=pose, dims=dims))
        self.collision_geometry = {
            "hazard_mesh_policy": "per_collider_world_aabb_plus_planning_clearance",
            "hazard_mesh_box_counts": padded_mesh_counts,
            "retained_triangle_mesh_count": len(meshes),
            "planning_box_count": len(boxes),
            "fcl_and_physx_geometry": "original",
        }
        cfg = MotionGenConfig.load_from_robot_config(
            deepcopy(self.robot_dict), WorldConfig(mesh=meshes, cuboid=boxes), self.args,
            interpolation_dt=self.dt, interpolation_steps=5000,
            use_cuda_graph=self.use_cuda_graph, collision_checker_type=CollisionCheckerType.MESH,
            collision_cache={"mesh": max(len(meshes), 1), "obb": max(len(boxes), 1)},
            collision_activation_distance=.04, num_ik_seeds=32, num_trajopt_seeds=4,
            trajopt_tsteps=40, position_threshold=.002, rotation_threshold=.02,
            self_collision_check=True, self_collision_opt=True,
            # The collector consumes the selected path and CPU last_plan
            # summary only. Retaining IK/trajopt/finetune debug results keeps
            # their GPU tensors alive separately for every environment.
            store_debug_in_result=False,
            ik_seed=int(seed), graph_seed=int(seed),
        )
        self.motion_gen = MotionGen(cfg)
        self._last_grip = None
        self.last_plan = None
        self.context_timings['constructor_s'] = time.perf_counter()-constructor_started
        if warmup:
            self.warmup()
        print("CUROBO_READY", len(meshes), "meshes", len(boxes), "boxes",
              "hazard_mesh_boxes", padded_mesh_counts, flush=True)

    def context_report(self):
        return {'use_cuda_graph': self.use_cuda_graph, 'seed': self.seed,
                'warmup_completed': self._warmed_up, 'phase_reused': self._context_reused,
                'timings_seconds': dict(self.context_timings), 'calls': dict(self.context_calls),
                'timing_scope': 'constructor excludes warmup; query includes pre-query setup and CPU work; phase_reset includes any world_update. These nested values are not additive.',
                'seed_semantics': 'Configured seed retained; explicit phase reset also resets both finetune solvers. Reuse does not promise bitwise equality with fresh contexts.'}

    def _reset_all_seeds(self):
        self.motion_gen.reset(reset_seed=True)
        # MotionGen.reset_seed omits these two solvers in the pinned release.
        self.motion_gen.finetune_trajopt_solver.reset_seed()
        self.motion_gen.finetune_js_trajopt_solver.reset_seed()

    def warmup(self):
        """Capture/warm single-pose, single-joint and PRM paths once."""
        if self._warmed_up:
            return
        started = time.perf_counter()
        try:
            self.motion_gen.warmup(enable_graph=True, batch=None, warmup_js_trajopt=True)
            self._reset_all_seeds()
            self._warmed_up = True
        finally:
            self.context_timings['warmup_s'] = self.context_timings.get('warmup_s', 0.) + time.perf_counter()-started
            self.context_calls['warmup_s'] = self.context_calls.get('warmup_s', 0) + 1

    @_timed_context_operation('phase_reset_s')
    def reset_for_new_phase(self, ignore_names=(), grip_binary=0, phase=None, hazard_key=None):
        """Reuse fixed topology with measured geometry, then invalidate PRM.

        False means the hazard cannot be updated in place; rebuild before any
        query. Update errors propagate. CUDA operation graphs remain allocated.
        """
        if hazard_key is not None and not self.update_dynamic_obstacle(hazard_key, reset=False):
            return False
        self._last_grip = None  # Apply the measured gripper just as a new context would.
        self._context_reused = True
        self._query_signature = None
        self._prepare_path_query(ignore_names, grip_binary, phase)
        self._reset_all_seeds()
        return True

    @staticmethod
    def _pose_list(T):
        q = Rotation.from_matrix(T[:3, :3]).as_quat()[[3, 0, 1, 2]]
        return np.r_[T[:3, 3], q].tolist()

    def _planning_box(self, lo, hi, padding):
        """Express a padded world-axis AABB in the robot-base frame."""
        lo, hi = np.asarray(lo), np.asarray(hi)
        T = self.T_B_W.copy()
        T[:3, 3] = self.T_B_W[:3, :3] @ ((lo + hi) / 2) + self.T_B_W[:3, 3]
        return self._pose_list(T), (np.maximum(hi - lo, 1e-4) + 2*padding).tolist()

    @_timed_context_operation('world_update_s')
    def update_dynamic_obstacle(self, key, *, reset=True):
        """Refresh existing hazard boxes from the caller's updated FCL world.

        Only the original glass/barrier Cuboids are supported. Return False
        without changing the planner when the selection includes another
        geometry type or changed collider topology; the caller must rebuild.
        Errors while updating cuRobo propagate instead of accepting a stale
        world. Neither obstacle masks nor the attached bottle are changed.
        """
        selected = [(name, lo, hi) for name, _, lo, hi in self.world.objects
                    if self.world._matches(name, key)]
        expected = {name for name in self._dynamic_box_names if self.world._matches(name, key)}
        if not selected or {name for name, _, _ in selected} != expected:
            return False
        checker = self.motion_gen.world_coll_checker
        updates = []
        for name, lo, hi in selected:
            cpu_box = checker.world_model.get_obstacle(name)
            if cpu_box is None:
                return False
            pose, dims = self._planning_box(lo, hi, self.world._margin(name, 'planning'))
            updates.append((name, pose, dims, cpu_box))
        from curobo.types.math import Pose
        for name, pose, dims, cpu_box in updates:
            checker.update_obb_pose(name=name, w_obj_pose=Pose.from_list(pose, self.args))
            checker.update_obb_dims(self.args.to_device(dims), name=name)
            cpu_box.pose, cpu_box.dims = pose, dims
        # Individual collider updates do not invalidate cached PRM roadmaps.
        # Match the old fresh-planner seed policy at this update boundary.
        if reset:
            if getattr(self, '_context_reused', False) or getattr(self, 'use_cuda_graph', False):
                self._reset_all_seeds()
            else:
                self.motion_gen.reset(reset_seed=True)
        return True

    @staticmethod
    def _ignored(name, ignore_names):
        return any("/" + v + "/" in name or name.endswith("/" + v) for v in ignore_names)

    def _sync_gripper_and_attachment(self, grip_binary, object_padding_m=0.):
        # The six USD/URDF joints have mimic signs +,+,-,+,-,-.
        # While carrying the mustard bottle, the fingers stop around 0.38 rad.
        # A live measured value can be provided by the caller on the collision world.
        measured = getattr(self.world, "gripper_joint_position", None)
        grip = float(measured if measured is not None else (.38 if grip_binary else 0.))
        grip = float(np.clip(grip, 0., np.pi / 4))
        if self._last_grip is None or abs(grip - self._last_grip) > .002:
            locked = dict(self.robot_dict["kinematics"]["lock_joints"])
            for name, sign in {
                "finger_joint": 1., "right_outer_knuckle_joint": 1.,
                "left_inner_finger_joint": -1., "right_inner_finger_joint": 1.,
                "left_inner_finger_knuckle_joint": -1., "right_inner_finger_knuckle_joint": -1.,
            }.items():
                locked[name] = sign * grip
            self.motion_gen.update_locked_joints(locked, deepcopy(self.robot_dict))
            self._last_grip = grip
        if self.world.attachment is None:
            self.motion_gen.detach_spheres_from_robot()
            return
        # A 3x3x10 sphere approximation of the carried bottle, in its true TCP-relative pose.
        # Cap radii at the top/bottom so resting support contact is not made into a
        # fictitious penetration by the spherical approximation. The caller checks
        # the complete bottle box independently with FCL at every output sample.
        centers = np.array(np.meshgrid(np.linspace(-.0167, .0167, 3),
                                       np.linspace(-.022, .022, 3),
                                       np.linspace(-.07, .07, 10), indexing="ij")).reshape(3, -1).T
        # A 3 mm contact band is left to the exact FCL bottle/support check:
        # the physics-settled bottle root is 0.9993 m above a 0.92 m counter.
        # Using its nominal 0.081 m half-height here invents a start collision.
        radii = np.minimum(.019, .077 - np.abs(centers[:, 2]))
        T = self.T_ee_tcp @ self.world.attachment
        centers = centers @ T[:3, :3].T + T[:3, 3]
        spheres = np.zeros((100, 4), dtype=np.float32)
        spheres[:, 3] = -100.
        spheres[:len(centers), :3] = centers
        spheres[:len(centers), 3] = radii + object_padding_m
        self.motion_gen.attach_spheres_to_robot(sphere_tensor=self.args.to_device(spheres))

    @staticmethod
    def _path_array(positions):
        path = np.asarray(positions, dtype=float)
        if path.ndim != 2 or path.shape[1] != 7 or len(path) == 0:
            raise ValueError("Expected a nonempty (N, 7) arm joint path")
        if not np.isfinite(path).all():
            raise ValueError("Arm joint path contains nonfinite values")
        return path

    def _prepare_path_query(self, ignore_names, grip_binary, phase):
        for name in self.obstacle_names:
            self.motion_gen.world_coll_checker.enable_obstacle(
                name, enable=not self._ignored(name, ignore_names))
        padding = (self.transport_object_padding_m if self.world.attachment is not None
                   and phase not in ('lift', 'lower', 'retreat') else 0.)
        self._sync_gripper_and_attachment(grip_binary, padding)
        if getattr(self, '_context_reused', False) or getattr(self, 'use_cuda_graph', False):
            signature = (tuple(sorted(ignore_names)), self._last_grip, float(padding),
                         None if self.world.attachment is None else
                         tuple(np.asarray(self.world.attachment).ravel()))
            if self._query_signature is not None and signature != self._query_signature:
                # A prior roadmap may describe open fingers or an unattached
                # bottle. Keep CUDA graphs, but never keep those PRM edges.
                self.motion_gen.reset(reset_seed=False)
                self.context_calls['query_geometry_roadmap_reset'] = self.context_calls.get('query_geometry_roadmap_reset', 0) + 1
            self._query_signature = signature
        return padding

    def retime_path(self, positions, source_dt=None, min_duration=0.):
        """Return an (N, 7) path at the collector period and original speed cap.

        With source_dt, retain uniform source timing and only slow it down.
        Without it, allocate time by each segment's largest joint displacement.
        The endpoints are retained exactly. Validate the returned segments after
        any geometric correction; time resampling is not a collision check.
        """
        path = self._path_array(positions)
        if not np.isfinite(min_duration) or min_duration < 0:
            raise ValueError("min_duration must be finite and nonnegative")
        if self.dt <= 0 or self.max_joint_speed <= 0:
            raise ValueError("Action period and joint speed must be positive")
        if len(path) == 1:
            path = np.repeat(path, 2, axis=0)
        deltas = np.max(np.abs(np.diff(path, axis=0)), axis=1)
        if source_dt is not None:
            source_dt = float(source_dt)
            if not np.isfinite(source_dt) or source_dt <= 0:
                raise ValueError("source_dt must be finite and positive")
            scale = max(1., float(deltas.max()) / source_dt / self.max_joint_speed,
                        min_duration / ((len(path)-1)*source_dt))
            knot_times = np.arange(len(path)) * source_dt * scale
        else:
            segment_times = np.maximum.reduce([
                np.full(len(path)-1, self.dt),
                np.full(len(path)-1, min_duration/(len(path)-1)),
                deltas/self.max_joint_speed,
            ])
            knot_times = np.r_[0., np.cumsum(segment_times)]
        times = np.minimum(np.arange(int(np.ceil(knot_times[-1]/self.dt))+1)*self.dt,
                           knot_times[-1])
        retimed = np.stack([np.interp(times, knot_times, path[:, j]) for j in range(7)], axis=-1)
        retimed[0], retimed[-1] = path[0], path[-1]
        return retimed

    def retime_path_smooth(self, positions, max_joint_speed=.2, max_joint_acceleration=.4,
                           min_segment_duration=.4):
        """Traverse each geometric edge with zero endpoint velocity/acceleration.

        Sparse RRT/guide vertices are geometric corners. Quintic rest-to-rest
        timing follows those same edges without instantaneous velocity changes.
        The returned path still requires the ordinary complete geometry checks.
        """
        path = self._path_array(positions)
        speed = min(float(max_joint_speed), self.max_joint_speed)
        acceleration = float(max_joint_acceleration)
        if min(speed, acceleration, min_segment_duration) <= 0:
            raise ValueError('Smooth timing limits must be positive')
        result = [path[0].copy()]
        for start, goal in zip(path[:-1], path[1:]):
            distance = float(np.max(np.abs(goal-start)))
            if distance < 1e-12:
                continue
            # Exact maxima of the quintic blend's first and second derivatives.
            duration = max(min_segment_duration, 1.875*distance/speed,
                           np.sqrt((10./np.sqrt(3.))*distance/acceleration))
            count = max(1, int(np.ceil(duration/self.dt)))
            u = np.arange(1, count+1, dtype=float)/count
            blend = 10*u**3 - 15*u**4 + 6*u**5
            result.extend(start + fraction*(goal-start) for fraction in blend)
            result[-1] = goal.copy()
        if len(result) == 1:
            result.append(path[-1].copy())
        return np.asarray(result)

    @_timed_context_operation('path_validation_s')
    def validate_path(self, positions, ignore_names=(), grip_binary=0, phase=None,
                      resolution=.02):
        """Check dense path samples against cuRobo and the original FCL rules.

        cuRobo checks joint limits, world geometry and self collision; FCL adds
        the existing independent clearance and carried-object tilt constraints.
        Returns diagnostics with ``valid``; the caller must reject false before
        execution. A failed cuRobo check or one failed FCL sample is sufficient
        to reject: remaining FCL points are explicitly reported as unchecked.
        A true result still requires both checks at every original dense sample.
        The phase selects the same bottle padding as plan_pose.
        """
        started = time.perf_counter()
        path = self._path_array(positions)
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        padding = self._prepare_path_query(ignore_names, grip_binary, phase)
        samples, edges, fractions = [path[0]], [0], [0.]
        for edge, (a, b) in enumerate(zip(path[:-1], path[1:])):
            count = max(2, int(np.ceil(np.max(np.abs(b-a))/resolution))+1)
            for fraction in np.linspace(0., 1., count)[1:]:
                samples.append(a+(b-a)*fraction)
                edges.append(edge)
                fractions.append(float(fraction))
        samples = np.asarray(samples)
        feasible = self.motion_gen.ik_solver.check_valid(self.args.to_device(samples))
        feasible = feasible.detach().cpu().numpy().astype(bool).reshape(-1)
        if len(feasible) != len(samples):
            raise RuntimeError("cuRobo returned an unexpected path-validity shape")
        curobo_failures = [
            {"sample": int(i), "edge": edges[i], "fraction": fractions[i]}
            for i in np.flatnonzero(~feasible)
        ]
        fcl_failures = []
        fcl_samples_checked = 0
        # A known-invalid cuRobo path cannot be accepted by the independent
        # checker. Avoid spending minutes collecting redundant FCL failures.
        # Otherwise visit the exact same dense points in a hierarchical order;
        # the first real failure rejects, while acceptance exhausts them all.
        indices = () if curobo_failures else _hierarchical_sample_indices(len(samples))
        for i in indices:
            fcl_samples_checked += 1
            if not self.world.valid(samples[i], ignore_names):
                fcl_failures.append({
                    "sample": i, "edge": edges[i], "fraction": fractions[i],
                    "collision": deepcopy(self.world.last_collision),
                    "measurement": deepcopy(self.world.last_clearance),
                })
                break
        validation_complete = fcl_samples_checked == len(samples)
        return {
            "valid": not curobo_failures and not fcl_failures,
            "samples": len(samples), "input_waypoints": len(path),
            "resolution_rad": float(resolution),
            "curobo_failures": curobo_failures, "fcl_failures": fcl_failures,
            "curobo_samples_checked": len(samples),
            "fcl_samples_checked": fcl_samples_checked,
            "fcl_samples_unchecked": len(samples) - fcl_samples_checked,
            "validation_complete": validation_complete,
            "early_rejection": ("curobo_constraint" if curobo_failures else
                                "independent_geometry" if not validation_complete else None),
            "sample_order": "cuRobo all original dense samples; FCL endpoints then interval midpoints",
            "transport_object_padding_m": padding,
            "gripper_joint_position": self._last_grip,
            "validation_seconds": time.perf_counter()-started,
        }

    @_timed_context_operation('query_s')
    def plan_joint_goal(self, q_arm7, goal_q_arm7, ignore_names=(), grip_binary=0,
                        phase='above_bowl'):
        """Plan to an explicit redundant arm configuration using MotionGen.

        Like plan_pose, this does not certify the independent FCL constraints.
        The caller may correct raw_positions, then retime_path and validate_path
        before execution. raw_positions retain source_dt from MotionGen.
        """
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

        start_q = self._path_array(np.asarray(q_arm7)[None])[0]
        goal_q = self._path_array(np.asarray(goal_q_arm7)[None])[0]
        padding = self._prepare_path_query(ignore_names, grip_binary, phase)
        joint_names = [f"panda_joint{i}" for i in range(1, 8)]
        start = JointState.from_position(self.args.to_device(start_q[None]), joint_names=joint_names)
        goal = JointState.from_position(self.args.to_device(goal_q[None]), joint_names=joint_names)
        started = time.perf_counter()
        result = self.motion_gen.plan_single_js(
            start, goal, MotionGenPlanConfig(max_attempts=4, enable_graph_attempt=2,
                                             enable_finetune_trajopt=True, time_dilation_factor=.8))
        self.torch.cuda.synchronize()
        if not bool(result.success.all()):
            raise RuntimeError(f"cuRobo joint-goal MotionGen failed: {result.status}; "
                               f"valid_query={result.valid_query}")
        raw = self._path_array(result.get_interpolated_plan().position.detach().cpu().numpy())
        if len(raw) < 2:
            raw = np.vstack([start_q, raw[-1]])
        source_dt = float(result.interpolation_dt)
        positions = self.retime_path(raw, source_dt=source_dt)
        start_error = float(np.max(np.abs(positions[0]-start_q)))
        goal_error = float(np.max(np.abs(positions[-1]-goal_q)))
        if start_error > 1e-4:
            raise RuntimeError("cuRobo joint-goal output does not start at the actual joint state")
        if goal_error > 1e-4:
            raise RuntimeError("cuRobo joint-goal output does not end at the requested joint state")
        self.last_plan = {
            "backend": "curobo", "method": "motion_gen_joint_goal",
            "collision_geometry": self.collision_geometry,
            "status": "SUCCESS" if result.status is None else str(result.status),
            "planning_seconds": time.perf_counter()-started,
            "samples": len(positions), "dt": self.dt,
            "raw_samples": len(raw), "source_dt": source_dt,
            "gripper_joint_position": self._last_grip,
            "attached_bottle": self.world.attachment is not None,
            "transport_object_padding_m": padding,
            "start_joint_error_rad": start_error, "goal_joint_error_rad": goal_error,
            "goal_q_arm": goal_q.tolist(),
        }
        return {"positions": positions, "raw_positions": raw, "source_dt": source_dt,
                "dt": self.dt, "metadata": self.last_plan,
                "effective_goal_pose": self.kin.fk(goal_q)}

    def _cartesian_vertical(self, q_arm7, target, phase):
        """Keep grasp, placement and withdrawal motions vertical and local.

        MotionGen handles global motion. Its global IK optimizer may change
        redundant joint branches, so contact approach uses a local warm-started
        least-squares solve, followed by cuRobo and FCL geometry checks.
        """

        started = time.perf_counter()
        initial = self.kin.fk(q_arm7)
        goal = np.array(target, copy=True)
        goal[:3, :3] = initial[:3, :3]
        count = max(2, int(np.ceil(np.linalg.norm(goal[:3, 3]-initial[:3, 3])/.002))+1)
        path = [np.asarray(q_arm7).copy()]
        for fraction in np.linspace(0., 1., count)[1:]:
            tcp = initial.copy()
            tcp[:3, 3] = initial[:3, 3] + fraction*(goal[:3, 3]-initial[:3, 3])
            q = self.kin.ik(tcp, path[-1], attempts=1)
            if np.max(np.abs(q-path[-1])) > .15:
                raise RuntimeError(f"Local Cartesian IK switched joint branches during {phase}")
            path.append(q)
        path = np.asarray(path)
        # At least three seconds, while respecting the same joint command speed cap.
        segment_times = np.maximum(3./(len(path)-1),
                                   np.max(np.abs(np.diff(path, axis=0)), axis=1)/self.max_joint_speed)
        knot_times = np.r_[0., np.cumsum(segment_times)]
        times = np.minimum(np.arange(int(np.ceil(knot_times[-1]/self.dt))+1)*self.dt, knot_times[-1])
        positions = np.stack([np.interp(times, knot_times, path[:, j]) for j in range(7)], axis=-1)
        poses = np.array([self.kin.fk(q) for q in positions])
        xy = np.linalg.norm(poses[:, :2, 3]-goal[:2, 3], axis=1)
        rotation = Rotation.from_matrix(poses[:, :3, :3] @ goal[:3, :3].T).magnitude()
        zlo, zhi = sorted([initial[2, 3], goal[2, 3]])
        if xy.max() > .005 or rotation.max() > np.deg2rad(2.):
            raise RuntimeError(f"Cartesian {phase} left the 5 mm / 2 degree vertical column")
        if poses[:, 2, 3].min() < zlo-.002 or poses[:, 2, 3].max() > zhi+.002:
            raise RuntimeError(f"Cartesian {phase} left the requested height interval")
        feasible = self.motion_gen.ik_solver.check_valid(self.args.to_device(positions))
        if not bool(feasible.all()):
            raise RuntimeError("cuRobo rejected the local Cartesian path's collision/joint constraints")
        self.last_plan = {
            "backend": "curobo", "method": "cartesian_ik", "solver": "usd_scipy_ik", "status": "SUCCESS",
            "collision_geometry": self.collision_geometry,
            "planning_seconds": time.perf_counter()-started,
            "samples": len(positions), "dt": self.dt, "ik_waypoints": count,
            "gripper_joint_position": self._last_grip, "attached_bottle": self.world.attachment is not None,
            "vertical_motion_phase": phase,
            "vertical_lowering": phase == "lower", "orientation_mode": "hold_current",
            "max_xy_deviation_m": float(xy.max()),
            "max_orientation_deviation_deg": float(np.rad2deg(rotation.max())),
            "minimum_tcp_z_m": float(poses[:, 2, 3].min()),
            "maximum_tcp_z_m": float(poses[:, 2, 3].max()),
            "start_joint_error_rad": float(np.max(np.abs(positions[0]-q_arm7))),
        }
        return {"positions": positions, "dt": self.dt, "metadata": self.last_plan,
                "effective_goal_pose": goal}

    @_timed_context_operation('query_s')
    def plan_pose(self, q_arm7, T_W_TCP, ignore_names=(), grip_binary=0, phase=None):
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

        object_padding = self._prepare_path_query(ignore_names, grip_binary, phase)
        # Approach is contact-sensitive: a global joint-space detour can push
        # the still-free target bottle sideways before the fingers close.
        if phase in ("approach", "lift", "lower", "retreat"):
            return self._cartesian_vertical(q_arm7, T_W_TCP, phase)
        T_B_ee = self.T_B_W @ T_W_TCP @ np.linalg.inv(self.T_ee_tcp)
        goal = Pose.from_list(self._pose_list(T_B_ee), self.args)
        joint_names = [f"panda_joint{i}" for i in range(1, 8)]
        start = JointState.from_position(self.args.to_device(np.asarray(q_arm7)[None]), joint_names=joint_names)
        started = time.perf_counter()
        result = self.motion_gen.plan_single(
            start, goal, MotionGenPlanConfig(max_attempts=4, enable_graph_attempt=2,
                                             enable_finetune_trajopt=True, time_dilation_factor=.8))
        self.torch.cuda.synchronize()
        if not bool(result.success.all()):
            raise RuntimeError(f"cuRobo MotionGen failed: {result.status}; valid_query={result.valid_query}; "
                               f"position_error={result.position_error}; rotation_error={result.rotation_error}")
        trajectory = result.get_interpolated_plan().position.detach().cpu().numpy()
        actual_dt = float(result.interpolation_dt)
        if len(trajectory) < 2:
            trajectory = np.vstack([np.asarray(q_arm7), trajectory[-1]])
        # Retime to the collector's exact action period, with a hard command-speed cap.
        peak_speed = np.max(np.abs(np.diff(trajectory, axis=0))) / actual_dt
        scale = max(1., peak_speed / self.max_joint_speed)
        original_times = np.arange(len(trajectory)) * actual_dt * scale
        count = int(np.ceil(original_times[-1] / self.dt)) + 1
        times = np.minimum(np.arange(count) * self.dt, original_times[-1])
        positions = np.stack([np.interp(times, original_times, trajectory[:, j]) for j in range(7)], axis=-1)
        if np.max(np.abs(positions[0]-q_arm7)) > 1e-4:
            raise RuntimeError("cuRobo output does not start at the actual joint state")
        self.last_plan = {
            "backend": "curobo", "method": "motion_gen",
            "collision_geometry": self.collision_geometry,
            "status": "SUCCESS" if result.status is None else str(result.status),
            "planning_seconds": time.perf_counter() - started,
            "samples": len(positions), "dt": self.dt,
            "gripper_joint_position": self._last_grip,
            "attached_bottle": self.world.attachment is not None,
            "transport_object_padding_m": object_padding,
        }
        return {"positions": positions, "dt": self.dt, "metadata": self.last_plan,
                "effective_goal_pose": T_W_TCP}
