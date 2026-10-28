"""Full post-reset physical evidence for independent assigned-task collectors.

The recorder is installed before any warmup/settling action. It does not step,
reset, write object state, or create constraints. Array order is declared in its
metadata, preserving banana/bowl identities rather than a mustard-shaped schema.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .startup_recorder import array, background_topology, pose_change, sha


class TaskPhysicsRecorder:
    def __init__(self, env, envdef, contacts):
        from pxr import Usd, UsdGeom, UsdPhysics
        self.e, self.envdef, self.contacts = env, envdef, contacts
        self.robot, self.scene = env.scene["robot"], env.scene
        self.task = envdef.task
        self.phase, self.action_index, self.on_sample = "reset", -1, None
        self.rows = []
        self.callback_errors = []
        self.reset_step = int(env.sim.get_physics_step_count())
        self.object_names = list(envdef.resolved_task_layout["objects"])
        self.sim_view = self.scene["robot_contacts"]._physics_sim_view
        root = "/World/envs/env_0/kitchen"
        self.background_view = self.sim_view.create_rigid_body_view(root + "/**")
        topology = background_topology(env.sim.stage, self.background_view)
        self.background_paths = list(self.background_view.prim_paths)
        self.static_paths = topology["static_anchor_paths"]
        self.reference_paths = [root + "/" + item["prim_path"].lstrip("/")
            for item in envdef.source_task_spec["object_references"]]
        self.articulation_views, articulation_errors = [], []
        for prim in Usd.PrimRange(env.sim.stage.GetPrimAtPath(root)):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                path = str(prim.GetPath())
                try:
                    view = self.sim_view.create_articulation_view(path)
                    if view.count != 1:
                        raise RuntimeError(f"Expected one native articulation at {path}, got {view.count}")
                    # Force a real read now. USD joint attributes are not native q.
                    array(view.get_dof_positions())
                    self.articulation_views.append((path, view))
                except Exception as error:
                    articulation_errors.append(dict(path=path, error=repr(error)))
        self.metadata = dict(schema_version=1, task_id=envdef.resolved_task_layout["task_id"],
            reset_physics_step=self.reset_step, physics_dt_s=float(env.physics_dt), action_dt_s=float(env.step_dt),
            joint_names=list(self.robot.joint_names), robot_body_names=list(self.robot.body_names),
            contact_body_names=list(self.scene["robot_contacts"].body_names),
            task_object_names=self.object_names, background_paths=self.background_paths,
            background_topology=topology, static_anchor_paths=self.static_paths,
            task_reference_paths=self.reference_paths,
            native_articulation_paths=[path for path, _ in self.articulation_views],
            native_articulation_read_errors=articulation_errors,
            source_layout=envdef.resolved_task_layout, source_relations=envdef.source_relation_audit,
            background_physics=envdef.background_physics_provenance,
            source_hashes={name: sha(item["usd_path"]) for name, item in envdef.resolved_task_layout["objects"].items()},
            source_task_success="Unmodified PickAndPlaceTask predicate sequence, plus separately recorded final object_on_destination",
            scope="Post-reset initial state and every scene.update physics substep, including warmup and 5s initial settle; no reset-internal pre-return observation claim")
        self.capture()
        self.original_update = self.scene.update

        def update(dt, *args, **kwargs):
            result = self.original_update(dt, *args, **kwargs)
            step = int(env.sim.get_physics_step_count())
            if step != self.rows[-1]["physics_step"]:
                try:
                    row = self.capture()
                    if self.on_sample is not None:
                        self.on_sample(row)
                except Exception as error:
                    self.callback_errors.append(dict(physics_step=step, phase=self.phase, error=repr(error)))
                    raise
            return result
        self.scene.update = update

    def capture(self):
        from pxr import UsdGeom
        cache = UsdGeom.XformCache()
        robot = self.robot
        body = np.concatenate((array(robot.data.body_pos_w)[0], array(robot.data.body_quat_w)[0]), axis=-1)
        palm = body[robot.body_names.index("base_link")]
        tcp = np.r_[palm[:3]+Rotation.from_quat(palm[3:]).apply([.131, 0., 0.]), palm[3:]]
        states = np.stack([array(self.scene[name].data.root_state_w)[0] for name in self.object_names])
        row = dict(physics_step=int(self.e.sim.get_physics_step_count()), phase=self.phase,
            action_index=self.action_index, q=array(robot.data.joint_pos)[0], dq=array(robot.data.joint_vel)[0],
            planning_attachment=np.eye(4), has_planning_attachment=False,
            robot_body_pose_w=body, tcp_pose_w=tcp, task_object_states=states,
            robot_contact_force=array(self.scene["robot_contacts"].data.net_forces_w)[0],
            destination_contact_force=array(self.scene[self.task.contact_sensor_cfg.name].data.force_matrix_w)[0],
            background_rigid_poses=array(self.background_view.get_transforms()),
            background_rigid_velocities=array(self.background_view.get_velocities()))
        for key, paths in (("static_anchor_transforms", self.static_paths), ("task_reference_transforms", self.reference_paths)):
            row[key] = np.asarray([np.asarray(cache.GetLocalToWorldTransform(self.e.sim.stage.GetPrimAtPath(path))).T
                                  for path in paths]).reshape(-1, 4, 4)
        for index, (_, view) in enumerate(self.articulation_views):
            row[f"background_articulation_{index}_q"] = array(view.get_dof_positions())
            row[f"background_articulation_{index}_dq"] = array(view.get_dof_velocities())
        self.rows.append(row)
        self.contacts.sample(self.phase, self.action_index, states[self.object_names.index("banana"), :7],
                             physics_step=row["physics_step"])
        return row

    def close(self):
        self.scene.update = self.original_update

    def write(self, directory, actions):
        directory = Path(directory)
        arrays = {key: np.asarray([row[key] for row in self.rows]) for key in self.rows[0]}
        self.metadata["callback_errors"] = self.callback_errors
        self.metadata["physics_samples"] = len(self.rows)
        np.savez_compressed(directory/"physics_states.npz", **arrays)
        (directory/"physics_metadata.json").write_text(json.dumps(self.metadata, indent=2, allow_nan=False)+"\n")
        (directory/"grasp_contacts.json").write_text(json.dumps(self.contacts.records, allow_nan=False)+"\n")
        # A startup sidecar uses this task's explicit schema; it is not passed to
        # the old mustard validator, whose field meanings are different.
        prefix = np.isin(arrays["phase"], ["reset", "warmup", "initial_settle"])
        np.savez_compressed(directory/"startup_physics_states.npz", **{key: value[prefix] for key, value in arrays.items()})
        startup = dict(schema_version=1, task_object_names=self.object_names,
            first_physics_step=int(arrays["physics_step"][0]),
            last_physics_step=int(arrays["physics_step"][prefix][-1]), samples=int(prefix.sum()),
            continuous=bool(np.all(np.diff(arrays["physics_step"][prefix]) == 1)),
            duration_s=float((arrays["physics_step"][prefix][-1]-arrays["physics_step"][0])*self.e.physics_dt),
            scope="Complete reset/warmup/5s-settle prefix from the task-aware full recorder; separate source geometry checks retained in result.json")
        (directory/"startup_audit.json").write_text(json.dumps(startup, indent=2, allow_nan=False)+"\n")
        return audit_physics(arrays, self.metadata, actions, self.contacts.records)


def audit_physics(data, meta, actions, contacts):
    """CPU evidence coverage, background drift, force and camera alignment gates."""
    steps = np.asarray(data["physics_step"])
    count, dt = len(steps), meta["physics_dt_s"]
    expected_shapes = dict(q=(count, len(meta["joint_names"])), dq=(count, len(meta["joint_names"])),
        task_object_states=(count, len(meta["task_object_names"]), 13),
        robot_body_pose_w=(count, len(meta["robot_body_names"]), 7),
        robot_contact_force=(count, len(meta["contact_body_names"]), 3),
        destination_contact_force=(count, 1, 1, 3),
        background_rigid_poses=(count, len(meta["background_paths"]), 7),
        background_rigid_velocities=(count, len(meta["background_paths"]), 6),
        static_anchor_transforms=(count, len(meta["static_anchor_paths"]), 4, 4),
        task_reference_transforms=(count, len(meta["task_reference_paths"]), 4, 4))
    for key, shape in expected_shapes.items():
        if np.asarray(data[key]).shape != shape:
            raise ValueError(f"Missing/misaligned physical field: {key}; expected {shape}")
    decimation = round(meta["action_dt_s"]/dt)
    elapsed = (steps-steps[0])*dt
    finite = all(value.dtype.kind in "US" or np.isfinite(value).all() for value in data.values())
    free = np.flatnonzero(elapsed >= 5.-1e-8)
    linked = set(meta["background_topology"]["joint_connected_native_paths"])
    background = []
    if len(free):
        for index, path in enumerate(meta["background_paths"]):
            start = 0 if path in linked else int(free[0])
            motion = pose_change(data["background_rigid_poses"][:, index], start)
            background.append(dict(path=path, joint_connected=path in linked, **motion,
                passed=motion["max_translation_m"] <= .002 and motion["max_rotation_deg"] <= 1.))
    nonfinger = [i for i, name in enumerate(meta["contact_body_names"]) if name.startswith("panda_link") or name == "base_link"]
    if not nonfinger:
        raise ValueError("No actual arm/palm contact sensor bodies")
    max_force = float(np.linalg.norm(data["robot_contact_force"][:, nonfinger], axis=-1).max())
    alignment = [row["camera_physics_steps"] for row in actions]
    checks = dict(
        finite_state=bool(finite),
        reset_initial_and_all_substeps=bool(count == len(actions)*decimation+1 and steps[0] == meta["reset_physics_step"]
            and np.all(np.diff(steps) == 1) and not meta["callback_errors"]),
        raw_finger_contact_every_sample=len(contacts) == count and
            [row["physics_step"] for row in contacts] == steps.tolist(),
        camera_action_alignment=bool(alignment and all(pair[0] == pair[1] == meta["reset_physics_step"]+i*decimation
            for i, pair in enumerate(alignment))),
        background_topology_complete=bool(linked and linked <= set(meta["background_paths"]) and
            not meta["background_topology"]["unobserved_joint_connected_bodies"]),
        background_stable=bool(background and all(row["passed"] for row in background)),
        background_native_joints_observed=not meta["native_articulation_read_errors"],
        static_anchors_stable=bool(not data["static_anchor_transforms"].size or
            np.max(np.abs(data["static_anchor_transforms"]-data["static_anchor_transforms"][0])) <= 1e-6),
        task_references_stable=bool(not data["task_reference_transforms"].size or
            np.max(np.abs(data["task_reference_transforms"]-data["task_reference_transforms"][0])) <= 1e-6),
        max_arm_contact_force_below_original_5n=max_force < 5.)
    return dict(passed=all(checks.values()), checks=checks, physics_samples=count,
        expected_physics_samples=len(actions)*decimation+1, physics_dt_s=dt,
        max_arm_contact_force_n=max_force, background=background,
        required_independent_sidecars=["neutral task probe", "fixed base/source layout audit", "actual q self-collision audit", "camera pose audit", "visual review"])
