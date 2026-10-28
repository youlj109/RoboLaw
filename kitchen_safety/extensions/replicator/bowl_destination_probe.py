"""Diagnostic-only original bowl/floor observations; never dataset acceptance.

No normal collector imports this module. Lower keeps the original 2 mm gate.
The explicit bounded release diagnostic records a rejected original triangle
pair while inspecting native contact. All other source pairs remain checked.
"""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from .bowl_collision import BowlCollisionWorld
from .bowl_destination_query import destination_floor, selected_target
from .bowl_source_support_query import collider_geometry_evidence, digest
from .task_contacts import _array

TASK = "kitchen_bench_replicator_g_shape_bowl_sink"
BODY = "/World/envs/env_0/bowl"
FLOOR = "/World/envs/env_0/kitchen/Sink/BasinBottom"
MAX_CONTACTS = 4096
DECLARATION = {
    "schema_version": 1, "task_id": TASK,
    "diagnostic_only": True, "accepted": False, "physical_trial_passed": False,
    "noncontact_clearance_m": .002, "release_gap_window_m": [.003, .005],
    "landing_xy_tolerance_m": .005, "maximum_release_actions": 7,
    "maximum_release_physics_steps": 56, "physics_dt_s": 1/120,
    "action_dt_s": 1/15, "release_target": FLOOR, "payload_body": BODY,
    "force_threshold_n": .1, "support_cone_half_angle_rad": float(np.pi/4),
    "contact_capacity": MAX_CONTACTS,
    "continuation_scope": "Only original source bowl/BasinBottom intersection during bounded release; original rejection retained. Not a production penetration tolerance.",
}


def read_declaration(path):
    value = json.loads(Path(path).read_text())
    if value != DECLARATION:
        raise ValueError("Diagnostic declaration differs from the reviewed fixed bound/rules")
    return value


def unpack_native_contact(data, *, capacity=MAX_CONTACTS):
    """Exact one-body/one-filter raw contact inventory with no truncation."""
    force, point, normal, separation, count, start = map(_array, data)
    if count.size != 1 or start.size != 1:
        raise ValueError("Need exactly one bowl/floor raw contact pair")
    n, first = int(count.reshape(-1)[0]), int(start.reshape(-1)[0])
    if n < 0 or first < 0 or first+n >= capacity:
        raise ValueError("Destination raw contact buffer invalid or at declared capacity")
    if (point.ndim != 2 or point.shape[1:] != (3,) or normal.shape != point.shape or
        first+n > len(point) or force.size < first+n or separation.size < first+n):
        raise ValueError("Destination contact arrays are incomplete")
    selected = slice(first, first+n)
    points, normals = point[selected], normal[selected]
    forces, gaps = force.reshape(-1)[selected], separation.reshape(-1)[selected]
    if not all(np.isfinite(v).all() for v in (points, normals, forces, gaps)):
        raise ValueError("Nonfinite destination raw contact")
    return dict(count=n, buffer_count=count.tolist(), buffer_start=start.tolist(),
        buffer_capacity=capacity, points_world_m=points.tolist(), normals_world=normals.tolist(),
        normal_forces_n=forces.tolist(), separations_m=gaps.tolist())


def native_offsets(root_view):
    """Read only supported tensor getters; unavailable is never replaced by 0."""
    report = {}
    for name in ("get_rest_offsets", "get_contact_offsets"):
        try:
            values = _array(getattr(root_view, name)())
            if not values.size or not np.isfinite(values).all():
                raise ValueError("Empty/nonfinite native offset array")
            report[name] = dict(available=True, values_m=values.tolist(), shape=list(values.shape))
        except Exception as error:
            report[name] = dict(available=False, reason=f"{type(error).__name__}: {error}")
    return report


class DestinationNativeContacts:
    def __init__(self, physics_view, physics_dt, sensor_identity, bowl_root_view):
        self.identity = deepcopy(sensor_identity)
        if (self.identity["actual_pick_up_path"] != BODY or
            self.identity["actual_destination_path"] != FLOOR or
            self.identity["sensed_native_paths"] != [BODY] or
            self.identity["resolved_filter_prim_paths"] != [FLOOR] or
            self.identity["native_filter_count"] != 1 or
            physics_dt != DECLARATION["physics_dt_s"]):
            raise ValueError("Native destination identity/time base differs from the original task")
        self.dt = physics_dt
        self.view = physics_view.create_rigid_contact_view([BODY],
            filter_patterns=[[FLOOR]], max_contact_data_count=MAX_CONTACTS)
        if int(self.view.filter_count) != 1:
            raise ValueError("Independent native destination view has wrong filter inventory")
        self.offsets = dict(bowl=native_offsets(bowl_root_view),
            static_floor=dict(available=False,
                reason="No verified static-collider offset tensor getter; not inferred from absent USD fields"))
        self.rows = []

    def sample(self, row):
        value = dict(physics_step=int(row["physics_step"]), phase=row["phase"],
            action_index=int(row["action_index"]), exact_body=BODY, exact_filter=FLOOR,
            **unpack_native_contact(self.view.get_contact_data(dt=self.dt)))
        normals = np.asarray(value['normals_world']).reshape(-1, 3)
        loads = np.asarray(value['normal_forces_n'])
        summed = (loads[:, None]*normals).sum(0)
        original = np.asarray(row['destination_contact_force']).reshape(-1, 3).sum(0)
        if not np.isfinite(original).all():
            raise ValueError('Nonfinite original filtered destination force')
        value.update(native_normal_force_vector_sum_n=summed.tolist(),
            original_filtered_force_n=original.tolist(), native_minus_original_force_n=(summed-original).tolist(),
            native_original_force_difference_norm_n=float(np.linalg.norm(summed-original)),
            force_difference_is_diagnostic_only=True)
        self.rows.append(value)
        return value


def support_continuation_checks(source, native, filtered_force, low, high):
    """Diagnostic geometric bounds/normal evidence, not a penetration allowance."""
    low, high, force = np.asarray(low), np.asarray(high), np.asarray(filtered_force)
    points = np.asarray(native["points_world_m"], float).reshape(-1, 3)
    normals = np.asarray(native["normals_world"], float).reshape(-1, 3)
    loads = np.asarray(native["normal_forces_n"], float)
    gaps = np.asarray(native["separations_m"], float)
    source_contacts = source["contacts"]
    source_points = np.asarray([c["point_world_m"] for c in source_contacts], float).reshape(-1, 3)
    source_normals = np.asarray([c["normal_world"] for c in source_contacts], float).reshape(-1, 3)
    def footprint(p):
        return bool(len(p) and np.all(p[:, :2] >= low[:2]) and np.all(p[:, :2] <= high[:2]))
    def axis(n):
        length = np.linalg.norm(n, axis=1)
        # FCL's source-bowl/floor ordered pair gives the opposite sign from
        # force on the bowl. Its geometric axis is separate from native force.
        return bool(len(n) and np.all(length > 0.) and
            np.all(np.abs(n[:, 2])/np.maximum(length, 1e-30) >= np.cos(np.pi/4)))
    complete = bool(len(points) == len(normals) == len(loads) == len(gaps) and
        all(np.isfinite(v).all() for v in (points, normals, loads, gaps)))
    # Installed omni.physics.tensors/api.py:5040 specifies normal_force *
    # normal as each contribution to the sensor/filter force. Zero-load
    # contacts have no force direction: retain them and check their position,
    # but never count them as support or apply a directional cone to zero.
    loaded = loads > 0.
    contributions = loads[:, None]*normals if complete else np.empty((0, 3))
    native_sum = contributions.sum(0)
    load_normals = normals[loaded] if complete else np.empty((0, 3))
    lengths = np.linalg.norm(load_normals, axis=1)
    native_up = bool(len(load_normals) and np.all(lengths > 0.) and
        np.all(load_normals[:, 2]/np.maximum(lengths, 1e-30) >= np.cos(np.pi/4)))
    force_norm = float(np.linalg.norm(force))
    checks = dict(native_exact_identity=native.get("exact_body") == BODY and native.get("exact_filter") == FLOOR,
        raw_contact_present=native["count"] > 0 and len(points) == native["count"],
        raw_contact_complete=complete,
        raw_contact_loaded=bool(len(loads) and np.any(loads > 0.)),
        raw_normal_loads_nonnegative=bool(len(loads) and np.all(loads >= 0.)),
        native_points_in_floor_footprint=footprint(points), native_loaded_normals_in_upward_cone=native_up,
        original_force_above_threshold=force_norm >= .1,
        original_force_in_upward_cone=bool(force_norm > 0. and force[2]/force_norm >= np.cos(np.pi/4)),
        source_contact_present=bool(source_contacts), source_contact_points_in_floor_footprint=footprint(source_points),
        source_normal_axis_within_original_cone=axis(source_normals),
        bowl_entire_xy_inside_floor=source["full_bowl_xy_inside_floor"],
        source_bowl_above_floor_bottom=source["minimum_source_z_m"] >= low[2],
        native_contacts_above_floor_bottom=bool(len(points) and np.all(points[:, 2] >= low[2])))
    return dict(passed=all(checks.values()), checks=checks,
        native_normal_force_vector_sum_n=native_sum.tolist(),
        original_filtered_force_n=force.tolist(), native_minus_original_force_n=(native_sum-force).tolist(),
        native_original_force_difference_norm_n=float(np.linalg.norm(native_sum-force)),
        force_difference_is_diagnostic_only=True,
        native_contact_force_semantics=dict(loaded_contact_count=int(loaded.sum()),
            unloaded_contact_count=int((loads == 0.).sum()),
            rule="API normal_force*normal contributes to sensor/filter force. Positive loads must point within original +Z cone; zero loads retain geometry/raw records and cannot prove support. Negative normal loads fail closed. No invented force-agreement tolerance."),
        scope="Only bounded diagnostic continuation; no native/source penetration tolerance or task success")


def release_boundary(source, *, plan_complete, xy_error_m, contact_summary, current_step):
    steps = [row["physics_step"] for row in contact_summary.get("steps", [])]
    checks = dict(original_plan_complete=bool(plan_complete),
        source_noncontact_2mm=bool(not source["intersecting"] and source["distance_m"] >= .002),
        current_gap_in_declared_window=.003 <= source["source_gap_m"] <= .005,
        landing_xy_original_5mm=bool(np.isfinite(xy_error_m) and xy_error_m <= .005),
        latest_ten_actual_lower_steps=steps == list(range(current_step-9, current_step+1)),
        original_lower_bilateral_loaded=bool(contact_summary.get("both_fingers_loaded")),
        original_lower_bilateral_near=bool(contact_summary.get("both_fingers_near")),
        source_xy_inside_floor=bool(source["full_bowl_xy_inside_floor"]))
    return dict(passed=all(checks.values()), checks=checks, source=source,
        xy_error_m=float(xy_error_m), physics_step=int(current_step), contact_summary=contact_summary)


class DiagnosticDestinationWorld(BowlCollisionWorld):
    """Private probe world. Does not alter normal bowl world or its assets."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.probe_floor_binding = collider_geometry_evidence(self, destination_floor(self)[0])
        self.probe_payload_hash = self.payload_geometry.provenance["source_sha256"]
        self.probe_records, self.strict_failure_ledger = [], []
        self.probe_row = self.probe_native = None
        self.release_start_step = None
        self._probe_execution_active = False
        self._probe_measurements = {}

    def source_floor_observation(self, pose):
        floor = destination_floor(self)
        geometry = collider_geometry_evidence(self, floor[0])
        if (geometry["geometry_sha256"] != self.probe_floor_binding["geometry_sha256"] or
            self.payload_geometry.provenance["source_sha256"] != self.probe_payload_hash):
            raise ValueError("Diagnostic floor/payload source binding changed")
        pose = np.asarray(pose)
        shape = self.fcl.CollisionObject(self.payload_triangle_model, self.fcl.Transform(pose[:3, :3], pose[:3, 3]))
        distance = float(self.fcl.distance(shape, floor[1], self.fcl.DistanceRequest(), self.fcl.DistanceResult()))
        result = self.fcl.CollisionResult()
        count = int(self.fcl.collide(shape, floor[1],
            self.fcl.CollisionRequest(num_max_contacts=MAX_CONTACTS, enable_contact=True), result))
        if not np.isfinite(distance) or count >= MAX_CONTACTS or len(result.contacts) >= MAX_CONTACTS:
            raise ValueError("Source floor distance/contact inventory incomplete")
        points = self.payload_geometry.vertices @ pose[:3, :3].T + pose[:3, 3]
        low, high = points.min(0), points.max(0)
        return dict(distance_m=max(0., distance), intersecting=bool(count or distance < 0.),
            source_gap_m=float(low[2]-floor[3][2]), minimum_source_z_m=float(low[2]),
            full_bowl_xy_inside_floor=bool(np.all(low[:2] >= floor[2][:2]) and np.all(high[:2] <= floor[3][:2])),
            floor_bounds=[floor[2].tolist(), floor[3].tolist()],
            exact_floor_geometry_sha256=geometry["geometry_sha256"],
            contacts=[dict(point_world_m=np.asarray(c.pos).tolist(), normal_world=np.asarray(c.normal).tolist(),
                penetration_depth_m=float(c.penetration_depth), source_triangle=int(c.b1), floor_triangle=int(c.b2))
                for c in result.contacts])

    def _floor_measurement(self):
        # Planning queries use original source geometry at >=2mm, no probe data.
        pose = self._query_payload_pose
        floor = destination_floor(self)
        shape = self.fcl.CollisionObject(self.payload_triangle_model, self.fcl.Transform(pose[:3, :3], pose[:3, 3]))
        margin = .002 if self.phase == "lower" else 0.
        strict = self._exact_measurement((0., "payload/bowl", FLOOR, shape, floor[1], margin))
        strict.update(contact_pair="payload/destination", noncontact_required_m=.002,
            representation="Original source triangles; diagnostic lower retains 2mm")
        if not self._probe_execution_active:
            return strict
        row = self.probe_row
        if row is None or self.probe_native is None or self.probe_native["physics_step"] != row["physics_step"]:
            raise ValueError("Missing aligned native destination observation")
        step = int(row["physics_step"])
        if step in self._probe_measurements:
            return deepcopy(self._probe_measurements[step])
        source = self.source_floor_observation(pose)
        record = dict(physics_step=step, phase=self.phase, action_index=int(row["action_index"]),
            source=source, original_strict_measurement=deepcopy(strict), diagnostic_continuation=False)
        answer = deepcopy(strict)
        if self.phase == "release":
            if self.release_start_step is None or not 1 <= step-self.release_start_step <= 56:
                raise ValueError("Release observation exceeded predeclared 56 physical steps")
            if source["minimum_source_z_m"] < floor[2][2] or not source["full_bowl_xy_inside_floor"]:
                raise ValueError("Diagnostic bowl crossed original floor footprint/entity lower boundary")
            if not strict["valid"]:
                force = np.asarray(row["destination_contact_force"]).reshape(-1, 3).sum(0)
                continuation = support_continuation_checks(source, self.probe_native, force, floor[2], floor[3])
                record["support_continuation"] = continuation
                if continuation["passed"]:
                    answer.update(valid=True, diagnostic_continuation_only=True, original_strict_valid=False)
                    record["diagnostic_continuation"] = True
                self.strict_failure_ledger.append(deepcopy(record))
        elif not strict["valid"]:
            self.strict_failure_ledger.append(deepcopy(record))
        self.probe_records.append(record)
        self._probe_measurements[step] = deepcopy(answer)
        return answer

    def _measurement(self, pair):
        if pair[1] == "payload/bowl" and pair[2] == FLOOR and self.phase in ("lower", "release"):
            return self._floor_measurement()
        return super()._measurement(pair)

    def valid(self, q, ignore=(), *, mode="planning", gripper_q=None, object_pose=None, obstacle_filter=None):
        self._probe_execution_active = mode == "execution"
        try:
            valid = super().valid(q, ignore, mode=mode, gripper_q=gripper_q,
                object_pose=object_pose, obstacle_filter=obstacle_filter)
            if not valid:
                return False
            # Explicitly observe this pair even when broad-phase proves it far.
            # All other pairs have already retained the original source gates.
            if self._probe_execution_active and self.phase in ("lower", "release"):
                measurement = self._floor_measurement()
                if not measurement["valid"]:
                    self.last_clearance = measurement
                    self.last_collision = (measurement["robot_part"], measurement["obstacle"])
                    return False
            return True
        finally:
            self._probe_execution_active = False
