"""Water-only lowering refinement for attached bowl / the added valve Grip.

The original bowl/BasinBottom refinement is preserved. A disjoint exhaustive
CUDA decomposition retains every robot/world and every other payload/world
pair, including all original margins. Only the added Grip/payload sphere pair
uses the source bowl's solid convex envelope and source analytic cuboid at the
unchanged 2 mm noncontact margin. No physical masks or state writes occur.
"""
from copy import deepcopy

import numpy as np

from ..bowl_transport_curobo import BowlTransportCuroboPlanner
from ..bowl_destination_query import destination_floor
from ..bowl_source_support_query import collider_geometry_evidence, digest, rigid
from ..task_plan_diagnostics import constraint_components
from .water_control import GRIP


def grip_geometry(world):
    rows = [row for row in world.objects if row[0] == GRIP]
    provenance = [row for row in world.valve_geometry_refinement if row["path"] == GRIP]
    if (len(rows) != 1 or len(provenance) != 1 or provenance[0]["source_schema"] != "Cube"
            or world.default_clearance != .002 or world.payload_name != "bowl"):
        raise ValueError("Water lowering requires the original bowl and one source valve Grip cuboid")
    obj = rows[0][1]
    if obj.getNodeType() != world.fcl.Box(.090, .020, .016).getNodeType():
        raise ValueError("The added Grip must retain its exact analytic Box collider")
    # The source-derived CUDA mesh and analytic object have the same pose.
    # Bind vertices/topology as well as the transform, never just an AABB.
    vertices, _ = world.meshes[GRIP]
    local = (np.asarray(vertices)-obj.getTranslation())@obj.getRotation()
    source_float3_dimensions = np.asarray([.090, .020, .016], dtype=np.float32).astype(float)
    if not np.allclose(np.ptp(local, axis=0), source_float3_dimensions, atol=1e-9, rtol=0.):
        raise ValueError("Unexpected source Grip dimensions")
    return rows[0], collider_geometry_evidence(world, GRIP)


def source_bowl_grip_clearance(world, kin, positions):
    """Conservative solid source envelope, not a hollow triangle-only test."""
    grip, evidence = grip_geometry(world)
    attachment = rigid(world.attachment, "observed bowl attachment")
    path = np.asarray(positions, float)
    if path.ndim != 2 or path.shape[1] != 7 or not np.isfinite(path).all():
        raise ValueError("Require finite seven-joint source geometry samples")
    rows = []
    for index, q in enumerate(path):
        pose = kin.fk(q)@attachment
        shape = world.fcl.CollisionObject(world.payload_model,
            world.fcl.Transform(pose[:3, :3], pose[:3, 3]))
        overlap = bool(world.fcl.collide(shape, grip[1], world.fcl.CollisionRequest(), world.fcl.CollisionResult()))
        distance = float(world.fcl.distance(shape, grip[1], world.fcl.DistanceRequest(), world.fcl.DistanceResult()))
        if not np.isfinite(distance):
            raise ValueError("Nonfinite source bowl/Grip distance")
        rows.append(dict(sample=index, distance_m=distance, intersecting=overlap,
            passed=bool(not overlap and distance >= world.default_clearance)))
    return dict(passed=all(row["passed"] for row in rows), rows=rows,
        minimum_distance_m=min(row["distance_m"] for row in rows), required_clearance_m=world.default_clearance,
        pair=["attached_object/bowl", GRIP], grip_geometry=evidence,
        original_bowl_source_sha256=world.payload_geometry.provenance["source_sha256"],
        approximation="Solid convex hull of all original bowl collision vertices versus exact added Grip cuboid; convex envelope is conservative around the hollow bowl",
        contact_exemption=False, physical_state_written=False)


def partition_water_lowering(planner, state, original):
    """Four disjoint original GPU pair sets, with all masks restored finally."""
    floor = destination_floor(planner.world)[0]
    grip_geometry(planner.world)
    cost = planner.motion_gen.ik_solver.rollout_fn.primitive_collision_constraint
    checker = cost.world_coll_checker
    config = planner.motion_gen.ik_solver.rollout_fn.dynamics_model.robot_model.kinematics_config
    reverse = {value: key for key, value in config.link_name_to_idx_map.items()}
    labels = [reverse[int(i)] for i in config.link_sphere_idx_map.detach().cpu().numpy()]
    payload = [i for i, name in enumerate(labels) if name == "attached_object"]
    robot = [i for i, name in enumerate(labels) if name != "attached_object"]
    if not payload or not robot or sorted(payload+robot) != list(range(len(labels))):
        raise ValueError("Incomplete robot/payload sphere partition")
    masks = planner.candidate_contact_mask["object_switches"]
    if list(masks) != ["bowl"]:
        raise ValueError("Only the duplicate carried bowl may be masked")
    enabled = [name for name in planner.obstacle_names if not planner._ignored(name, masks)]
    if floor == GRIP or floor not in enabled or GRIP not in enabled or len(enabled) != len(set(enabled)):
        raise ValueError("Missing or ambiguous original floor / added Grip")
    groups = (("robot_all_world", payload, enabled),
        ("payload_non_support", robot, [name for name in enabled if name not in (floor, GRIP)]),
        ("payload_source_support", robot, [floor]),
        ("payload_added_grip", robot, [GRIP]))
    rows = {}
    try:
        for name, inactive, obstacles in groups:
            query = state.robot_spheres.clone()
            query[..., inactive, 3] = -100.
            active = set(obstacles)
            for obstacle in planner.obstacle_names:
                checker.enable_obstacle(obstacle, enable=obstacle in active)
            values = cost.forward(query.contiguous(), env_query_idx=None).detach().cpu().numpy()
            values = values.reshape(len(values), -1).max(axis=1)
            if not np.isfinite(values).all():
                raise ValueError("Nonfinite original GPU pair costs")
            rows[name] = dict(costs=values.tolist(), maximum_cost=float(values.max()),
                nonzero_samples=np.flatnonzero(values != 0.).tolist(), enabled_obstacles=obstacles,
                active_sphere_ids=robot if name == "robot_all_world" else payload)
    finally:
        for obstacle in planner.obstacle_names:
            checker.enable_obstacle(obstacle, enable=obstacle in enabled)
    union = np.logical_or.reduce([np.asarray(row["costs"]) != 0. for row in rows.values()])
    original_bad = np.asarray(original["components"]["world_collision"]["costs"]) != 0.
    if not np.array_equal(union, original_bad):
        raise ValueError("Original full-world query and exhaustive pair decomposition disagree")
    pair_count = sum(len(row["active_sphere_ids"])*len(row["enabled_obstacles"]) for row in rows.values())
    if pair_count != len(labels)*len(enabled):
        raise ValueError("GPU pair decomposition does not cover every original pair exactly once")
    return dict(rows=rows, original_zero_nonzero_equivalent=True, disjoint_exhaustive=True,
        original_sphere_world_pairs=pair_count,
        noncontact_passed=not rows["robot_all_world"]["nonzero_samples"] and not rows["payload_non_support"]["nonzero_samples"],
        refined_source_pairs=[["attached_object", floor], ["attached_object", GRIP]],
        original_margins_retained=True, physical_collision_filter_changes=False,
        payload_non_support_scope="Every original obstacle except source-validated original BasinBottom and added Grip; robot queries retain both")


class WaterBowlPlanner(BowlTransportCuroboPlanner):
    """Unmodified original controller behavior outside bound water lowering."""
    water_lowering_grip_binding = None

    def bind_sink_lowering(self, profile, **observations):
        evidence = super().bind_sink_lowering(profile, **observations)
        if self.world.flowing:
            raise ValueError("Cannot bind bowl lowering while water is flowing")
        _, grip = grip_geometry(self.world)
        self.water_lowering_grip_binding = dict(geometry_sha256=grip["geometry_sha256"],
            bowl_source_sha256=self.world.payload_geometry.provenance["source_sha256"],
            original_lowering_binding_sha256=evidence["evidence_sha256"])
        return dict(**evidence, added_grip_binding=deepcopy(self.water_lowering_grip_binding))

    def _pair_audit(self, positions, phase):
        original_report = super()._pair_audit(positions, phase)
        if phase != "lower":
            return original_report
        source = source_bowl_grip_clearance(self.world, self.kin, positions)
        binding = self.water_lowering_grip_binding
        checks = dict(bound=binding is not None, water_remains_off=not self.world.flowing,
            grip_unchanged=binding is not None and source["grip_geometry"]["geometry_sha256"] == binding["geometry_sha256"],
            bowl_source_unchanged=binding is not None and source["original_bowl_source_sha256"] == binding["bowl_source_sha256"],
            original_binding_unchanged=binding is not None and self.lowering_context.evidence()["evidence_sha256"] == binding["original_lowering_binding_sha256"])
        components, state = constraint_components(self, positions)
        if digest(components) != digest(original_report["original_components"]):
            raise ValueError("Original constraints changed during the exact-pair audit")
        partition = partition_water_lowering(self, state, components)
        report = deepcopy(original_report)
        report["original_support_pair_validation"] = original_report
        report["original_world_pair_partition"] = partition
        report["added_grip_geometry"] = source
        report["added_grip_binding"] = dict(passed=all(checks.values()), checks=checks, binding=deepcopy(binding))
        report["mandatory_checks"]["original_robot_floor_and_payload_other_spheres"] = partition["noncontact_passed"]
        report["mandatory_checks"].update(added_grip_binding=all(checks.values()), exact_source_bowl_added_grip_2mm=source["passed"])
        report["candidate_geometry_valid"] = all(report["mandatory_checks"].values())
        report["valid"] = bool(report["actual_state_binding"]["passed"] and report["candidate_geometry_valid"])
        report["scope"] = "Bound water-task lower only: source-geometry refinement of added Grip / attached bowl; original floor refinement unchanged, every robot/world and remaining payload/world sphere pair, joint/self and full source FCL remain mandatory"
        self.last_destination_validation = report
        return report
