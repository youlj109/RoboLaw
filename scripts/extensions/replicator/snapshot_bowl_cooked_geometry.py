#!/usr/bin/env python3
"""Snapshot original bowl/floor cooking representation without control actions.

Only --dry-run/--help are CPU entry points. Actual Kit initialization is queued
by the root agent under the shared GPU lock. This is a new initialized scene,
not replayed historical physics or accepted task data. No caches are cleared,
no state setters are called, and unsupported native data remains unavailable.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[3]
TASK = "kitchen_bench_replicator_g_shape_bowl_sink"
EXTRA_SOURCE = "kitchen_safety/extensions/replicator/cooked_geometry_snapshot.py"
API_ROOT = ROOT/".venv-arena/lib/python3.12/site-packages/isaacsim/extscache"
API_FILES = {
    "cooking_stub": "omni.physx-*/omni/physx/bindings/_physx.pyi",
    "cooking_binding_test": "omni.physx.tests-*/omni/physxtests/tests/PhysxCookingInterface.py",
    "coordinate_demo": "omni.physx.demos-*/omni/physxdemos/scenes/ConvexMeshDataDemo.py",
    "compound_query_test": "omni.physx.tests-*/omni/physxtests/tests/PhysxPropertyQueryInterface.py",
    "tensor_api": "omni.physics.tensors-*/omni/physics/tensors/api.py",
}


def write(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def installed_api():
    from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import require, sha
    result = {}
    for name, pattern in API_FILES.items():
        matches = list(API_ROOT.glob(pattern))
        require(len(matches) == 1, "Installed API path absent/ambiguous: " + pattern)
        path = matches[0]
        result[name] = dict(path=str(path), sha256=sha(path))
    result["coordinate_contract"] = dict(frame="collision_prim_local", world_matrix="USD source prim local-to-world, including scale",
        evidence="ConvexMeshDataDemo.py:15-18 and 209-234 transform callback vertices with source_to_world",
        unverified="Installed demo demonstrates transform semantics; not a formal native shape-slot mapping")
    result["compound_contract"] = dict(evidence="PhysxPropertyQueryInterface.py:264: one report for a convex-decomposition compound",
        collider_callback_count_is_native_shape_count=False)
    return result


def mesh_snapshot(prim):
    """Read exactly the authored/composed mesh and its collision attributes."""
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics
    from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import require, digest
    require(prim.IsValid() and prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI), "Missing original collision mesh")
    mesh = UsdGeom.Mesh(prim)
    geometry = dict(points_local=np.asarray(mesh.GetPointsAttr().Get(), dtype=float).tolist(),
        face_vertex_counts=list(map(int, mesh.GetFaceVertexCountsAttr().Get())),
        face_vertex_indices=list(map(int, mesh.GetFaceVertexIndicesAttr().Get())))
    matrix = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float).T
    attrs = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if name.startswith(("physics:", "physx")):
            value = attr.Get()
            # Preserve unsupported schema values (including sentinel inf) as
            # source repr, never silently convert an unknown offset to zero.
            attrs[name] = dict(authored=bool(attr.HasAuthoredValueOpinion()), value_repr=repr(value))
    return dict(prim_path=str(prim.GetPath()), type_name=prim.GetTypeName(),
        approximation=str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()),
        collision_enabled=bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()),
        geometry=geometry, source_geometry_sha256=digest(geometry),
        local_to_world_column_matrix=matrix.tolist(), local_to_world_sha256=digest(matrix.tolist()),
        applied_schemas=list(prim.GetAppliedSchemas()), collision_attributes=attrs,
        prim_stack=[dict(layer=spec.layer.identifier, path=str(spec.path)) for spec in prim.GetPrimStack()])


def preflight(args):
    from pxr import Usd
    from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import read_declaration, sha, require
    declaration = read_declaration(args.declaration, ROOT)
    layout = json.loads(args.layout.read_text())
    require(layout["task_id"] == TASK and "obstacle" not in layout, "Require original source baseline")
    source = {}
    for name, path, prim_path in [("bowl", ROOT/layout["objects"]["bowl"]["usd_path"], "/bowl/obj_000013_Mesh"),
                                  ("floor", ROOT/layout["kitchen_usd"], "/World/Sink/BasinBottom")]:
        stage = Usd.Stage.Open(str(path))
        source[name] = dict(file=str(path), file_sha256=sha(path), **mesh_snapshot(stage.GetPrimAtPath(prim_path)))
    require(source["bowl"]["approximation"] == "convexDecomposition" and source["floor"]["approximation"] == "boundingCube",
            "Original collision approximations changed")
    return dict(schema_version=1, accepted=False, physical_trial_passed=False, diagnostic_only=True,
        cpu_contract_passed=True, actual_snapshot_created=False, declaration=declaration, installed_api=installed_api(),
        source=source, limitation="CPU source checks do not prove cooking success, actual native shape equivalence or stable support")


def array(value):
    import numpy as np
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    elif hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value).copy()


def query_cooking(stage, path, matrix, app, declaration):
    from pxr import UsdUtils, PhysicsSchemaTools
    from omni.physx import get_physx_cooking_interface
    from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import representation
    cooking = get_physx_cooking_interface()
    rows, callback_errors = [], []
    def callback(result, convexes):
        try:
            rows.append(representation(result.name, convexes, matrix))
        except Exception as error:
            callback_errors.append(repr(error))
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    request = cooking.request_convex_collision_representation(stage_id=stage_id,
        collision_prim_id=PhysicsSchemaTools.sdfPathToInt(path), run_asynchronously=True, on_result=callback)
    started, updates = time.monotonic(), 0
    while not rows and not callback_errors and updates < declaration["maximum_update_calls"] and time.monotonic()-started < declaration["maximum_wait_seconds"]:
        app.update()
        updates += 1
    if not rows and not callback_errors:
        cooking.cancel_collision_representation_task(task=request, invoke_callback=False)
        return dict(available=False, result="BOUNDED_TIMEOUT", hulls=[], hull_count=0,
            runtime_shape_equivalence_verified=False, update_calls=updates)
    if callback_errors:
        raise RuntimeError("Invalid cooking callback geometry: " + repr(callback_errors))
    if len(rows) != 1:
        raise RuntimeError("Cooking returned multiple callbacks")
    return dict(**rows[0], stage_id=stage_id, requested_collision_prim=path,
        update_calls=updates, wall_seconds=time.monotonic()-started)


def query_compound(stage, path, app, declaration):
    """A compound response has no native shape-slot IDs; keep that limit explicit."""
    from pxr import UsdUtils, PhysicsSchemaTools
    from omni.physx import get_physx_property_query_interface
    from omni.physx.bindings._physx import PhysxPropertyQueryMode
    rows, errors, finished = [], [], []
    def collider(info):
        try:
            row = dict(result=info.result.name, path_id=int(info.path_id), stage_id=int(info.stage_id))
            if info.result.name == "VALID":
                row.update(path=str(PhysicsSchemaTools.intToSdfPath(info.path_id)), volume=float(info.volume),
                    aabb_local_min=list(map(float, info.aabb_local_min)), aabb_local_max=list(map(float, info.aabb_local_max)),
                    local_pos=list(map(float, info.local_pos)), local_rot=list(map(float, info.local_rot)))
            rows.append(row)
        except Exception as error:
            errors.append(repr(error))
    # A static floor may be unsupported by this rigid-body query. Preserve its
    # failure; never apply RigidBodyAPI or otherwise change it for enumeration.
    get_physx_property_query_interface().query_prim(
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt(),
        prim_id=PhysicsSchemaTools.sdfPathToInt(path),
        query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
        timeout_ms=1000*declaration["maximum_wait_seconds"],
        finished_fn=lambda: finished.append(True), collider_fn=collider)
    started, updates = time.monotonic(), 0
    while not finished and not errors and updates < declaration["maximum_update_calls"] and time.monotonic()-started < declaration["maximum_wait_seconds"]:
        app.update(); updates += 1
    return dict(requested_path=path, finished=bool(finished), callbacks=rows, errors=errors, update_calls=updates,
        callback_count_is_shape_count=False, native_shape_slot_mapping_verified=False)


def layer_manifest(stage):
    from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import sha
    rows = []
    for layer in stage.GetUsedLayers():
        path = Path(layer.realPath) if layer.realPath else None
        rows.append(dict(identifier=layer.identifier, real_path=layer.realPath,
            file_sha256=sha(path) if path is not None and path.is_file() else None,
            composed_layer_text_sha256=hashlib.sha256(layer.ExportToString().encode()).hexdigest(),
            anonymous=bool(layer.anonymous)))
    return sorted(rows, key=lambda row: row["identifier"])


def run(args, provenance, cpu):
    import numpy as np
    import torch
    import carb
    import omni.kit.app
    import omni.timeline
    import omni.physx
    from omni.physx.bindings._physx import SETTING_NUM_THREADS
    from pxr import UsdGeom, UsdPhysics, Usd
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.replicator.task_environment import AssignedTaskCfg, AssignedTaskEnvironment
    from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import (
        BOWL_ROOT, require, sha, digest, native_inventory, transform_points,
    )
    torch.set_num_threads(args.torch_threads)
    carb.settings.get_settings().set_int(SETTING_NUM_THREADS, args.physics_threads)
    omni.physx.get_physx_interface().set_thread_count(args.physics_threads)
    result = dict(schema_version=1, status="initializing", accepted=False, physical_trial_passed=False,
        task_success=False, diagnostic_only=True, runtime_shape_equivalence_verified=False,
        explicit_control_actions=0, explicit_simulation_steps=0, snapshot_complete=False,
        declaration=cpu["declaration"], installed_api=cpu["installed_api"], targets={})
    env = None
    try:
        definition = AssignedTaskEnvironment().build(AssignedTaskCfg(task_id=TASK, variant="base", enable_cameras=True,
            episode_length_s=180., layout_path=str(args.layout)))
        provenance.bind_environment(definition)
        builder = ArenaEnvBuilder(definition, ArenaEnvBuilderCfg(num_envs=1, seed=42, device="cpu", presets="physx", solve_relations=False))
        cfg, kwargs = builder.compose_manager_cfg()
        cfg.observations.camera_obs = None
        cfg.sim.render_interval = cfg.decimation
        env = builder.make_registered(cfg, kwargs)
        env.reset()
        e = env.unwrapped
        # Pausing retains native views. No replay, pose writes, action or step.
        omni.timeline.get_timeline_interface().pause()
        stage, app = e.sim.stage, omni.kit.app.get_app()
        first_step = int(e.sim.get_physics_step_count())
        first_q = array(e.scene["robot"].data.joint_pos)
        first_pose = array(e.scene["bowl"].root_view.get_transforms())
        result.update(reset_physics_step=first_step, requested_device="cpu", cameras_enabled=True,
            live_physics_threads=carb.settings.get_settings().get(SETTING_NUM_THREADS), live_torch_threads=torch.get_num_threads(),
            stage_meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)), stage_up_axis=str(UsdGeom.GetStageUpAxis(stage)),
            background_physics=definition.background_physics_provenance, source_relation_audit=definition.source_relation_audit,
            native_bowl_pose_at_start=first_pose.tolist())
        require(result["stage_meters_per_unit"] == 1. and result["stage_up_axis"] == "Z", "Unreviewed stage units/axis")
        result["used_layers_at_start"] = layer_manifest(stage)
        # Preserve composed geometry independently of future cached overlays.
        flat = stage.Flatten()
        flat.Export(str(args.output/"composed_snapshot.usdc"))
        result["composed_snapshot_sha256"] = sha(args.output/"composed_snapshot.usdc")
        for name, path in cpu["declaration"]["targets"].items():
            prim = stage.GetPrimAtPath(path)
            mesh = mesh_snapshot(prim)
            require(mesh["source_geometry_sha256"] == cpu["source"][name]["source_geometry_sha256"],
                "Actual spawned collision mesh differs from declared source: " + name)
            require(mesh["approximation"] == cpu["source"][name]["approximation"] and mesh["collision_enabled"], "Collision approximation/enabled state changed")
            source_world = transform_points(mesh["geometry"]["points_local"], mesh["local_to_world_column_matrix"])
            mesh["source_bounds_world"] = [source_world.min(0).tolist(), source_world.max(0).tolist()]
            cooking = query_cooking(stage, path, mesh["local_to_world_column_matrix"], app, cpu["declaration"])
            result["targets"][name] = dict(source=mesh, cooking=cooking,
                source_file_sha256=cpu["source"][name]["file_sha256"])
            write(args.output/(name+"_cooking.json"), result["targets"][name])
        bowl_prim = stage.GetPrimAtPath(BOWL_ROOT)
        colliders = [str(prim.GetPath()) for prim in Usd.PrimRange(bowl_prim) if prim.HasAPI(UsdPhysics.CollisionAPI)]
        view = e.scene["bowl"].root_view
        offsets = {"rest_offsets": array(view.get_rest_offsets()), "contact_offsets": array(view.get_contact_offsets())}
        result["native_bowl_inventory"] = native_inventory(list(view.prim_paths), int(view.count), int(view.max_shapes), offsets,
            colliders, result["targets"]["bowl"]["cooking"].get("hull_count"))
        require(list(view.prim_paths) == [BOWL_ROOT] and colliders == [cpu["declaration"]["targets"]["bowl"]],
            "Ambiguous bowl rigid body/collision prim mapping")
        result["bowl_compound_query"] = query_compound(stage, BOWL_ROOT, app, cpu["declaration"])
        result["floor_compound_query"] = query_compound(stage, cpu["declaration"]["targets"]["floor"], app, cpu["declaration"])
        result["native_floor_inventory"] = dict(available=False, actual_shape_count=None, rest_offsets=None,
            contact_offsets=None, reason="Original static collider has no RigidBodyView; no schema/state mutation to force one")
        result["used_layers_at_end"] = layer_manifest(stage)
        result["last_physics_step"] = int(e.sim.get_physics_step_count())
        result["native_bowl_pose_at_end"] = array(view.get_transforms()).tolist()
        result["paused_snapshot_checks"] = dict(no_physics_progress=result["last_physics_step"] == first_step,
            unchanged_native_bowl_pose=np.array_equal(first_pose, result["native_bowl_pose_at_end"]),
            unchanged_robot_q=np.array_equal(first_q, array(e.scene["robot"].data.joint_pos)),
            unchanged_layer_contents=result["used_layers_at_start"] == result["used_layers_at_end"])
        require(all(result["paused_snapshot_checks"].values()), "Snapshot query changed pose, step count or composed stage")
        result["snapshot_complete"] = True
        result["status"] = "snapshot_complete"
        result["scope"] = "Fresh identical-source reset stage and copied public cooking representation. No historical pose replay; no native slot-to-hull identity or running shape geometric equality claim. Unsupported floor data remains unavailable."
    except Exception as error:
        result.update(status="failed", error=repr(error))
    finally:
        if env is not None and provenance.scene_bound:
            try:
                report = provenance.finish_task(args.output, env.unwrapped.sim.get_physics_step_count())
                result["provenance_passed"] = report["passed"]
                if not report["passed"]:
                    result.update(status="failed", provenance_error="Source changed during snapshot")
            except Exception as error:
                result.update(status="failed", provenance_error=repr(error))
        if env is not None:
            try:
                env.close()
            except Exception as error:
                result.update(status="failed", close_error=repr(error))
        write(args.output/"snapshot.json", result)
    print(json.dumps({key: result.get(key) for key in ("status", "accepted", "snapshot_complete", "error")}), flush=True)
    return 0 if result["status"] == "snapshot_complete" else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--layout", type=Path, default=ROOT/"configs/extensions/replicator/g_shape_bowl_sink_source_layout.json")
    parser.add_argument("--declaration", type=Path, default=ROOT/"configs/extensions/replicator/g_bowl_cooked_snapshot.json")
    parser.add_argument("--device", choices=["cpu"], default="cpu")
    parser.add_argument("--physics-threads", type=int, choices=[2], default=2)
    parser.add_argument("--torch-threads", type=int, choices=[2], default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    args.output, args.layout, args.declaration = args.output.resolve(), args.layout.resolve(), args.declaration.resolve()
    if not args.output.is_relative_to(ROOT/"reports"):
        parser.error("Snapshot output must be a new reports directory, never a dataset/success directory")
    args.output.mkdir(parents=True, exist_ok=False)
    if args.dry_run:
        cpu = preflight(args)
        write(args.output/"cpu_preflight.json", cpu)
        print(json.dumps(dict(cpu_contract_passed=True, actual_snapshot_created=False, accepted=False)), flush=True)
        return 0
    import fcntl
    import os
    lock_path = Path("/workspace/reports/skill_trials/gpu.lock")
    if ROOT != Path("/workspace") or not lock_path.exists():
        raise RuntimeError("Use verified /workspace mount and existing shared GPU lock")
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        from kitchen_safety.extensions.replicator.task_collection_evidence import TaskRunProvenance
        provenance = TaskRunProvenance(args.layout, collector_path=__file__)
        provenance.capture(EXTRA_SOURCE, "execution_source", snapshot=True)
        provenance.capture(args.declaration, "diagnostic_declaration_input", snapshot=True)
        api = installed_api()
        api_output = args.output/"installed_api_sources"
        api_output.mkdir()
        for name, value in api.items():
            if "path" in value:
                # Installed tests include CRLF. Preserve original bytes rather
                # than normalized read_text snapshots from the runtime helper.
                provenance.capture(value["path"], "installed_api_definition", expected_sha256=value["sha256"])
                (api_output/(name+Path(value["path"]).suffix)).write_bytes(Path(value["path"]).read_bytes())
        # Actual runs must not import pxr before SimulationApp. The standalone
        # --dry-run is intentionally a separate CPU process. Hashes/JSON/API
        # source bytes are captured here; composed USD reads follow Kit start.
        from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import read_declaration
        read_declaration(args.declaration, ROOT)
        write(args.output/"pre_app_provenance.json", provenance.check())
        from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
        from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
        os.environ["HEADLESS"] = "1"
        runtime_args = get_isaaclab_arena_cli_parser().parse_args(["--enable_cameras", "--device", "cpu", "--seed", "42"])
        with SimulationAppContext(runtime_args):
            cpu = preflight(args)
            write(args.output/"cpu_preflight.json", cpu)
            code = run(args, provenance, cpu)
            if code:
                raise RuntimeError("Cooking snapshot failed; retained " + str(args.output/"snapshot.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
