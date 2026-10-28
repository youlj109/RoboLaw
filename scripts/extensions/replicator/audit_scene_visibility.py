#!/usr/bin/env python3
"""Re-render loaded scenes with per-object visibility; only change the camera.

The first scene builds remain immutable. Every selected v2 USD must match the
original v1 USD byte-for-byte after removing the overview-camera prim from both
Sdf layers. This is an actual RTX image check, never a physics/trajectory check.
"""
from __future__ import annotations
import json
import math
import os
import shutil
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def noncamera_text(text):
    from pxr import Sdf
    layer = Sdf.Layer.CreateAnonymous("noncamera.usda")
    if not layer.ImportFromString(text):
        raise ValueError("Could not parse scene layer")
    edit = Sdf.BatchNamespaceEdit()
    edit.Add("/World/overview_camera", Sdf.Path.emptyPath)
    if not layer.Apply(edit):
        raise ValueError("Could not remove camera from comparison layer")
    return layer.ExportToString()


def candidates(layout, previous):
    base = list(layout["robot_position"])
    yaw = math.radians(layout["robot_yaw_deg"])
    forward, side = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
    positions = [base, layout["obstacle"]["position"],
                 *(item["position"] for item in layout["objects"].values())]
    target = [sum(p[i] for p in positions) / len(positions) for i in range(2)] + [1.0]
    rows = [(previous["camera_eye"], previous["camera_target"])]
    if layout["obstacle"]["category"] == "human":
        # Frame the entire referenced person, not a few hand/leg pixels.
        hazard = layout["obstacle"]["position"]
        object_center = [sum(item["position"][i] for item in layout["objects"].values()) / len(layout["objects"]) for i in range(2)]
        target = [.5*hazard[i] + .25*base[i] + .25*object_center[i] for i in range(2)] + [.9]
        rows = []
        for back,sideways,height in ((1.,.4,2.8),(1.,-.4,2.8),(.3,.2,3.3),(.6,-.3,3.3),(1.3,.2,3.1),(1.3,-.4,3.1)):
            rows.append(([target[0]-forward[0]*back+side[0]*sideways,
                          target[1]-forward[1]*back+side[1]*sideways,height],target))
        return rows
    for back, sideways, height in ((.45,-.55,2.35),(.65,.75,2.65),(.65,-.75,2.65),(-.05,.15,3.1),(1.1,-.4,2.1)):
        rows.append(([base[0] - forward[0]*back + side[0]*sideways,
                      base[1] - forward[1]*back + side[1]*sideways, height], target))
    return rows


def author_camera(source_text, destination, eye, target, *, focal_length=None, fit_human=False):
    from pxr import Gf, Sdf, Usd, UsdGeom
    destination.parent.mkdir(parents=True, exist_ok=True)
    layer = Sdf.Layer.CreateNew(str(destination))
    layer.ImportFromString(source_text)
    stage = Usd.Stage.Open(layer)
    camera = UsdGeom.Xformable(stage.GetPrimAtPath("/World/overview_camera"))
    ops = camera.GetOrderedXformOps()
    if len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
        raise ValueError("Expected exactly one source camera transform")
    ops[0].Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0,0,1)).GetInverse())
    if focal_length is not None:
        UsdGeom.Camera(stage.GetPrimAtPath("/World/overview_camera")).GetFocalLengthAttr().Set(float(focal_length))
    if fit_human:
        direction = [eye[i]-target[i] for i in range(3)]
        for fit_step in range(24):
            if human_framing(stage)["passed"]:
                break
            distance_scale = 1.1 ** (fit_step+1)
            eye = [target[i] + direction[i]*distance_scale for i in range(3)]
            ops[0].Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0,0,1)).GetInverse())
        else:
            raise ValueError("Could not frame original human bounds with camera-only fitting")
    layer.Save()
    if noncamera_text(source_text) != noncamera_text(destination.read_text()):
        raise ValueError("Camera adaptation changed a non-camera USD field")
    return list(eye)


def pixels(segmentation, layout):
    import numpy as np
    if not isinstance(segmentation, dict) or "data" not in segmentation:
        raise ValueError("No actual instance-id segmentation buffer")
    data = np.asarray(segmentation["data"])
    labels = segmentation.get("info", {}).get("idToLabels", {})
    prefixes = {"robot": "/World/robot/", "hazard": "/World/hazard/"}
    prefixes.update({"object:" + name: "/World/objects/" + name + "/" for name in layout["objects"]})
    counts = {}
    for name, prefix in prefixes.items():
        ids = [int(value) for value, label in labels.items() if prefix in str(label)]
        counts[name] = int(np.isin(data, ids).sum())
    return counts


def human_framing(stage):
    from pxr import Usd, UsdGeom
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        stage.GetPrimAtPath("/World/hazard")).ComputeAlignedRange()
    camera = UsdGeom.Camera(stage.GetPrimAtPath("/World/overview_camera")).GetCamera(Usd.TimeCode.Default())
    frustum = camera.frustum
    world_to_ndc = frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()
    projected = [list(world_to_ndc.Transform(bounds.GetCorner(i))) for i in range(8)]
    passed = all(-.98 <= p[0] <= .98 and -.98 <= p[1] <= .98 and -1. <= p[2] <= 1. for p in projected)
    return {"passed": passed, "world_bounds_min":list(bounds.GetMin()), "world_bounds_max":list(bounds.GetMax()),
            "projected_bbox_corners_ndc":projected, "image_border_margin_fraction":.01}


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    from kitchen_safety.extensions.replicator.scene_build_runtime import (
        SCOPE, atomic_json, inventory, relative, sha, stamp, update_progress)
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument("--progress", type=Path, default=ROOT / "reports/replicator_scene_build/progress.json")
    parser.add_argument("--source", type=Path, default=ROOT / "reports/replicator_scene_build/scenes")
    parser.add_argument("--output", type=Path, default=ROOT / "reports/replicator_scene_build/scenes_v2")
    parser.add_argument("--minimum-pixels", type=int, default=20)
    parser.add_argument("--task", default="")
    parser.add_argument("--variant", default="")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    for name in ("progress", "source", "output"):
        setattr(args, name, getattr(args, name).resolve())
    if args.minimum_pixels < 20:
        raise ValueError("Visibility requires at least 20 actual pixels per entity")
    args.enable_cameras = True
    os.environ.setdefault("ISAACLAB_ARENA_FORCE_EXIT_ON_COMPLETE", "1")
    rows = json.loads(args.progress.read_text())["entries"]
    rows = [row for row in rows if (not args.task or args.task in row["task_id"])
            and (not args.variant or row["variant_id"] == args.variant)]
    with SimulationAppContext(args) as context:
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        app, kit = context.app_launcher.app, omni.usd.get_context()
        timeline = omni.timeline.get_timeline_interface()
        for row in rows:
            short = row["task_id"].removeprefix("kitchen_bench_replicator_")
            old_dir, output = args.source / short / row["variant_id"], args.output / short / row["variant_id"]
            evidence_path, final_scene, final_image = output / "simulation_load.json", output / "scene.usda", output / "preview.png"
            config = ROOT / row["config_path"]
            if args.resume and evidence_path.exists():
                done = json.loads(evidence_path.read_text())
                if (done.get("checks", {}).get("each_entity_visible") is True
                    and final_scene.is_file() and final_image.is_file()
                    and done["scene_sha256"] == sha(final_scene) and done["preview_sha256"] == sha(final_image)
                    and done["layout_sha256"] == sha(config)
                    and all(sha(ROOT / path) == value for path,value in done["source_paths_sha256"].items())):
                    update_progress(args.progress,row,status="ready",scene_path=relative(final_scene),preview_path=relative(final_image),
                        evidence_path=relative(evidence_path),checks=done["checks"],error=None)
                    continue
            update_progress(args.progress, row, status="loading", error=None)
            started = time.monotonic()
            tries = []
            try:
                old = json.loads((old_dir / "simulation_load.json").read_text())
                layout = json.loads(config.read_text())
                source_path = old_dir / "scene.usda"
                source_text = source_path.read_text()
                if old["scene_sha256"] != sha(source_path) or old["layout_sha256"] != sha(config):
                    raise ValueError("First-pass scene/config hashes changed")
                if not all(sha(ROOT / p) == digest for p,digest in old["source_paths_sha256"].items()):
                    raise ValueError("Original source asset changed")
                chosen = None
                for attempt, (eye,target) in enumerate(candidates(layout,old)):
                    candidate = output / f"camera_candidate_{attempt:02d}.usda"
                    if candidate.exists():
                        raise FileExistsError(f"Refusing to overwrite prior camera evidence: {candidate}")
                    is_human = layout["obstacle"]["category"] == "human"
                    eye = author_camera(source_text,candidate,eye,target,focal_length=18. if is_human else None,fit_human=is_human)
                    timeline.stop()
                    if not kit.open_stage(str(candidate)):
                        raise ValueError("Isaac could not load candidate scene")
                    for _ in range(240):
                        app.update()
                        if kit.get_stage_loading_status()[2] == 0:
                            break
                    else:
                        raise TimeoutError("Assets did not finish loading")
                    actual = inventory(kit.get_stage())
                    if actual != old["inventory"]:
                        raise ValueError("Non-camera scene inventory changed")
                    product = rep.create.render_product("/World/overview_camera",(960,640))
                    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb",device="cpu")
                    id_annotator = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation_fast",init_params={"colorize":False},device="cpu")
                    rgb_annotator.attach(product); id_annotator.attach(product)
                    try:
                        for _ in range(24):
                            app.update()
                        rgb = np.asarray(rgb_annotator.get_data())
                        counts = pixels(id_annotator.get_data(),layout)
                        if rgb.shape != (640,960,4) or not np.isfinite(rgb).all() or float(rgb[:,:,:3].std()) <= 2:
                            raise ValueError("Actual RGB is blank/invalid")
                        if timeline.is_playing():
                            raise ValueError("Timeline played during render-only audit")
                        image_path = output / f"camera_candidate_{attempt:02d}.png"
                        Image.fromarray(rgb[:,:,:3].astype(np.uint8)).save(image_path)
                        framing = human_framing(kit.get_stage()) if is_human else {"passed": True, "scope": "not a human scene"}
                        passed = all(count >= args.minimum_pixels for count in counts.values()) and framing["passed"]
                        trial = {"candidate":attempt,"eye":eye,"target":target,"counts":counts,"passed":passed,"human_framing":framing,
                                 "scene_sha256":sha(candidate),"preview_sha256":sha(image_path),
                                 "scene_path":relative(candidate),"preview_path":relative(image_path)}
                        tries.append(trial)
                        if passed:
                            chosen = (candidate,image_path,trial,actual,float(rgb[:,:,:3].std()))
                    finally:
                        id_annotator.detach(); rgb_annotator.detach(); product.destroy()
                        for _ in range(2): app.update()
                    if chosen:
                        break
                if not chosen:
                    raise ValueError(f"No candidate exposes every original task object/robot/hazard at >= {args.minimum_pixels} pixels; see candidate evidence")
                candidate,image_path,trial,actual,rgb_std = chosen
                shutil.copyfile(candidate,final_scene);shutil.copyfile(image_path,final_image)
                if sha(candidate) != sha(final_scene) or noncamera_text(source_text) != noncamera_text(final_scene.read_text()):
                    raise ValueError("Final scene differs from loaded candidate or original non-camera fields")
                checks = dict(old["checks"], each_entity_visible=True, human_fully_in_frame=trial["human_framing"]["passed"], noncamera_fields_unchanged=True,
                              original_source_assets_unchanged=all(sha(ROOT / path)==value for path,value in old["source_paths_sha256"].items()))
                evidence = dict(old,created_at=stamp(),kind="isaac_scene_load_render_per_entity",checks=checks,
                    scene_path=relative(final_scene),preview_path=relative(final_image),scene_sha256=sha(final_scene),
                    preview_sha256=sha(final_image),camera_eye=trial["eye"],camera_target=trial["target"],
                    actual_loaded_candidate=trial["scene_path"],actual_loaded_candidate_sha256=trial["scene_sha256"],
                    previous_scene_path=relative(source_path),previous_scene_sha256=sha(source_path),
                    visibility_pixels=trial["counts"],minimum_pixels_per_entity=args.minimum_pixels,
                    visibility_attempts=tries,rgb_std=rgb_std,elapsed_seconds=time.monotonic()-started,
                    accepted=False,task_success=False,physical_trial_passed=False)
                atomic_json(evidence_path,evidence)
                update_progress(args.progress,row,status="ready",scene_path=relative(final_scene),preview_path=relative(final_image),
                    evidence_path=relative(evidence_path),checks=checks,error=None)
                print("SCENE_VISIBLE",short,row["variant_id"],trial["candidate"],trial["counts"],flush=True)
            except Exception as error:
                detail=f"{type(error).__name__}: {error}"
                atomic_json(evidence_path,dict(schema_version=1,kind="isaac_scene_load_render_per_entity",scope=SCOPE,
                    task_id=row["task_id"],variant_id=row["variant_id"],created_at=stamp(),checks={"simulation_loaded":False,"each_entity_visible":False},
                    visibility_attempts=tries,error=detail,traceback=traceback.format_exc(),accepted=False,task_success=False,physical_trial_passed=False))
                update_progress(args.progress,row,status="failed",evidence_path=relative(evidence_path),checks={"simulation_loaded":False,"each_entity_visible":False},error=detail)
                print("SCENE_VISIBILITY_FAILED",short,row["variant_id"],detail,flush=True)
        print("SCENE_VISIBILITY_FINISHED",len(rows),flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
