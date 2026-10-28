#!/usr/bin/env python3
"""Really load/render standalone Replicator scene variants; never collect data."""
from __future__ import annotations

import argparse
import json
import os
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    from kitchen_safety.extensions.replicator.scene_build_runtime import (
        REQUIRED_LOAD_CHECKS, SCOPE, atomic_json, compose, inventory, relative, sha, stamp, update_progress)
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument("--manifest", type=Path, default=ROOT / "configs/extensions/replicator/scene_variants/manifest.json")
    parser.add_argument("--progress", type=Path, default=ROOT / "reports/replicator_scene_build/progress.json")
    parser.add_argument("--output", type=Path, default=ROOT / "reports/replicator_scene_build/scenes")
    parser.add_argument("--task", default="")
    parser.add_argument("--variant", default="")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--render-subframes", type=int, default=6)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=640)
    args = parser.parse_args()
    for name in ("manifest", "progress", "output"):
        setattr(args, name, getattr(args, name).resolve())
    args.enable_cameras = True
    os.environ.setdefault("ISAACLAB_ARENA_FORCE_EXIT_ON_COMPLETE", "1")
    manifest = json.loads(args.manifest.read_text())
    rows = manifest.get("entries", manifest.get("scenes"))
    if not rows:
        raise ValueError("Manifest contains no entries/scenes")
    selected = [row for row in rows if (not args.task or args.task in row["task_id"])
                and (not args.variant or args.variant == row["variant_id"])]
    if args.limit:
        selected = selected[:args.limit]
    if not selected:
        raise ValueError("No matching scenes")
    with SimulationAppContext(args) as context:
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        app = context.app_launcher.app
        kit_context = omni.usd.get_context()
        timeline = omni.timeline.get_timeline_interface()
        for row in selected:
            short = row["task_id"].removeprefix("kitchen_bench_replicator_")
            destination = args.output / short / row["variant_id"]
            scene_path, preview_path = destination / "scene.usda", destination / "preview.png"
            evidence_path = destination / "simulation_load.json"
            config_path = ROOT / row.get("config_path", row.get("layout_path", ""))
            if args.resume and evidence_path.exists():
                old = json.loads(evidence_path.read_text())
                if (all(old.get("checks", {}).get(key) is True for key in REQUIRED_LOAD_CHECKS)
                    and not old.get("error") and not old.get("visual_review_failed")
                    and preview_path.exists() and scene_path.exists()
                    and old.get("layout_sha256") == sha(config_path) and old.get("scene_sha256") == sha(scene_path)
                    and old.get("preview_sha256") == sha(preview_path)
                    and old.get("source_paths_sha256")
                    and all((ROOT / path).is_file() and sha(ROOT / path) == value
                            for path, value in old["source_paths_sha256"].items())):
                    update_progress(args.progress, row, status="ready", scene_path=relative(scene_path),
                        preview_path=relative(preview_path), evidence_path=relative(evidence_path), checks=old["checks"], error=None)
                    continue
            started = time.monotonic()
            product = annotator = ids_annotator = None
            update_progress(args.progress, row, status="loading", config_path=relative(config_path), error=None)
            try:
                if scene_path.exists():
                    scene_path.unlink()
                composition = compose(config_path, scene_path)
                timeline.stop()
                if not kit_context.open_stage(str(scene_path.resolve())):
                    raise RuntimeError("Isaac Sim could not open exported scene")
                for _ in range(240):
                    app.update()
                    if kit_context.get_stage_loading_status()[2] == 0:
                        break
                else:
                    raise TimeoutError("Stage asset loading did not finish within 240 updates")
                live_stage = kit_context.get_stage()
                actual_inventory = inventory(live_stage)
                if actual_inventory != composition["inventory"]:
                    raise ValueError("Live Isaac stage physics/mesh inventory differs from exported scene")
                product = rep.create.render_product("/World/overview_camera", (args.width, args.height))
                annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
                annotator.attach(product)
                ids_annotator = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation_fast",
                    init_params={"colorize": False}, device="cpu")
                ids_annotator.attach(product)
                # RTX annotators update on Kit renders, exactly as Isaac Lab's
                # camera renderer does. The stopped timeline remains at time 0.
                for _ in range(3):
                    for _ in range(max(16, args.render_subframes * 3)):
                        app.update()
                    rgb = np.asarray(annotator.get_data())
                    if rgb.shape == (args.height, args.width, 4) and np.isfinite(rgb).all() and float(rgb[:, :, :3].std()) > 2:
                        break
                if rgb.shape != (args.height, args.width, 4) or not np.isfinite(rgb).all() or float(rgb[:, :, :3].std()) <= 2:
                    raise ValueError(f"Missing/blank actual RTX RGB: shape={rgb.shape}")
                if timeline.is_playing():
                    raise RuntimeError("Timeline unexpectedly playing during render-only scene build")
                segmentation = ids_annotator.get_data()
                visibility = {}
                if isinstance(segmentation, dict) and "data" in segmentation:
                    label_map = segmentation.get("info", {}).get("idToLabels", {})
                    for group in ("robot", "objects", "hazard"):
                        ids = [int(value) for value, label in label_map.items() if f"/World/{group}/" in str(label)]
                        visibility[group] = int(np.isin(np.asarray(segmentation["data"]), ids).sum())
                else:
                    label_map = {}
                Image.fromarray(rgb[:, :, :3].astype(np.uint8)).save(preview_path)
                checks = {"simulation_loaded": True, "render_nonempty": True,
                          "source_assets_unchanged": all(sha(ROOT / path) == value for path, value in composition["source_paths_sha256"].items()),
                          "robot_articulation_present": actual_inventory["robot"]["articulation_count"] > 0,
                          "task_objects_rigid_present": actual_inventory["objects"]["rigid_body_count"] > 0,
                          "hazard_collision_enabled": actual_inventory["hazard"]["enabled_collision_count"] > 0,
                          "physics_not_stepped": True, "native_physics_initialized": False,
                          "task_success_verified": False, "avoidance_verified": False}
                if not checks["source_assets_unchanged"]:
                    raise ValueError("Source asset changed during scene loading")
                evidence = dict(schema_version=1, kind="isaac_scene_load_render", scope=SCOPE,
                    task_id=row["task_id"], variant_id=row["variant_id"], created_at=stamp(),
                    config_path=relative(config_path), scene_path=relative(scene_path), preview_path=relative(preview_path),
                    checks=checks, **composition, live_inventory=actual_inventory,
                    visibility_pixels=visibility, segmentation_labels=label_map,
                    preview_sha256=sha(preview_path), rgb_shape=list(rgb.shape), rgb_std=float(rgb[:, :, :3].std()),
                    physics_steps_requested=0, timeline_playing=False, elapsed_seconds=time.monotonic() - started,
                    accepted=False, task_success=False, physical_trial_passed=False)
                atomic_json(evidence_path, evidence)
                update_progress(args.progress, row, status="ready", scene_path=relative(scene_path),
                    preview_path=relative(preview_path), evidence_path=relative(evidence_path), checks=checks, error=None)
                print("SCENE_READY", row["task_id"], row["variant_id"], round(evidence["elapsed_seconds"], 2), flush=True)
            except Exception as error:
                detail = f"{type(error).__name__}: {error}"
                atomic_json(evidence_path, dict(schema_version=1, kind="isaac_scene_load_render", scope=SCOPE,
                    task_id=row["task_id"], variant_id=row["variant_id"], created_at=stamp(),
                    checks={"simulation_loaded": False}, error=detail, traceback=traceback.format_exc(),
                    accepted=False, task_success=False, physical_trial_passed=False))
                update_progress(args.progress, row, status="failed", evidence_path=relative(evidence_path),
                    checks={"simulation_loaded": False}, error=detail)
                print("SCENE_FAILED", row["task_id"], row["variant_id"], detail, flush=True)
            finally:
                if ids_annotator is not None:
                    ids_annotator.detach()
                if annotator is not None:
                    annotator.detach()
                if product is not None:
                    product.destroy()
                timeline.stop()
                for _ in range(2):
                    app.update()
        print("SCENE_BUILD_FINISHED", len(selected), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
