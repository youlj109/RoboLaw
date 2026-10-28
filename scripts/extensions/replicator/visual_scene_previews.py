#!/usr/bin/env python3
"""Render camera-only overview/side/top inspection views from any scene manifest.

--prepare-only performs CPU USD composition/camera ranking, without Kit or GPU.
Normal rendering never steps physics, hides an entity, or moves scene geometry.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import atomic_json, compose, inventory, relative, sha, stamp
from scripts.extensions.replicator.visual_camera_override import validate_camera_overrides, override_fingerprint, inspection_entities

SCOPE = 'Render-only camera inspection: source-authored robot pose, no native physics, no trajectory, no safety acceptance.'
EXPOSURE_KEYS = ('/rtx/post/tonemap/op', '/rtx/post/tonemap/exposureTime', '/rtx/post/tonemap/filmIso',
                 '/rtx/post/tonemap/fNumber', '/rtx/post/tonemap/responsivity', '/rtx/post/histogram/enabled')


def exposure_override(original, ev):
    """Installed RTX post_widgets.py documents exposureTime and Clamp bypass."""
    exposure = original.get('/rtx/post/tonemap/exposureTime')
    if not isinstance(exposure, (int, float)) or exposure <= 0:
        raise ValueError('Renderer does not expose a valid original exposureTime')
    values = dict(original)
    values['/rtx/post/tonemap/exposureTime'] = exposure * 2 ** ev
    values['/rtx/post/histogram/enabled'] = False
    if values['/rtx/post/tonemap/op'] == 0:
        values['/rtx/post/tonemap/op'] = 1
    return values


def update(path, row, **changes):
    data = json.loads(path.read_text()) if path.exists() else dict(schema_version=1, total=154, entries=[])
    entry = next((e for e in data['entries'] if e['scene_id'] == row['scene_id']), None)
    if entry is None:
        entry = {k: row.get(k) for k in ('scene_id', 'task_id', 'variant_id', 'category', 'config_path')}
        data['entries'].append(entry)
    entry.update(changes, updated_at=stamp(), render_only=True, accepted=False)
    data.update(stage='multi_view_rendering', updated_at=stamp(), scope=SCOPE)
    data['counts'] = {state: sum(e.get('render_status') == state for e in data['entries'])
                      for state in ('prepared', 'rendering', 'ready', 'needs_view_review', 'failed')}
    atomic_json(path, data)


def prepare_row(row, output, width, height, candidates_per_kind):
    from pxr import Usd
    from scripts.extensions.replicator.visual_scene_camera import landmarks, camera_candidates
    config = (ROOT / row['config_path']).resolve()
    layout = json.loads(config.read_text())
    destination = output / row['task_id'].removeprefix('kitchen_bench_replicator_') / row['variant_id']
    destination.mkdir(parents=True, exist_ok=True)
    source, preparation = destination / 'source_scene.usda', destination / 'preparation.json'
    if preparation.exists():
        previous = json.loads(preparation.read_text())
        if (previous['config_sha256'] != sha(config) or previous['source_scene_sha256'] != sha(source)
                or previous.get('camera_overrides_sha256') != override_fingerprint(row)
                or previous['width'] != width or previous['height'] != height
                or any(sha(ROOT / p) != digest for p, digest in previous['composition']['source_paths_sha256'].items())):
            raise ValueError('Prepared scene inputs changed; use a new --output to preserve evidence')
        return destination, layout, previous
    if source.exists():
        raise FileExistsError('Incomplete earlier composition retained; use a new --output: ' + str(source))
    composition = compose(config, source)
    stage = Usd.Stage.Open(str(source))
    info = landmarks(stage, layout)
    cameras = camera_candidates(info, layout, width, height, candidates_per_kind)
    info['entities'].update(inspection_entities(stage,row.get('inspection_prims',[])))
    if row.get('camera_overrides'):
        overrides=validate_camera_overrides(row['camera_overrides'])
        overridden={c['kind'] for c in overrides}
        cameras=[c for c in cameras if c['kind'] not in overridden]+overrides
        if any(name not in info['entities'] for c in overrides for name in c.get('required_entities',[])):
            raise ValueError('Unknown required inspection entity')
    previous = dict(schema_version=1, scope=SCOPE, config_path=relative(config), config_sha256=sha(config),
                    source_scene_path=relative(source), source_scene_sha256=sha(source), composition=composition,
                    entities=info['entities'], camera_candidates=cameras, width=width, height=height,
                    camera_overrides_sha256=override_fingerprint(row),
                    camera_overrides_source_path=row.get('camera_overrides_source_path'),
                    camera_overrides_source_sha256=row.get('camera_overrides_source_sha256'),
                    camera_ranking='Projected object area/framing and conservative kitchen/robot mesh-AABB ray occlusion; ranking only, final visibility comes from actual pixels.',
                    native_physics_verified=False, accepted=False)
    atomic_json(preparation, previous)
    return destination, layout, previous


def render_rows(args, rows):
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    # All USD/Replicator imports used for rendering occur after AppLauncher.
    args.headless, args.enable_cameras = True, True
    os.environ.setdefault('ISAACLAB_ARENA_FORCE_EXIT_ON_COMPLETE', '1')
    with SimulationAppContext(args) as context:
        import numpy as np
        import carb.settings
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from scripts.extensions.replicator.retry_scene_visibility import author_camera, noncamera_text
        from scripts.extensions.replicator.visual_scene_camera import measure_visibility, multi_view_summary
        app, kit = context.app_launcher.app, omni.usd.get_context()
        timeline = omni.timeline.get_timeline_interface()
        settings = carb.settings.get_settings()
        for row in rows:
            product = rgb_annotator = segmentation_annotator = None
            original_settings = None
            destination = args.output / row['task_id'].removeprefix('kitchen_bench_replicator_') / row['variant_id']
            update(args.progress, row, render_status='rendering')
            views, started = [], time.monotonic()
            try:
                destination, layout, prepared = prepare_row(row, args.output, args.width, args.height, args.candidates_per_kind)
                evidence = destination / 'multi_view.json'
                if args.resume and evidence.exists():
                    saved = json.loads(evidence.read_text())
                    if (saved.get('config_sha256') == sha(ROOT / row['config_path'])
                            and saved.get('readability', {}).get('readability_passed')
                            and all(sha(ROOT / v['preview_path']) == v['preview_sha256'] for v in saved['views'])):
                        update(args.progress, row, render_status='ready', readability_passed=True,
                               evidence_path=relative(evidence), evidence_sha256=sha(evidence), views=saved['selected_views'],
                               preview_path=saved['selected_views'][0]['preview_path'], config_sha256=prepared['config_sha256'])
                        continue
                source = ROOT / prepared['source_scene_path']
                source_text = source.read_text()
                timeline.stop()
                print('VISUAL_STAGE_OPEN', row['scene_id'], flush=True)
                if not kit.open_stage(str(source)):
                    raise ValueError('Could not open composed stage')
                for _ in range(360):
                    app.update()
                    if kit.get_stage_loading_status()[2] == 0:
                        break
                else:
                    raise TimeoutError('Stage did not finish loading')
                # Let outstanding stage/renderer events settle before creating
                # SyntheticData graphs. Previous crash occurred on its first tick.
                for _ in range(24):
                    app.update()
                stage = kit.get_stage()
                original_settings = {key: settings.get(key) for key in EXPOSURE_KEYS}
                if inventory(stage) != prepared['composition']['inventory']:
                    raise ValueError('Scene inventory changed after loading')
                product = rep.create.render_product('/World/overview_camera', (args.width, args.height))
                rgb_annotator = rep.AnnotatorRegistry.get_annotator('rgb', device='cpu')
                rgb_annotator.attach(product)
                print('VISUAL_RGB_WARMUP', row['scene_id'], flush=True)
                for _ in range(24):
                    app.update()
                segmentation_name = 'instance_id_segmentation_fast' if args.segmentation == 'fast' else 'instance_id_segmentation'
                segmentation_annotator = rep.AnnotatorRegistry.get_annotator(segmentation_name,
                    init_params={'colorize': False}, device='cpu')
                segmentation_annotator.attach(product)
                print('VISUAL_SEGMENTATION_WARMUP', row['scene_id'], segmentation_name, flush=True)
                for _ in range(12):
                    app.update()
                selected = []
                for kind in ('overview', 'operation_side', 'operation_top'):
                    kind_views = []
                    for camera in [c for c in prepared['camera_candidates'] if c['kind'] == kind]:
                        for key, value in original_settings.items():
                            if value is not None:
                                settings.set(key, value)
                        view_id = f'{kind}_{len(views):02d}'
                        while (destination / (view_id + '.usda')).exists():
                            view_id += '_retry'
                        scene_path, image_path = destination / (view_id + '.usda'), destination / (view_id + '.png')
                        author_camera(source_text, scene_path, camera['eye'], camera['target'], focal_length=camera['focal_length_mm'])
                        # Keep one stage and one render product alive for all
                        # candidates. Only camera opinions enter the session layer.
                        with Usd.EditContext(stage, stage.GetSessionLayer()):
                            cam = UsdGeom.Camera(stage.GetPrimAtPath('/World/overview_camera'))
                            UsdGeom.Xformable(cam).GetOrderedXformOps()[0].Set(Gf.Matrix4d().SetLookAt(
                                Gf.Vec3d(*camera['eye']), Gf.Vec3d(*camera['target']), Gf.Vec3d(0, 0, 1)).GetInverse())
                            cam.GetFocalLengthAttr().Set(camera['focal_length_mm'])
                        print('VISUAL_CAPTURE', row['scene_id'], view_id, flush=True)
                        for _ in range(args.warmup_frames):
                            app.update()
                        rgb = np.asarray(rgb_annotator.get_data()).copy()
                        valid = rgb.shape == (args.height, args.width, 4) and np.isfinite(rgb).all() and float(rgb[:, :, :3].std()) > 2
                        if not valid:
                            raise ValueError('Blank/invalid actual RTX RGB: ' + str(rgb.shape))
                        if timeline.is_playing():
                            raise ValueError('Timeline played during render-only inspection')
                        visibility = measure_visibility(segmentation_annotator.get_data(), rgb, prepared['entities'], camera, args.width, args.height)
                        original_white = float((rgb[:, :, :3] >= 250).all(axis=2).mean())
                        exposure_record = dict(original=original_settings, applied=original_settings,
                            requested_ev=0, original_near_white_fraction=original_white, retry_performed=False)
                        needs_exposure = original_white > args.near_white_threshold or any(
                            (m['near_white_fraction'] or 0) > .65 for name, m in visibility.items() if name != 'robot')
                        requested_ev=camera.get('exposure_ev',args.exposure_retry_ev)
                        forced_exposure='exposure_ev' in camera and requested_ev<0
                        if (needs_exposure or forced_exposure) and requested_ev < 0:
                            original_image = destination / (view_id + '_original_exposure.png')
                            Image.fromarray(rgb[:, :, :3].astype(np.uint8)).save(original_image)
                            adjusted = exposure_override(original_settings, requested_ev)
                            for key, value in adjusted.items():
                                if value is not None:
                                    settings.set(key, value)
                            for _ in range(args.warmup_frames):
                                app.update()
                            adjusted_rgb = np.asarray(rgb_annotator.get_data()).copy()
                            if adjusted_rgb.shape != rgb.shape or not np.isfinite(adjusted_rgb).all():
                                raise ValueError('Invalid lower-exposure RGB')
                            adjusted_white = float((adjusted_rgb[:, :, :3] >= 250).all(axis=2).mean())
                            selected_adjusted = (forced_exposure or adjusted_white < original_white) and float(adjusted_rgb[:, :, :3].std()) > 2
                            if selected_adjusted:
                                rgb = adjusted_rgb
                                visibility = measure_visibility(segmentation_annotator.get_data(), rgb, prepared['entities'], camera, args.width, args.height)
                            exposure_record.update(retry_performed=True, requested_ev=requested_ev,forced_by_camera_override=forced_exposure,
                                attempted=adjusted, applied=adjusted if selected_adjusted else original_settings,
                                lower_exposure_selected=selected_adjusted, retry_near_white_fraction=adjusted_white,
                                original_preview_path=relative(original_image), original_preview_sha256=sha(original_image))
                        exposure_record['settings_sha256'] = hashlib.sha256(json.dumps(exposure_record['applied'], sort_keys=True).encode()).hexdigest()
                        Image.fromarray(rgb[:, :, :3].astype(np.uint8)).save(image_path)
                        noncamera_equal = noncamera_text(source_text) == noncamera_text(scene_path.read_text())
                        if not noncamera_equal:
                            raise ValueError('Camera candidate changed non-camera USD fields')
                        view = dict(view_id=view_id, **camera, rgb_valid=True, visibility=visibility,
                            required_entities_same_frame_readable=all(visibility[name]['readable_pixels'] for name in camera.get('required_entities',[])) if camera.get('required_entities') else None,
                            preview_path=relative(image_path), preview_sha256=sha(image_path), scene_path=relative(scene_path),
                            scene_sha256=sha(scene_path), noncamera_fields_unchanged=True,
                            renderer_settings=exposure_record, visual_quality_verified=False,
                            rgb_std=float(rgb[:, :, :3].std()), near_white_fraction=float((rgb[:, :, :3]>=250).all(axis=2).mean()))
                        views.append(view)
                        kind_views.append(view)
                        atomic_json(destination / (view_id + '.json'), view)
                        required = ['robot', 'hazard'] if kind == 'overview' else [name for name in prepared['entities'] if name.startswith(('object:', 'target:'))] + (['human:hand_region'] if layout['obstacle']['category'] == 'human' else ['hazard'])
                        if not camera.get('inspection_override') and all(visibility[name]['readable_pixels'] for name in required):
                            break
                    chosen = max(kind_views, key=lambda v: (v.get('required_entities_same_frame_readable') is True,
                        sum(min(2., m['pixels']/m['minimum_pixels']) for m in v['visibility'].values())))
                    selected.append(chosen)
                readability = multi_view_summary(views, prepared['entities'])
                # Preserve an additional candidate if it supplies the only
                # readable view of an entity omitted by the three default picks.
                for name in prepared['entities']:
                    if not any(v['visibility'][name]['readable_pixels'] for v in selected):
                        supplement = next((v for v in views if v['visibility'][name]['readable_pixels']), None)
                        if supplement is not None and supplement not in selected:
                            selected.append(supplement)
                unchanged = (sha(ROOT / row['config_path']) == prepared['config_sha256']
                    and all(sha(ROOT / p) == digest for p, digest in prepared['composition']['source_paths_sha256'].items()))
                if not unchanged:
                    raise ValueError('Scene config/source changed while rendering')
                report = dict(schema_version=1, scope=SCOPE, scene_id=row['scene_id'], task_id=row['task_id'], variant_id=row['variant_id'],
                    config_path=row['config_path'], config_sha256=prepared['config_sha256'], created_at=stamp(),
                    views=views, selected_views=selected, readability=readability, source_assets_unchanged=True,
                    physics_steps_requested=0, native_physics_verified=False, accepted=False,
                    human_hand_visual_review_required=layout['obstacle']['category']=='human',
                    visual_quality_verified=False, renderer_code_sha256=sha(Path(__file__)),
                    camera_overrides_sha256=prepared.get('camera_overrides_sha256'),
                    camera_overrides_source_path=prepared.get('camera_overrides_source_path'),
                    camera_overrides_source_sha256=prepared.get('camera_overrides_source_sha256'),
                    exposure_policy=dict(retry_ev=args.exposure_retry_ev, near_white_threshold=args.near_white_threshold,
                        local_documentation='omni.rtx.settings.core/omni/rtx/settings/core/widgets/post_widgets.py:ToneMappingSettingsFrame,AutoExposureSettingsFrame'),
                    elapsed_seconds=time.monotonic()-started)
                atomic_json(evidence, report)
                update(args.progress, row, render_status='ready' if readability['readability_passed'] else 'needs_view_review',
                       config_sha256=prepared['config_sha256'], readability_passed=readability['readability_passed'],
                       evidence_path=relative(evidence), evidence_sha256=sha(evidence), preview_path=selected[0]['preview_path'], views=selected,
                       reason='多视角像素条件通过，仍需看图复核。' if readability['readability_passed'] else '部分实体仍不清楚：'+', '.join(readability['missing_entities']))
            except Exception as error:
                destination.mkdir(parents=True, exist_ok=True)
                failure = destination / ('failure_' + str(time.time_ns()) + '.json')
                atomic_json(failure, dict(scope=SCOPE, error=repr(error), traceback=traceback.format_exc(), views=views, accepted=False))
                update(args.progress, row, render_status='failed', evidence_path=relative(failure), reason=repr(error))
                print('VISUAL_FAILED', row['scene_id'], repr(error), flush=True)
            finally:
                if original_settings:
                    for key, value in original_settings.items():
                        if value is not None:
                            settings.set(key, value)
                print('VISUAL_DETACH', row['scene_id'], flush=True)
                if segmentation_annotator is not None:
                    segmentation_annotator.detach()
                if rgb_annotator is not None:
                    rgb_annotator.detach()
                for _ in range(8):
                    app.update()
                if product is not None:
                    product.destroy()
                for _ in range(8):
                    app.update()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT/'configs/extensions/replicator/visual_scene_variants/manifest.json')
    parser.add_argument('--camera-overrides',type=Path,help='JSON entries: scene_id, config_sha256, camera_overrides and optional inspection_prims')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--progress', type=Path, default=ROOT/'reports/replicator_safety_upgrade/visual_placement/render_progress.json')
    parser.add_argument('--task', default='')
    parser.add_argument('--variant', default='')
    parser.add_argument('--exclude-variant', action='append', default=[], help='Repeat to exclude unstable placement variants')
    parser.add_argument('--sample-trio', action='store_true')
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=848)
    parser.add_argument('--candidates-per-kind', type=int, default=2)
    parser.add_argument('--warmup-frames', type=int, default=24)
    parser.add_argument('--exposure-retry-ev', type=float, default=-2., help='Retry overexposed views with this camera exposure compensation; 0 disables')
    parser.add_argument('--near-white-threshold', type=float, default=.15)
    parser.add_argument('--segmentation', choices=['fast', 'stable'], default='fast')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    for name in ('manifest', 'output', 'progress'):
        setattr(args, name, getattr(args, name).resolve())
    if args.width < 960 or args.height < 640 or args.candidates_per_kind < 1 or args.warmup_frames < 16:
        parser.error('Inspection requires >=960x640, >=1 candidate per kind and >=16 warmup frames')
    if not -6 <= args.exposure_retry_ev <= 0 or not 0 < args.near_white_threshold <= 1:
        parser.error('Exposure retry must be between -6 and 0 EV, near-white threshold within (0, 1]')
    manifest = json.loads(args.manifest.read_text())
    rows = manifest.get('scenes', manifest.get('entries', []))
    rows = [dict(row, config_path=row.get('config_path', row.get('layout_path'))) for row in rows]
    trio = {('g_shape_mustard_bowl', 'property_tall_glass'), ('u_shape_banana_bowl', 'property_tablet_stand'), ('l_shape_banana_bowl', 'property_vase')}
    rows = [r for r in rows if (not args.task or args.task in r['task_id']) and (not args.variant or args.variant == r['variant_id'])
            and r['variant_id'] not in args.exclude_variant
            and (not args.sample_trio or (r['task_id'].removeprefix('kitchen_bench_replicator_'), r['variant_id']) in trio)]
    if args.limit:
        rows = rows[:args.limit]
    if not rows:
        parser.error('No matching scenes')
    for row in rows:
        row.setdefault('scene_id', row['task_id'].removeprefix('kitchen_bench_replicator_')+'__'+row['variant_id'])
    if args.camera_overrides:
        overrides_path=args.camera_overrides.resolve()
        overrides=json.loads(overrides_path.read_text())
        overrides={r['scene_id']:r for r in overrides['entries']}
        for row in rows:
            if row['scene_id'] in overrides:
                value=overrides[row['scene_id']]
                if value['config_sha256']!=sha(ROOT/row['config_path']):
                    raise ValueError('Camera overrides bound to a different scene config')
                row.update(camera_overrides=value['camera_overrides'],inspection_prims=value.get('inspection_prims',[]),
                    camera_overrides_source_path=relative(overrides_path),camera_overrides_source_sha256=sha(overrides_path))
    if args.prepare_only:
        for row in rows:
            destination, layout, prepared = prepare_row(row, args.output, args.width, args.height, args.candidates_per_kind)
            update(args.progress, row, render_status='prepared', preparation_path=relative(destination/'preparation.json'),
                   config_sha256=prepared['config_sha256'])
            print('VISUAL_PREPARED', row['scene_id'], flush=True)
        return 0
    lock = ROOT/'reports/skill_trials/gpu.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        render_rows(args, rows)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
