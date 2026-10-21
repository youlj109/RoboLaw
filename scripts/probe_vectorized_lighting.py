"""Static multi-env light/noise diagnostic; never contributes dataset quota.

Two baseline groups measure temporal variation. In linked mode only
env_0 lights change for the third group. Failed appearance warm-ups are retained
and do not prevent diagnostic PNGs; their success flags are never overridden.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import traceback


RENDERER_SETTING_PATHS = (
    '/rtx/rendermode', '/rtx/post/aa/op', '/rtx/post/dlss/execMode',
    '/rtx/rtpt/cached/enabled', '/rtx/rtpt/lightcache/cached/enabled',
    '/rtx/indirectDiffuse/enabled', '/rtx/reflections/enabled',
    '/rtx/directLighting/sampledLighting/samplesPerPixel',
    '/rtx-transient/dldenoiser/enabled', '/rtx/post/histogram/enabled',
    '/rtx/post/tonemap/op', '/rtx/scenedb/skipMostLights',
    '/rtx/domeLight/upperLowerStrategy', '/rtx/background/source/type',
    '/rtx-transient/sceneDb/lightLink/enabled',
    '/rtx-transient/sceneDb/shadowLink/enabled')


def renderer_settings_readback(settings):
    return {key: settings.get(key) for key in RENDERER_SETTING_PATHS}


def apply_aa_override(settings, aa_op):
    """Change only AA after camera initialization, recording actual SDK values.

    Camera renderer initialization can overwrite launch-time Kit settings. This
    optional diagnostic override is applied once, after the initial hold steps;
    later readbacks expose any further override instead of silently reapplying it.
    """
    if aa_op is not None and aa_op not in range(5):
        raise ValueError('AA op must be one of 0, 1, 2, 3, 4')
    before = renderer_settings_readback(settings)
    if aa_op is not None:
        settings.set('/rtx/post/aa/op', aa_op)
    after = renderer_settings_readback(settings)
    return {'requested': aa_op, 'set_attempted': aa_op is not None,
            'before': before, 'after': after,
            'requested_value_observed_after_set': (
                after['/rtx/post/aa/op'] == aa_op if aa_op is not None else None)}


def make_linked_lights_standalone(stage, appearance):
    """Diagnostic: flatten local light values, then deactivate global templates.

    Snapshot every light before editing any: later environment roots can inherit
    env_0. Explicitly empty reference lists block weaker inherited references as
    well as the local internal references. This does not change light settings or
    prove RTX isolation; the subsequent images provide the actual control.
    """
    from pxr import Sdf, Usd, UsdLux

    if appearance.get('lighting_mode') != 'linked_experimental':
        raise ValueError('Standalone lights require linked_experimental appearance')

    def json_value(value):
        if value is None or isinstance(value, (str, bool, int, float)):
            return value
        if isinstance(value, Sdf.AssetPath):
            return {'asset_path': value.path, 'resolved_path': value.resolvedPath}
        if isinstance(value, Sdf.Path):
            return str(value)
        if isinstance(value, dict):
            return {str(key): json_value(item) for key, item in value.items()}
        try:
            return [json_value(item) for item in value]
        except TypeError:
            return str(value)

    def snapshot(path):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.HasAPI(UsdLux.LightAPI):
            raise ValueError(f'Expected composed light at {path}')
        attrs = []
        for attr in prim.GetAttributes():
            value = attr.Get()
            times = [(time, attr.Get(time)) for time in attr.GetTimeSamples()]
            connections = attr.GetConnections()
            if value is None and not times and not connections:
                continue
            attrs.append({'name': attr.GetName(), 'type': attr.GetTypeName(),
                          'custom': attr.IsCustom(), 'variability': attr.GetVariability(),
                          'value': value, 'time_samples': times, 'connections': connections,
                          'metadata': {key: item for key, item in attr.GetAllMetadata().items()
                                       if key not in ('typeName', 'custom', 'variability', 'default',
                                                      'timeSamples', 'connectionPaths')}})
        relationships = [{'name': rel.GetName(), 'custom': rel.IsCustom(),
                          'targets': rel.GetTargets(),
                          'metadata': {key: item for key, item in rel.GetAllMetadata().items()
                                       if key not in ('custom', 'variability', 'targetPaths')}}
                         for rel in prim.GetRelationships()]
        references = prim.GetMetadata('references')
        return {'path': str(path), 'type': prim.GetTypeName(), 'active': prim.IsActive(),
                'applied_schemas': prim.GetAppliedSchemas(), 'attributes': attrs,
                'relationships': relationships,
                'has_authored_references': prim.HasAuthoredReferences(),
                'composed_reference_items': [str(item) for item in references.GetAppliedItems()] if references else [],
                'reference_metadata': str(references)}

    def report_snapshot(row):
        return {**{key: value for key, value in row.items() if key not in ('attributes', 'relationships')},
                'attributes': [{**attr, 'type': str(attr['type']),
                                'variability': str(attr['variability']),
                                'value': json_value(attr['value']),
                                'time_samples': [[time, json_value(value)] for time, value in attr['time_samples']],
                                'connections': [str(path) for path in attr['connections']],
                                'metadata': {key: json_value(value) for key, value in attr['metadata'].items()}}
                               for attr in row['attributes']],
                'relationships': [{**rel, 'targets': [str(path) for path in rel['targets']],
                                   'metadata': {key: json_value(value) for key, value in rel['metadata'].items()}}
                                  for rel in row['relationships']]}

    local_paths = [row['path'] for row in appearance['lights'] if row['env_index'] is not None]
    if not local_paths or len(set(local_paths)) != len(local_paths):
        raise ValueError('Expected unique local linked-light paths')
    before = [snapshot(path) for path in local_paths]
    global_paths = list(appearance['environments'][0]['requested_lights'])
    globals_before = [{'path': path, 'type': stage.GetPrimAtPath(path).GetTypeName(),
                       'active': stage.GetPrimAtPath(path).IsActive()} for path in global_paths]
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for row in before:
            stage.RemovePrim(row['path'])
            prim = stage.DefinePrim(row['path'], row['type'])
            prim.GetReferences().SetReferences([])
            prim.SetActive(row['active'])
            prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(row['applied_schemas']))
            for item in row['attributes']:
                attr = prim.CreateAttribute(item['name'], item['type'], custom=item['custom'],
                                            variability=item['variability'])
                if item['value'] is not None:
                    attr.Set(item['value'])
                for time, value in item['time_samples']:
                    attr.Set(value, time)
                attr.SetConnections(item['connections'])
                for key, value in item['metadata'].items():
                    attr.SetMetadata(key, value)
            for item in row['relationships']:
                rel = prim.CreateRelationship(item['name'], custom=item['custom'])
                rel.SetTargets(item['targets'])
                for key, value in item['metadata'].items():
                    rel.SetMetadata(key, value)
        for path in global_paths:
            stage.GetPrimAtPath(path).SetActive(False)
    return {'applied': True, 'diagnostic_only': True, 'production_quota_eligible': False,
            'reference_note': 'An explicitly empty reference list blocks inherited references; '
                              'HasAuthoredReferences may be true while composed_reference_items is empty.',
            'edit_layer': 'session', 'local_lights_before': [report_snapshot(row) for row in before],
            'local_lights_after': [report_snapshot(snapshot(path)) for path in local_paths],
            'global_lights_before': globals_before,
            'global_lights_after': [{'path': path, 'type': stage.GetPrimAtPath(path).GetTypeName(),
                                    'active': stage.GetPrimAtPath(path).IsActive()} for path in global_paths]}


def linked_light_parameters(stage, appearance):
    """Read actual local-light values for reversible, separate interventions."""
    from pxr import UsdLux

    rows = []
    for source in appearance['lights']:
        if source['env_index'] is None:
            continue
        prim = stage.GetPrimAtPath(source['path'])
        light = UsdLux.LightAPI(prim)
        rows.append({'path': source['path'], 'env_index': source['env_index'],
                     'type': prim.GetTypeName(), 'intensity': float(light.GetIntensityAttr().Get()),
                     'temperature_k': float(light.GetColorTemperatureAttr().Get()),
                     'color_temperature_enabled': bool(light.GetEnableColorTemperatureAttr().Get())})
    return rows


def set_light_type_intervention(stage, baselines, light_type=None):
    """Restore all baseline parameters, then change only env_0's chosen type.

    Unchanged attributes are not rewritten: that would needlessly invalidate the
    renderer's history for the control environment. None restores the baseline.
    """
    from pxr import Usd, UsdLux

    if light_type not in (None, 'DomeLight', 'DistantLight'):
        raise ValueError('Intervention type must be DomeLight, DistantLight or None')
    changes = []
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for baseline in baselines:
            desired = {key: baseline[key] for key in ('intensity', 'temperature_k', 'color_temperature_enabled')}
            if baseline['env_index'] == 0 and baseline['type'] == light_type:
                desired.update(intensity=baseline['intensity'] * 2., color_temperature_enabled=True,
                               temperature_k=2000. if baseline['temperature_k'] >= 5000. else 10000.)
            light = UsdLux.LightAPI(stage.GetPrimAtPath(baseline['path']))
            attrs = {'intensity': light.GetIntensityAttr(), 'temperature_k': light.GetColorTemperatureAttr(),
                     'color_temperature_enabled': light.GetEnableColorTemperatureAttr()}
            before = {key: attr.Get() for key, attr in attrs.items()}
            changed = [key for key in attrs if before[key] != desired[key]]
            if not changed:
                continue
            for key in changed:
                attrs[key].Set(desired[key])
            changes.append({'path': baseline['path'], 'env_index': baseline['env_index'],
                            'type': baseline['type'], 'before': before,
                            'after': {key: attr.Get() for key, attr in attrs.items()},
                            'changed_attributes': changed})
    return {'target_env': 0, 'target_type': light_type, 'applied': bool(changes), 'lights': changes,
            'operation': 'restore original baseline' if light_type is None else
                         'restore original baseline, then double intensity and change temperature for one light type'}


def light_collection_membership(stage, environment_roots):
    """Evaluate USD collections on actual geometry paths, independent of RTX.

    One representative Gprim per asset subtree keeps this report small while
    covering kitchen, robot, manipulated objects and obstacle geometry.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdLux

    roots = []
    for path in environment_roots:
        prim = stage.GetPrimAtPath(path)
        representatives = {}
        for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if child.IsA(UsdGeom.Gprim):
                relative = str(child.GetPath().MakeRelativePath(prim.GetPath()))
                representatives.setdefault(relative.split('/')[0], str(child.GetPath()))
        roots.append({'path': str(path), 'direct_inherits': [str(value) for value in prim.GetInherits().GetAllDirectInherits()],
                      'geometry_paths': list(representatives.values())})
    lights = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdLux.LightAPI):
            continue
        row = {'path': str(prim.GetPath()), 'type': prim.GetTypeName(), 'collections': {}}
        for name in ('lightLink', 'shadowLink'):
            collection = Usd.CollectionAPI(prim, name)
            query = collection.ComputeMembershipQuery()
            row['collections'][name] = {
                root['path']: {'root_included': bool(query.IsPathIncluded(Sdf.Path(root['path']))),
                               'geometry': {path: bool(query.IsPathIncluded(Sdf.Path(path)))
                                            for path in root['geometry_paths']}}
                for root in roots}
        lights.append(row)
    return {'meaning': 'Composed USD collection membership, not proof of RTX illumination membership',
            'environment_roots': roots, 'lights': lights}


def record_static_rgb_trace(writer, sim, path, frames):
    """Save individual 160x90 BOX samples, with no temporal filtering or gate.

    The trace is a diagnostic after warm-up, including when warm-up failed.
    Every frame has a fresh render generation and retains its own image. It is
    not an action trajectory and cannot certify randomized collection quality.
    """
    import h5py
    import numpy as np
    from kitchen_safety.vectorized_dataset import _ensure_render_update

    if frames < 1:
        raise ValueError('Trace needs at least one render frame')
    indices = list(writer.writers)
    before = int(sim.get_physics_step_count())
    started = time.perf_counter()
    report = {'path': str(path), 'frames': 0, 'environment_ids': indices,
              'sample_size': [160, 90], 'spatial_filter': 'Per-frame uint8 BOX',
              'temporal_filter': None, 'convergence_checked': False,
              'physics_step_count_before': before, 'error': None}
    try:
        with h5py.File(path, 'x') as handle:
            handle.attrs['diagnostic_only'] = True
            handle.attrs['production_quota_eligible'] = False
            handle.attrs['temporal_filter'] = 'none'
            handle.attrs['environment_ids'] = indices
            handle.attrs['physics_step_count'] = before
            generations = handle.create_dataset('render_generation', (frames,), dtype='i8')
            datasets = {name: handle.create_dataset(name, (frames, len(indices), 90, 160, 3),
                        dtype='u1', chunks=(1, 1, 90, 160, 3), compression='lzf') for name in writer.names}
            try:
                for frame in range(frames):
                    generation = int(sim.render_generation)
                    sim.render()
                    if int(sim.render_generation) <= generation:
                        raise RuntimeError('Trace render did not advance render generation')
                    _ensure_render_update()
                    if int(sim.get_physics_step_count()) != before:
                        raise RuntimeError('Physics advanced during static appearance trace')
                    snapshot = writer._warmup_snapshot(small_readback=True)
                    for name in writer.names:
                        values = np.stack([snapshot[index, name] for index in indices])
                        if values.shape != (len(indices), 90, 160, 3):
                            raise ValueError('Unexpected static trace image shape')
                        datasets[name][frame] = values.astype(np.uint8)
                    generations[frame] = int(sim.render_generation)
                    report['frames'] = frame + 1
            finally:
                handle.attrs['frames_written'] = report['frames']
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    report['physics_step_count_after'] = int(sim.get_physics_step_count())
    report['physics_advanced'] = report['physics_step_count_after'] != before
    report['elapsed_seconds'] = time.perf_counter() - started
    return report


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    parser = get_isaaclab_arena_cli_parser()
    parser.set_defaults(num_envs=2)
    parser.add_argument('--num-envs', dest='num_envs', type=int, default=argparse.SUPPRESS)
    parser.add_argument('--episode-spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--lighting', choices=['shared', 'linked_experimental'], default='linked_experimental')
    parser.add_argument('--appearance', choices=['randomized', 'authored'], default='randomized',
                        help='Controlled diagnostic: apply requested random appearance or preserve original USD')
    parser.add_argument('--samples-per-pixel', type=int, choices=[1, 2, 4, 8])
    parser.add_argument('--aa-op', type=int, choices=range(5),
                        help='Optional post-initialization AA override: 0 off, 1 TAA, 2 FXAA, 3 DLSS, 4 DLAA')
    parser.add_argument('--standalone-linked-lights', action='store_true',
                        help='Diagnostic: flatten local linked lights and deactivate original global light templates')
    parser.add_argument('--split-light-interventions', action='store_true',
                        help='Five groups: two baselines, env_0 Dome change, restored baseline, env_0 Distant change')
    parser.add_argument('--warmup-mode', choices=['single_frame', 'temporal_mean'], default='single_frame',
                        help='Temporal means are diagnostic; unaveraged comparisons must also pass original limits')
    parser.add_argument('--warmup-window-frames', type=int, default=16,
                        help='Non-overlapping temporal_mean window; single_frame retains its original 16-frame cadence')
    parser.add_argument('--warmup-max-frames', type=int, default=1024,
                        help='Pure-render appearance warm-up budget; no physics advancement')
    parser.add_argument('--baseline-only', action='store_true',
                        help='Run one warm-up group and save PNGs plus four consecutive low-resolution frames')
    parser.add_argument('--trace-frames', type=int, default=0,
                        help='After each group, save this many unaveraged consecutive 160x90 frames per camera/env')
    args = parser.parse_args()
    if args.warmup_mode != 'temporal_mean' and args.warmup_window_frames != 16:
        parser.error('warmup-window-frames is configurable only with --warmup-mode temporal_mean')
    if args.warmup_window_frames < 1 or args.warmup_max_frames < max(128, 4 * args.warmup_window_frames):
        parser.error('Warm-up requires a positive window and at least max(128, 4 * window) render frames')
    if args.num_envs < 1 or (args.num_envs < 2 and not args.baseline_only):
        parser.error('Lighting intervention requires two environments; baseline-only also supports one')
    if args.trace_frames < 0:
        parser.error('trace-frames must be nonnegative')
    if args.appearance == 'authored' and not args.baseline_only:
        parser.error('authored appearance is a baseline-only control')
    if args.standalone_linked_lights and (args.appearance != 'randomized' or args.lighting != 'linked_experimental'):
        parser.error('standalone-linked-lights requires randomized appearance and linked_experimental lighting')
    if args.split_light_interventions and (args.baseline_only or args.appearance != 'randomized'
                                           or args.lighting != 'linked_experimental'):
        parser.error('split-light-interventions requires linked_experimental randomized appearance without baseline-only')
    spec = json.loads(args.episode_spec.read_text())
    specs = [copy.deepcopy(spec) for _ in range(args.num_envs)]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    args.enable_cameras = True
    args.kit_args = ((args.kit_args or '') + f' --/log/file={out / "kit.log"}').strip()
    report = {'status': 'initializing', 'diagnostic_only': True, 'production_quota_eligible': False,
              'num_envs': args.num_envs, 'lighting': args.lighting, 'episode_spec': str(args.episode_spec),
              'appearance_mode': args.appearance, 'trace_frames_requested': args.trace_frames,
              'standalone_linked_lights_requested': args.standalone_linked_lights,
              'split_light_interventions_requested': args.split_light_interventions,
              'warmup_mode': args.warmup_mode, 'baseline_only': args.baseline_only,
              'groups': {}, 'comparisons': {}, 'independent_lighting_render_verified': False,
              'interpretation': 'Compare env_0 intervention against baseline temporal differences and unchanged envs. '
                                'Warm-up failure or image differences are diagnostic evidence, not accepted collection data.'}
    (out / 'episode_specs.json').write_text(json.dumps(specs, indent=2))
    started = time.perf_counter()
    with SimulationAppContext(args):
        import carb.settings
        import numpy as np
        import torch
        from PIL import Image
        from pxr import Tf, Usd, UsdLux, UsdShade
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from kitchen_safety.dataset import CAMERA_NAMES, numpy
        from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
        from kitchen_safety.vectorized_dataset import VectorizedCameraWriter
        from kitchen_safety.vectorized_scene import configure_vectorized_geometry, apply_vectorized_appearance

        torch.set_num_threads(1)
        env = writer = notice_listener = None
        renderer_settings = carb.settings.get_settings()

        def save_report():
            report['elapsed_seconds'] = time.perf_counter() - started
            (out / 'probe.json').write_text(json.dumps(report, indent=2))

        def stats(rgb):
            array = rgb.astype(np.float32)
            luma = array @ np.array([.2126, .7152, .0722], dtype=np.float32)
            return {'mean_rgb': array.mean(axis=(0, 1)).tolist(), 'mean_luma': float(luma.mean()),
                    'std_luma': float(luma.std()), 'min': float(array.min()), 'max': float(array.max())}

        def difference(first, second):
            delta = second.astype(np.float32) - first.astype(np.float32)
            absolute = np.abs(delta)
            luma_delta = delta @ np.array([.2126, .7152, .0722], dtype=np.float32)
            return {'mean_absolute_rgb_difference': float(absolute.mean()),
                    'percentile95_absolute_rgb_difference': float(np.percentile(absolute, 95)),
                    'signed_mean_luma_change': float(luma_delta.mean()),
                    'mean_absolute_luma_difference': float(np.abs(luma_delta).mean())}

        def light_readback(stage):
            rows = []
            for prim in stage.Traverse():
                if not prim.HasAPI(UsdLux.LightAPI):
                    continue
                light = UsdLux.LightAPI(prim)
                row = {'path': str(prim.GetPath()), 'type': prim.GetTypeName(),
                       'intensity': light.GetIntensityAttr().Get(),
                       'color_temperature_enabled': light.GetEnableColorTemperatureAttr().Get(),
                       'temperature_k': light.GetColorTemperatureAttr().Get(), 'links': {}}
                for name in ('lightLink', 'shadowLink'):
                    collection = Usd.CollectionAPI(prim, name)
                    row['links'][name] = {
                        'include_root': collection.GetIncludeRootAttr().Get(),
                        'expansion_rule': str(collection.GetExpansionRuleAttr().Get()),
                        'includes': [str(path) for path in collection.GetIncludesRel().GetTargets()],
                        'excludes': [str(path) for path in collection.GetExcludesRel().GetTargets()]}
                rows.append(row)
            return rows

        def visual_input_readback(stage):
            values = {}
            attributes = ('inputs:diffuse_tint', 'inputs:diffuseColor',
                          'inputs:reflection_roughness_constant', 'inputs:roughness',
                          'inputs:intensity', 'inputs:colorTemperature', 'inputs:enableColorTemperature')
            for prim in stage.Traverse():
                if not (prim.IsA(UsdShade.Shader) or prim.HasAPI(UsdLux.LightAPI)):
                    continue
                for name in attributes:
                    attr = prim.GetAttribute(name)
                    if attr:
                        value = attr.Get()
                        if value is not None and not isinstance(value, (str, bool, int, float)):
                            value = list(value)
                        values[str(attr.GetPath())] = value
            return values

        notice_activity = {'notices': 0, 'changed_path_count': 0, 'sample_paths': []}

        def on_stage_change(notice, stage):
            paths = [str(path) for path in (*notice.GetChangedInfoOnlyPaths(), *notice.GetResyncedPaths())
                     if str(path).startswith(('/World/envs/', '/World/Light', '/World/DirectionalLight'))]
            if not paths:
                return
            notice_activity['notices'] += 1
            notice_activity['changed_path_count'] += len(paths)
            notice_activity['sample_paths'] = sorted(set(notice_activity['sample_paths']).union(paths))[:256]

        try:
            environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
                variant=spec['layout']['obstacle']['category'], obstacle_id=spec['obstacle_id'],
                episode_spec=spec, enable_cameras=True, collection_seed=args.seed or 0))
            builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
                device=args.device, seed=args.seed or 0, presets='physx', num_envs=args.num_envs,
                env_spacing=args.env_spacing, solve_relations=False, disable_fabric=False,
                recorder_dataset_export_dir_path=str(out / 'arena_recorder')))
            cfg, kwargs = builder.compose_manager_cfg()
            cfg.sim.log_dir = str(out / 'isaaclab_logs')
            cfg.sim.render_interval = cfg.decimation
            cfg.scene.replicate_physics = True
            cfg.observations.camera_obs = None
            report['geometry'] = configure_vectorized_geometry(cfg, specs)
            if args.samples_per_pixel is not None:
                from isaaclab_physx.renderers.isaac_rtx_renderer_cfg import IsaacRtxRendererCfg
                for name in CAMERA_NAMES:
                    camera = getattr(cfg.scene, name)
                    if getattr(camera.renderer_cfg, 'renderer_type', None) == 'default':
                        camera.renderer_cfg = IsaacRtxRendererCfg()
                    camera.renderer_cfg.global_settings.samples_per_pixel = args.samples_per_pixel
            env = builder.make_registered(cfg, kwargs)
            env.reset()
            e, scene = env.unwrapped, env.unwrapped.scene
            robot = scene['robot']
            ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)
            joint_pos = robot.data.joint_pos
            joint_pos = joint_pos.torch if hasattr(joint_pos, 'torch') else joint_pos
            hold = torch.cat((joint_pos[:, ids], torch.zeros((args.num_envs, 1), device=e.device)), dim=1)
            for _ in range(8):
                env.step(hold)
            report['aa_override'] = apply_aa_override(renderer_settings, args.aa_op)
            report['physics_step_count_before_groups'] = int(e.sim.get_physics_step_count())
            report['appearance'] = (apply_vectorized_appearance(e.sim.stage, specs, lighting=args.lighting)
                if args.appearance == 'randomized' else
                {'requested_spec_appearance_applied': False, 'source': 'Original authored USD baseline',
                 'production_quota_eligible': False, 'lights': [], 'environments': []})
            if args.standalone_linked_lights:
                report['standalone_linked_lights'] = make_linked_lights_standalone(e.sim.stage, report['appearance'])
            report['lights_before'] = light_readback(e.sim.stage)
            light_baselines = linked_light_parameters(e.sim.stage, report['appearance'])
            if args.split_light_interventions:
                report['collection_membership_before'] = light_collection_membership(
                    e.sim.stage, [row['root'] for row in report['appearance']['environments']])
            report['visual_inputs_initial'] = visual_input_readback(e.sim.stage)
            report['renderer_settings'] = renderer_settings_readback(renderer_settings)
            report['samples_per_pixel'] = e.sim.get_setting('/rtx/directLighting/sampledLighting/samplesPerPixel')
            origins = numpy(scene.env_origins)
            writer = VectorizedCameraWriter(
                {i: out / 'writer_state' / f'env_{i:03d}' for i in range(args.num_envs)}, scene,
                dt=e.step_dt, env_origins=origins)
            report['writer_state_note'] = 'Zero-action HDF5 files only retain warm-up metadata; images are the diagnostic PNGs below.'
            notice_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_stage_change, e.sim.stage)
            groups = {}
            labels = (('baseline_a',) if args.baseline_only else
                      ('baseline_a', 'baseline_b', 'changed_dome', 'restored_baseline', 'changed_distant')
                      if args.split_light_interventions else ('baseline_a', 'baseline_b', 'changed'))
            report['diagnostic_group_schedule'] = list(labels)
            for label in labels:
                if args.split_light_interventions and label not in ('baseline_a', 'baseline_b'):
                    target_type = {'changed_dome': 'DomeLight', 'restored_baseline': None,
                                   'changed_distant': 'DistantLight'}[label]
                    report.setdefault('interventions', {})[label] = set_light_type_intervention(
                        e.sim.stage, light_baselines, target_type)
                if label == 'changed':
                    changes = []
                    if args.lighting == 'linked_experimental':
                        with Usd.EditContext(e.sim.stage, e.sim.stage.GetSessionLayer()):
                            for row in report['appearance']['lights']:
                                if row['env_index'] != 0:
                                    continue
                                light = UsdLux.LightAPI(e.sim.stage.GetPrimAtPath(row['path']))
                                old_intensity = float(light.GetIntensityAttr().Get())
                                old_temperature = float(light.GetColorTemperatureAttr().Get())
                                new_temperature = 2000. if old_temperature >= 5000. else 10000.
                                light.CreateIntensityAttr(old_intensity * 2)
                                light.CreateEnableColorTemperatureAttr(True)
                                light.CreateColorTemperatureAttr(new_temperature)
                                changes.append({'path': row['path'], 'old_intensity': old_intensity,
                                                'new_intensity': float(light.GetIntensityAttr().Get()),
                                                'old_temperature_k': old_temperature,
                                                'new_temperature_k': float(light.GetColorTemperatureAttr().Get())})
                    report['intervention'] = {'applied': bool(changes), 'lights': changes,
                        'note': 'Only env_0 linked lights change.' if changes else
                                'Shared mode: no light intervention; third group repeats the noise control.'}
                    report['lights_after'] = light_readback(e.sim.stage)
                inputs_before = visual_input_readback(e.sim.stage)
                activity_before = copy.deepcopy(notice_activity)
                settings_before = renderer_settings_readback(renderer_settings)
                warmup = writer.warm_up(e.sim, mode=args.warmup_mode,
                                       sample_every=args.warmup_window_frames, max_frames=args.warmup_max_frames)
                folder = out / label
                folder.mkdir()
                row = {'appearance_warmup': warmup, 'images': {}, 'low_resolution_sequences': {},
                       'light_readback': light_readback(e.sim.stage),
                       'renderer_settings_before': settings_before,
                       'physics_step_count_before': int(e.sim.get_physics_step_count())}
                report['groups'][label] = row
                captures, low = {}, {}
                for frame in range(4):
                    if frame:
                        e.sim.render()
                    captured = writer._capture()
                    for name in CAMERA_NAMES:
                        for index in range(args.num_envs):
                            key = f'env_{index:03d}_{name}'
                            rgb = captured[name]['rgb'][index]
                            small = np.asarray(Image.fromarray(rgb).resize((160, 90), Image.Resampling.BOX))
                            low.setdefault(key, []).append(small)
                            row['low_resolution_sequences'].setdefault(key, {'statistics': [], 'consecutive_differences': []})
                            sequence = row['low_resolution_sequences'][key]
                            sequence['statistics'].append(stats(small))
                            if frame:
                                sequence['consecutive_differences'].append(difference(low[key][-2], small))
                            if frame == 0:
                                path = folder / f'{key}.png'
                                Image.fromarray(rgb).save(path)
                                captures[key] = rgb.copy()
                                row['images'][key] = {'path': str(path), 'statistics': stats(rgb),
                                    'position_world': captured[name]['position'][index].tolist(),
                                    'environment_origin_world_m': origins[index].tolist(),
                                    'quaternion_world_ros_xyzw': captured[name]['quaternion'][index].tolist(),
                                    'intrinsics': captured[name]['intrinsics'][index].tolist(),
                                    'resolution': [1280, 720]}
                np.savez_compressed(folder / 'consecutive_low_resolution.npz', **{key: np.stack(value) for key, value in low.items()})
                if args.trace_frames:
                    row['static_rgb_trace'] = record_static_rgb_trace(
                        writer, e.sim, folder / 'static_rgb_trace.h5', args.trace_frames)
                inputs_after = visual_input_readback(e.sim.stage)
                row['visual_input_changes_during_group'] = {
                    key: {'before': inputs_before.get(key), 'after': inputs_after.get(key)}
                    for key in sorted(set(inputs_before) | set(inputs_after))
                    if inputs_before.get(key) != inputs_after.get(key)}
                row['stage_notice_activity_before'] = activity_before
                row['stage_notice_activity_after'] = copy.deepcopy(notice_activity)
                row['renderer_settings_after'] = renderer_settings_readback(renderer_settings)
                row['physics_step_count_after'] = int(e.sim.get_physics_step_count())
                row['physics_advanced'] = row['physics_step_count_after'] != row['physics_step_count_before']
                groups[label] = captures
                save_report()
                print('LIGHT_PROBE_GROUP', label, 'warmup_passed', warmup['passed'], flush=True)
            pairs = ((('baseline_a', 'baseline_b'), ('baseline_b', 'changed_dome'),
                      ('baseline_b', 'restored_baseline'), ('restored_baseline', 'changed_distant'))
                     if args.split_light_interventions else
                     (('baseline_a', 'baseline_b'), ('baseline_b', 'changed')))
            for first, second in pairs:
                if first not in groups or second not in groups:
                    continue
                report['comparisons'][f'{first}_to_{second}'] = {
                    key: difference(groups[first][key], groups[second][key]) for key in groups[first]}
            if args.split_light_interventions:
                report['collection_membership_after'] = light_collection_membership(
                    e.sim.stage, [row['root'] for row in report['appearance']['environments']])
            report['physics_step_count_after_groups'] = int(e.sim.get_physics_step_count())
            report['physics_advanced_during_groups'] = (
                report['physics_step_count_after_groups'] != report['physics_step_count_before_groups'])
            report['status'] = 'completed_diagnostic'
        except Exception:
            report.update(status='error', error=traceback.format_exc())
            raise
        finally:
            report['renderer_settings_end'] = renderer_settings_readback(renderer_settings)
            save_report()
            if notice_listener:
                notice_listener.Revoke()
            if writer:
                writer.close()
            if env:
                env.close()
            print('LIGHT_PROBE_RESULT', str(out / 'probe.json'), flush=True)


if __name__ == '__main__':
    main()
