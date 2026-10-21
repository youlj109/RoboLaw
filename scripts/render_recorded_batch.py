"""Render an explicit source/output manifest with one persistent Kit.

Compatible dynamic-obstacle episodes reuse the scene and its camera handles.
Changed static poses or topology/configuration rebuild the scene inside the
same Kit. Every episode retains the original warm-up and full state checks.
"""
from copy import deepcopy
import json
from pathlib import Path
import select
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def serve_requests(directory, execute, cleanup_failed, *, stdin=None):
    """Consume atomic files sequentially; stdin EOF drains then ends the Kit.

    No app updates occur while idle. The host owns stdin so its death also
    closes this worker after the current request, without a watchdog thread.
    """
    directory = Path(directory)
    stdin = sys.stdin if stdin is None else stdin
    handled = set()
    while True:
        # EOF takes precedence over unstarted requests when the host exits.
        readable, _, _ = select.select([stdin], [], [], .05)
        if readable and not stdin.buffer.read(1):
            return
        requests = sorted((directory / 'requests').glob('*.json'))
        for path in requests:
            if path.name in handled:
                continue
            handled.add(path.name)
            reply = {'id': path.stem, 'status': 'failed', 'completed': False}
            started = time.perf_counter()
            fatal = None
            try:
                item = json.loads(path.read_text())
                if item.get('id') != path.stem:
                    raise ValueError('Request id must match its filename')
                reply.update(execute(item))
                reply['status'] = 'completed' if reply.get('completed') is True else 'failed'
            except BaseException as error:
                reply.update(error=repr(error), traceback=traceback.format_exc())
                try:
                    cleanup_failed()
                except BaseException as cleanup_error:
                    reply['cleanup_error'] = repr(cleanup_error)
                    fatal = cleanup_error
                if not isinstance(error, Exception):
                    fatal = fatal or error
            reply['server_request_s'] = time.perf_counter() - started
            if fatal is not None:
                reply.update(worker_unavailable=True, status='worker_unavailable')
            atomic_json(directory / 'replies' / path.name, reply)
            if fatal is not None:
                raise fatal


def prepare_manifest(path):
    entries = json.loads(Path(path).read_text())
    if not isinstance(entries, list) or not entries:
        raise ValueError('Manifest must be a nonempty list of source_episode/output objects')
    result, outputs = [], set()
    for index, item in enumerate(entries):
        if not isinstance(item, dict) or not item.get('source_episode') or not item.get('output'):
            raise ValueError(f'Manifest entry {index} requires source_episode and output')
        source, output = Path(item['source_episode']).resolve(), Path(item['output']).resolve()
        if not source.is_dir():
            raise FileNotFoundError(source)
        if output.exists() or output in outputs:
            raise FileExistsError(f'Output must be new and unique: {output}')
        if output == source or output in source.parents:
            raise ValueError('Output cannot replace the source directory or an ancestor')
        outputs.add(output)
        result.append({'source_episode': str(source), 'output': str(output)})
    return result


def scene_signature(data):
    """Ordinary values needed to decide reuse versus rebuilding; no hashes."""
    spec, metadata = data['spec'], data['metadata']
    layout, obstacle = spec['layout'], spec['layout']['obstacle']
    configuration = metadata['source_configuration']
    return {'obstacle_id': spec['obstacle_id'],
            'obstacle_topology': {key: obstacle[key] for key in ('category', 'usd_path', 'static')},
            'fixed_layout': {key: layout.get(key) for key in ('robot_position', 'robot_yaw_deg', 'stand_height', 'counter_height')},
            'camera_config': data['camera_config'], 'kitchen': metadata['kitchen'],
            'robot': metadata['robot'], 'dt': data['dt'],
            'physics_dt': configuration['physics_dt'],
            'solver_position_iterations': configuration['solver_position_iterations']}


def scene_rebuild_reason(previous, current):
    if previous is None:
        return 'first_episode'
    if scene_signature(previous) != scene_signature(current):
        return 'topology_or_fixed_configuration_changed'
    old, new = previous['spec']['layout']['obstacle'], current['spec']['layout']['obstacle']
    if new['static'] and (old['position'], old.get('yaw_deg', 0.)) != (new['position'], new.get('yaw_deg', 0.)):
        return 'static_obstacle_pose_changed'
    return None


class RenderSceneCache:
    """Own one initialized scene and an isolated, replaceable appearance layer."""

    def __init__(self):
        self.env = self.kitchen = self.appearance_layer = None
        self.group_owner = None
        self.last_grouped_report = None
        self.previous = None
        self.scene_builds = self.scene_reuses = self.appearance_applications = 0
        self.rebuild_history = []

    def prepare_source(self, data):
        reason = scene_rebuild_reason(self.previous, data)
        reused = self.env is not None and reason is None
        if self.env is not None and reason is not None:
            started = time.perf_counter()
            self.close_scene()
            self.rebuild_history.append({'reason': reason, 'teardown_s': time.perf_counter() - started})
        if reused:
            self.scene_reuses += 1
        # Keep just the ordinary configuration required for the next decision;
        # retaining all frame arrays would grow memory with the manifest length.
        self.previous = {key: deepcopy(data[key]) for key in ('spec', 'metadata', 'camera_config', 'dt')}
        return {'scene_reused': reused, 'rebuild_reason': reason,
                'static_pose_update_strategy': 'Rebuild before physics initialization when static pose changes',
                'appearance_reset_strategy': 'Clear only the dedicated appearance layer; reapply original spec',
                'shared_kit': True}

    def get_or_create(self, builder, cfg, kwargs):
        if self.env is None:
            self.env = builder.make_registered(cfg, kwargs)
            self.env.reset()
            self.scene_builds += 1
        return self.env

    def apply_appearance(self, stage, spec):
        from pxr import Sdf
        from kitchen_safety.randomization import apply_appearance
        if self.appearance_layer is None:
            self.appearance_layer = Sdf.Layer.CreateAnonymous('recorded_episode_appearance.usda')
            stage.GetSessionLayer().subLayerPaths.insert(0, self.appearance_layer.identifier)
        else:
            # Clear and apply are deliberately separate composition operations:
            # apply_appearance must read the original material/light baselines.
            self.appearance_layer.Clear()
        report = apply_appearance(stage, spec, edit_layer=self.appearance_layer)
        self.appearance_applications += 1
        return report

    def static_pose_check(self, env, data):
        if data['dynamic_hazard']:
            return None
        from kitchen_safety.dataset import numpy
        from kitchen_safety.vectorized_scene import geometry_parameters
        from scripts.render_recorded_episode import pose_errors, translated_state
        name = data['metadata']['hazard']['name']
        local = geometry_parameters([data['spec']])['poses_local_xyz_xyzw'][name][0]
        origin = numpy(env.scene.env_origins)[0]
        expected = translated_state(local, 'environment_local', data['source_origin'], origin)
        actual = numpy(env.arena_world.get_pose_w(name))[0]
        return {'name': name, 'source': 'Recorded static layout; live Arena FrameView pose readback',
                'expected_pose_world_xyz_xyzw': expected.tolist(), 'actual_pose_world_xyz_xyzw': actual.tolist(),
                **pose_errors(actual, expected)}

    def close_scene(self):
        from isaaclab_arena.utils.isaaclab_utils.simulation_app import (
            collect_garbage_and_clear_cuda_cache, teardown_simulation_app,
        )
        env = self.env
        self.env = self.kitchen = self.appearance_layer = None
        try:
            try:
                if self.group_owner is not None:
                    self.group_owner.close()
                    self.last_grouped_report = self.group_owner.report()
            finally:
                self.group_owner = None
                if env is not None:
                    env.close()
        finally:
            env = None
            teardown_simulation_app(suppress_exceptions=False, make_new_stage=True)
            collect_garbage_and_clear_cuda_cache()


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    from scripts.render_recorded_episode import render_episode

    started = time.perf_counter()
    parser = get_isaaclab_arena_cli_parser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--manifest', type=Path)
    mode.add_argument('--serve', action='store_true')
    parser.add_argument('--request-dir', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--collection-purpose', choices=['diagnostic', 'production'], default='diagnostic')
    parser.add_argument('--collection-source', help='Originating collection run; final acceptance remains with verify_attempt')
    parser.add_argument('--camera-device', default=None)
    parser.add_argument('--background-rgb', action='store_true')
    parser.add_argument('--grouped-camera', action='store_true',
                        help='Opt-in grouped readout, retaining the original three camera mounts and warm-up')
    parser.add_argument('--refresh-render-products', action='store_true',
                        help='Recreate render products after each reused-scene appearance switch; retain original warm-up')
    parser.add_argument('--indices', type=int, nargs='+', help='Optional partial diagnostic indices for every source')
    args = parser.parse_args()
    if args.num_envs != 1:
        parser.error('Persistent offline rendering uses one environment')
    if args.serve and args.request_dir is None:
        parser.error('--serve requires --request-dir')
    entries = [] if args.serve else prepare_manifest(args.manifest)
    if args.serve:
        args.request_dir = args.request_dir.resolve()
        for name in ('requests', 'replies'):
            (args.request_dir / name).mkdir(parents=True, exist_ok=True)
    args.report = args.report.resolve()
    if args.report.exists():
        raise FileExistsError(args.report)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.enable_cameras = True
    args.kit_args = ((args.kit_args or '') + f' --/log/file={args.report.with_suffix(".kit.log")}').strip()
    report = {'completed': False, 'episodes': [], 'requested_episodes': len(entries),
              'production_quota_eligible': False, 'shared_kit': True, 'manifest': entries,
              'streaming_requests': bool(args.serve),
              'collection_purpose': args.collection_purpose, 'collection_source': args.collection_source,
              'acceptance_status': 'pending_batch_verification' if args.collection_purpose == 'production' else 'diagnostic',
              'grouped_camera_enabled': args.grouped_camera,
              'refresh_render_products_requested': args.refresh_render_products,
              'timing_boundary': 'Invocation through final writer drain and shared scene teardown; excludes SimulationApp shutdown',
              'validation_scope': 'Original per-episode warmup and full measured-state checks retained; run validate_collection --render-output separately for full JPEG/state validation'}
    cache = RenderSceneCache()

    def save():
        temporary = args.report.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2))
        temporary.replace(args.report)

    save()
    with SimulationAppContext(args) as app:
        report['app_startup_and_input_s'] = time.perf_counter() - started

        def execute(item):
            local = deepcopy(args)
            local.source_episode, local.output = Path(item['source_episode']), Path(item['output'])
            # Apply the same ordinary path/uniqueness rules as fixed manifests.
            source, output = local.source_episode.resolve(), local.output.resolve()
            if not source.is_dir():
                raise FileNotFoundError(source)
            if output.exists():
                raise FileExistsError(output)
            if source == output or output in source.parents:
                raise ValueError('Output cannot replace the source directory or an ancestor')
            tick = time.perf_counter()
            row = {'index': len(report['episodes']), **item, 'completed': False}
            report['episodes'].append(row)
            report['requested_episodes'] = len(report['episodes']) if args.serve else len(entries)
            save()
            try:
                result = render_episode(local, app_context=app, scene_cache=cache)
                row.update(completed=result['completed'], selected_frames_completed=result['selected_frames_completed'],
                           frames=result['rendered_frames'], scene_reuse=result['scene_reuse'],
                           appearance_warmup_passed=result.get('appearance_warmup', {}).get('passed'),
                           physics_steps=result.get('physics_steps'), render_report=str(local.output / 'render_report.json'))
                row.update(collection_purpose=result.get('collection_purpose', 'diagnostic'),
                           collection_source=result.get('collection_source'),
                           source_collection_job=result.get('source_collection_job'),
                           acceptance_status=result.get('acceptance_status'), production_quota_eligible=False)
                return dict(row)
            except BaseException as error:
                row.update(error=repr(error), traceback=traceback.format_exc())
                raise
            finally:
                row['elapsed_s'] = time.perf_counter() - tick
                save()

        try:
            if args.serve:
                atomic_json(args.request_dir / 'ready.json', {'ready': True,
                            'app_startup_and_input_s': report['app_startup_and_input_s']})
                serve_requests(args.request_dir, execute, cache.close_scene)
            else:
                for item in entries:
                    execute(item)
            report['completed'] = all(row['completed'] for row in report['episodes'])
        except BaseException as error:
            report.update(completed=False, error=repr(error), traceback=traceback.format_exc())
            raise
        finally:
            tick = time.perf_counter()
            try:
                cache.close_scene()
                report['scene_closed'] = True
            except BaseException as error:
                report.update(completed=False, cleanup_error=repr(error))
                raise
            finally:
                report.update(scene_close_s=time.perf_counter() - tick,
                              scene_builds=cache.scene_builds, scene_reuses=cache.scene_reuses,
                              appearance_applications=cache.appearance_applications,
                              rebuild_history=cache.rebuild_history,
                              total_before_app_exit_s=time.perf_counter() - started)
                if cache.last_grouped_report is not None:
                    report['grouped_camera_final'] = cache.last_grouped_report
                save()  # Kit may os._exit from __exit__.


if __name__ == '__main__':
    main()
