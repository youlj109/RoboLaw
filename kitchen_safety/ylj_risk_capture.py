"""ylj frozen scene/commands; current raw recorder and offline risk semantics."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from kitchen_safety.task_contract import digest

ROOT = Path(__file__).resolve().parents[1]
VERSION = 'robolaw-ylj-risk-integration-1'


def select_entry(index, scene_id):
    rows = [e for e in index['entries'] if e['scene_id'] == scene_id]
    if len(rows) != 1:
        raise ValueError('Select one exact ylj scene ID; no implicit variant selection')
    row = rows[0]
    return dict(scene_id=row['scene_id'], task_id=row['task_id'], variant_id=row['variant_id'],
        config_path=row['config_path'], config_sha256=row['config_sha256'],
        result_path=row['source_result_path'], result_sha256=row['source_result_sha256'],
        trajectory_path=row['artifacts']['trajectory']['path'],
        trajectory_sha256=row['artifacts']['trajectory']['sha256'],
        source_scene_path=row['source_scene']['path'], source_scene_sha256=row['source_scene']['sha256'])


def input_preflight(contract, index, scene_id, lock):
    entry = select_entry(index, scene_id)
    if entry['task_id'] != 'kitchen_bench_'+contract['task_id']:
        raise ValueError('ylj scene belongs to another native task')
    missing = []; changed = []
    for name in ('config', 'result', 'trajectory', 'source_scene'):
        path = ROOT/entry[name+'_path']
        if not path.is_file(): missing.append(entry[name+'_path'])
        elif hashlib.sha256(path.read_bytes()).hexdigest() != entry[name+'_sha256']:
            changed.append(entry[name+'_path'])
    new_human = entry['variant_id'] == 'human_default_mpfb' and not lock['human_collision_paths']
    source_gate_error=None
    if not missing and not changed:
        from kitchen_safety.extensions.replicator.planned_execution import load_inputs
        try:load_inputs(entry)  # Also validate original NPZ arrays and direct external asset SHA.
        except Exception as exc:source_gate_error=repr(exc)
    return dict(version=VERSION, entry=entry, missing_inputs=missing, changed_inputs=changed,
        original_source_gate_error=source_gate_error,
        unreviewed_human_would_expand_scope=new_human,
        ready=not missing and not changed and not new_human and source_gate_error is None,
        approved_PT_assets=len(lock['pt_targets']), online_risk_queries=0,
        simulation_started=False, replanned=False)


def commands(data, start_q, *, initial_settle=5., close=2., release=2., final_settle=3.):
    """Retain ylj's exact command samples, 30 Hz and inserted physical holds."""
    from kitchen_safety.extensions.replicator.planned_execution import validate_planned_arrays
    validate_planned_arrays(data); dt = float(data['dt'])
    if any(not math.isfinite(x) or x < 0 for x in (initial_settle, close, release, final_settle)):
        raise ValueError('Invalid ylj hold duration')
    def hold(q, grip, phase, seconds):
        for _ in range(int(math.ceil(seconds/dt))):
            yield dict(q=np.asarray(q), grip=grip, source_phase=phase, source_index=-1)
    yield from hold(start_q, 0., 'initial_settle', initial_settle)
    previous = None
    for i, (q, phase, grip) in enumerate(zip(data['q'], data['phase'], data['gripper'])):
        phase = str(phase)
        if phase != previous:
            if phase == 'lift': yield from hold(data['q'][i-1], float(grip), 'close', close)
            if phase == 'retreat': yield from hold(data['q'][i-1], 0., 'release', release)
        yield dict(q=q, grip=float(grip), source_phase=phase, source_index=i)
        previous = phase
    yield from hold(data['q'][-1], 0., 'final_settle', final_settle)


def capture(args, contract, entry, lock):
    """Called only inside SimulationAppContext; never builds risk GPU structures."""
    import torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.replicator.planned_execution import (
        load_inputs, build_environment, configure_physics_threads, reset_state_gate)
    from kitchen_safety.extensions.replicator.task_collection_evidence import TaskPredicateRecorder
    from kitchen_safety.task_risk_setup import prepare
    from kitchen_safety.task_capture import TaskRawRecorder
    from kitchen_safety.risk_observer import write_json
    from kitchen_safety.task_contract import digest
    from kitchen_safety.articulation_capture import array
    output = args.output; output.mkdir(parents=True, exist_ok=False)
    plan, layout, data = load_inputs(entry)  # Original SHA gates, no fallback/replan.
    if abs(float(data['dt'])-1/30) > 1e-9: raise ValueError('Frozen command clock differs')
    threads = configure_physics_threads(args.physics_threads)
    torch.set_num_threads(4)
    definition = build_environment(plan, layout, output/'physical_overlays', args.episode_seconds,
                                   background_name=contract['environment_id'])
    original_callback=definition.env_cfg_callback
    def causal_clock(cfg):
        cfg=original_callback(cfg)
        cfg.sim.dt=1/120;cfg.decimation=4;cfg.sim.render_interval=4
        cfg.observations.camera_obs=None
        return cfg
    definition.env_cfg_callback=causal_clock
    assets = definition.scene.assets
    metadata = dict(task_id=contract['task_id'], task_sha256=digest(contract),
        asset_path_templates={aid: assets[contract['environment_id'] if row['kind']=='background' else aid].get_prim_path()
                              for aid, row in contract['asset_declarations'].items()},
        source='ylj frozen layout without relation solving',
        background_physics=deepcopy(definition.background_physics_provenance))
    metadata['asset_path_templates']['hazard'] = assets['hazard'].get_prim_path()
    builder = ArenaEnvBuilder(definition, ArenaEnvBuilderCfg(num_envs=1, seed=args.seed,
        device=args.device, presets='physx', solve_relations=False, resolve_on_reset=False, disable_fabric=True))
    env = builder.make_registered(); e = env.unwrapped
    e.configure_episode_limit(1); env.reset(seed=args.seed)
    original_reset = e._reset_idx; terminal_events = []
    def keep_episode(ids):
        if len(ids): terminal_events.append(dict(physics_step=int(e.sim.get_physics_step_count()),
            reason='ylj sole-initialization policy; native terminal state retained'))
    e._reset_idx = keep_episode
    recorder = None; error = None; sequence_complete = False
    receipt = dict(version=VERSION, task_id=contract['task_id'], scene_id=entry['scene_id'],
        ylj_source_commit='e8c8fedfba335e3a1c7bcc979f380eb63138689f',task_sha256=digest(contract),
        entry=entry, reviewed_lock_sha256=digest(lock), background_physics=metadata['background_physics'],
        threads=threads, risk_evaluation_status='pending_offline', IR='disabled',
        online_risk_queries=0, online_risk_features=0, online_grades=0, online_RGB_frames=0,
        replanned=False, trajectory_retimed=False,
        scene_namespace_change_only={'kitchen':contract['environment_id']})
    try:
        # Humans require an already reviewed HS scope. Never assert absence for an added person.
        obstacle = dict(asset_id='hazard', category='HS' if entry['variant_id']=='human_default_mpfb' else 'PT')
        setup = prepare(env, contract, metadata, output, obstacle=obstacle, reviewed_lock=lock)
        if not setup['inventory']['complete'] or not setup['measured']['complete'] or setup['scope']['unresolved']:
            raise ValueError('Incomplete ylj scene evidence or unresolved task bindings')
        robot = e.scene['robot']; arm = [robot.joint_names.index('panda_joint'+str(i)) for i in range(1,8)]
        q_error = float(np.max(np.abs(array(robot.data.joint_pos)[0,arm]-plan['start_q'])))
        differences = []
        for name, spec in layout['objects'].items():
            state = setup['measured']['actor_states_world'][metadata['asset_path_templates'][name].replace('{ENV_REGEX_NS}','/World/envs/env_0')]
            angle = math.radians(spec.get('yaw_deg',0.))/2; quat = np.asarray(state[3:7])
            expected = np.array([0.,0.,math.sin(angle),math.cos(angle)])
            differences.append(dict(name=name, position_error_m=float(np.linalg.norm(np.asarray(state[:3])-spec['position'])),
                orientation_error_deg=float(np.degrees(2*np.arccos(np.clip(abs(np.dot(quat/np.linalg.norm(quat),expected)),0,1))))))
        gate = reset_state_gate(q_error, differences); receipt['reset_gate'] = gate
        if not gate['passed']: raise ValueError('ylj frozen reset identity gate failed')
        predicates = TaskPredicateRecorder(e, definition)
        recorder = TaskRawRecorder(env, output/'raw_episode', contract, setup['inventory'], setup['scope'],
            setup['rs'], setup['config'], output/'meshes', allow_terminal_continuation=True)
        predicates.sample_physics('initial',-1)
        original_update=e.scene.update
        def observe_task(*args,**kwargs):
            value=original_update(*args,**kwargs)
            if recorder.active:predicates.sample_physics(recorder.phase,recorder.action_index)
            return value
        recorder._set(e.scene,'update',observe_task)
        write_json(recorder.directory/'capture_context.json', metadata)
        # Freeze actual controller inputs separately from risk implementation sources.
        frozen = recorder.directory/'controller_inputs'; frozen.mkdir()
        import shutil
        for role in ('config','result','trajectory','source_scene'):
            source=ROOT/entry[role+'_path']; shutil.copy2(source, frozen/(role+source.suffix))
        write_json(recorder.directory/'policy_binding.json', dict(factory=VERSION, entry=entry,
            risk_phase_aliases={'close':'grasp','initial_settle':'settle','final_settle':'settle'},
            physical_holds_s=dict(initial_settle=args.initial_settle_seconds,close=args.close_seconds,
                release=args.release_seconds,final_settle=args.final_settle_seconds)))
        operation = contract['operations'][0]['operation_id']
        for command in commands(data, plan['start_q'], initial_settle=args.initial_settle_seconds,
                close=args.close_seconds, release=args.release_seconds, final_settle=args.final_settle_seconds):
            if recorder.action_index >= args.max_actions: raise RuntimeError('Explicit action limit reached')
            source_phase=command['source_phase'];phase={'close':'grasp','initial_settle':'settle','final_settle':'settle'}.get(source_phase,source_phase)
            action=torch.tensor(np.r_[command['q'],command['grip']][None],dtype=torch.float32,device=e.device)
            recorder.step(action,phase=phase,operation_id=operation,
                controller_evidence=dict(source='ylj frozen NPZ',source_phase=source_phase,
                                         source_trajectory_index=command['source_index']))
            predicates.sample_action(source_phase,recorder.action_index-1)
            if predicates.actions[-1]['terminal_failure']: break
        else: sequence_complete=True
        receipt.update(sequence_complete=sequence_complete,
            original_task_success=any(x['terminal_success'] for x in predicates.actions),
            final_original_destination_predicate=predicates.actions[-1]['original_object_on_destination'] if predicates.actions else None,
            scope_ceiling=setup['scope']['reviewed_scope_ceiling'])
        receipt['task_success']=bool(receipt['original_task_success'] and receipt['final_original_destination_predicate'])
        predicates.finish(output)
    except BaseException as exc:
        error=repr(exc); receipt['error']=error; raise
    finally:
        if recorder:
            recorder.close(task_success=(recorder.last_action_outcome or {}).get('success'), execution_error=error,
                           stop_reason='ylj_sequence_complete' if sequence_complete else 'ylj_execution_stopped')
            write_json(recorder.directory/'ylj_integration.json',receipt)
        receipt['terminal_events']=terminal_events
        write_json(output/'ylj_capture_receipt.json',receipt)
        e._reset_idx=original_reset;env.close()
