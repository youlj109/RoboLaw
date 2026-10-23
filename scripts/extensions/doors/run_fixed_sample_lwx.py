"""Bounded C01-C08 fixed-layout collection; one accepted sample, no batch quota."""
import argparse
import ast
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

GATES = [('result.json', 'accepted'), ('task_validation_full.json', 'accepted'),
         ('task_review_audit.json', 'passed'), ('source_contact_audit.json', 'passed'),
         ('self_collision_audit.json', 'passed')]


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def gate_values(episode):
    return {name: bool(read(episode / name).get(key)) if (episode / name).exists() else False
            for name, key in GATES}


def paired_accepted(report):
    pairs = report.get('pairs', [])
    if [p['seed'] for p in pairs] != [42, 43, 44]:
        return False
    if not (report.get('stable_policy_necessity_in_tested_seeds') and
            report.get('fixed_endpoint_rerouting_supported')):
        return False
    for pair in pairs:
        off, on = pair['off'], pair['on']
        if not off.get('success') or not on.get('success'):
            return False
        if not any(v.get('intersecting') for v in off['risk_with_added_geometry']['violations']):
            return False
        before, after = off['path_metrics'], on['path_metrics']
        if (after['joint_l2_length_rad'] / max(before['joint_l2_length_rad'], 1e-9) > 1.6 or
                after['tcp_length_m'] - before['tcp_length_m'] > .15):
            return False
    return True


def repeated_failure(history):
    reasons = [item.get('failure_signature', item.get('failure')) for item in history[-3:]]
    return len(reasons) == 3 and bool(reasons[0]) and len(set(reasons)) == 1


def failure_signature(stage, failure, returncode=None):
    if failure and 'EXECUTION_COLLISION ' in failure:
        detail = json.loads(failure.split('EXECUTION_COLLISION ', 1)[1])
        pair = detail.get('failure', {})
        return json.dumps([stage, 'execution_collision', detail.get('phase'), pair.get('robot_part'), pair.get('obstacle')])
    if failure:
        return stage + ':' + failure
    return stage + ':exit=' + str(returncode)


def collection_arguments(task, variant, config, policy, review, output, mode):
    args = ['--task-id', task, '--variant', variant, '--variant-config', str(config),
            '--output', str(output), '--mode', mode, '--validated-pregrasp', '--render-warmup',
            '--max-steps', '1500', '--review-config', str(review)]
    fields = {'seed': 'seed', 'handle_offset_y': 'handle-offset-y', 'handle_offset_z': 'handle-offset-z',
              'grasp_roll_deg': 'grasp-roll-deg', 'grasp_pitch_deg': 'grasp-pitch-deg',
              'handle_normal_offset_m': 'handle-normal-offset', 'pull_lead_rad': 'pull-lead-rad',
              'pull_steps': 'pull-steps', 'ik_limit_margin_rad': 'ik-limit-margin-rad',
              'release_retreat_m': 'release-retreat-m', 'release_tilt_deg': 'release-tilt-deg',
              'release_settle_steps': 'release-settle-steps'}
    for key, flag in fields.items():
        if key in policy:
            args += ['--' + flag, str(policy[key])]
    if policy.get('handle_height_candidates_m'):
        raise ValueError('This runner requires one fixed source-handle target; adaptive legacy policies need their original runner')
    return args


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--list-tasks', action='store_true')
    p.add_argument('--label')
    p.add_argument('--variant', choices=['base', 'human', 'property', 'robot'])
    p.add_argument('--baseline', type=Path)
    p.add_argument('--policy-from', type=Path, help='Recorded metadata.json or its episode directory')
    p.add_argument('--variant-config', type=Path)
    p.add_argument('--review-config', type=Path)
    p.add_argument('--output-root', type=Path)
    p.add_argument('--report-root', type=Path)
    p.add_argument('--release-retreat-m', type=float, help='Baseline-only policy revision; variants inherit their matching baseline')
    p.add_argument('--release-tilt-deg', type=float, help='Baseline-only release direction revision; variants inherit their matching baseline')
    p.add_argument('--release-settle-steps', type=int, help='Baseline-only fixed actuator hold before release; matched variants inherit this policy')
    p.add_argument('--pull-steps', type=int, help='Baseline-only pull duration revision, inherited by matched variants')
    p.add_argument('--container', default=os.environ.get('ARENA_CONTAINER'))
    p.add_argument('--runtime-python', default='/workspace/.venv-arena/bin/python')
    p.add_argument('--gpu-lock', default='/workspace/reports/skill_trials/gpu.lock')
    p.add_argument('--success-quota', type=int, default=1, choices=[1])
    p.add_argument('--max-attempts', type=int, default=1, choices=range(1, 11))
    p.add_argument('--resume', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    model = ast.parse(Path('kitchen_safety/extensions/doors/task_model.py').read_text())
    tasks = next(ast.literal_eval(node.value) for node in model.body
                 if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'TASK_IDS' for t in node.targets))
    if a.list_tasks:
        print(json.dumps(tasks, indent=2))
        return 0
    if a.label not in tasks or not a.variant or not a.output_root or not (a.policy_from or a.baseline) or not a.container:
        p.error('Specify --label, --variant, --output-root, --container, and --policy-from or --baseline')
    if a.variant != 'base' and (not a.baseline or not all(gate_values(a.baseline).values())):
        p.error('A baseline passing all five independent gates is required for a variant')
    metadata_path = a.policy_from or a.baseline
    metadata_path = metadata_path / 'metadata.json' if metadata_path.is_dir() else metadata_path
    metadata = read(metadata_path)
    if metadata['task'] != tasks[a.label]:
        p.error('Policy metadata task differs from requested task')
    if a.variant != 'base':
        baseline_metadata = read(a.baseline / 'metadata.json')
        if any(metadata[k] != baseline_metadata[k] for k in
               ['controller_parameters', 'robot_base', 'robot_yaw_deg', 'source_yaml_sha256']):
            p.error('Variant policy or source layout differs from the matching baseline')
    config = a.variant_config or Path('configs/extensions/doors') / (a.label.lower() + '_variants_lwx_01.json')
    review = a.review_config or next((path for path in [
        Path('configs/extensions/doors') / (a.label.lower() + '_review_lwx.json'),
        Path('configs/extensions/doors/review_gates_lwx.json')]
        if path.exists() and sha(path) == metadata['review_config_sha256']), None)
    if review is None:
        p.error('Cannot recover exact predeclared review config; provide --review-config')
    if sha(review) != metadata['review_config_sha256']:
        p.error('Review config differs from the recorded predeclared policy')
    policy = dict(metadata['controller_parameters'])
    if a.release_settle_steps is not None:
        if a.variant != 'base' or not 0 <= a.release_settle_steps <= 90:
            p.error('Release settling revision requires a new baseline and 0..90 steps')
        if a.release_settle_steps:
            policy['release_settle_steps'] = a.release_settle_steps
        else:
            policy.pop('release_settle_steps', None)
    if a.pull_steps is not None:
        if a.variant != 'base' or not 1 <= a.pull_steps <= 1200:
            p.error('Pull duration revision requires a new baseline and 1..1200 steps')
        policy['pull_steps'] = a.pull_steps
    if a.release_retreat_m is not None:
        if a.variant != 'base' or not 0. <= a.release_retreat_m <= .04:
            p.error('Release revision is allowed only for a new baseline and must be in [0,.04] m')
        if a.release_retreat_m:
            policy['release_retreat_m'] = a.release_retreat_m
        else:
            policy.pop('release_retreat_m', None)
    if a.release_tilt_deg is not None:
        if a.variant != 'base' or not -60. <= a.release_tilt_deg <= 60. or not policy.get('release_retreat_m'):
            p.error('Release tilt revision needs a new baseline with retreat and must be in [-60,60] deg')
        if a.release_tilt_deg:
            policy['release_tilt_deg'] = a.release_tilt_deg
        else:
            policy.pop('release_tilt_deg', None)
    if policy.get('max_joint_speed') != .5 or 'pregrasp_validation' not in policy:
        p.error('This runner supports the recorded .5 rad/s validated-pregrasp policy')
    recorded_config = read(Path(metadata['variant_config']))
    selected_config = read(config)
    if any(recorded_config.get(k) != selected_config.get(k) for k in ['_kitchen_asset', '_fixed_source_layout']):
        p.error('Kitchen or fixed source layout differs from recorded policy')
    prefix = ['docker', 'exec', a.container]
    environment = ['env', '-u', 'ISAACSIM_ROOT_PATH', '-u', 'ISAACLAB_PATH', '-u', 'OMNI_SERVER',
                   'PYTHONPATH=/workspace', 'HEADLESS=1', 'OPENBLAS_NUM_THREADS=1', 'OMP_NUM_THREADS=1', a.runtime_python, '-u']

    def command(script, args, gpu=False):
        return prefix + (['flock', a.gpu_lock] if gpu else []) + environment + ['scripts/extensions/doors/' + script, *map(str, args)]

    first = a.output_root / (a.label + '_' + a.variant + '_attempt01')
    preview = command('collect_tasks_lwx.py', collection_arguments(tasks[a.label], a.variant, config, policy, review, first, 'collect'), True)
    identity = dict(label=a.label, task=tasks[a.label], variant=a.variant, policy=policy,
                    policy_metadata=str(metadata_path), policy_metadata_sha256=sha(metadata_path),
                    variant_config=str(config), config_sha256=sha(config), review_config=str(review),
                    review_config_sha256=sha(review), baseline=str(a.baseline) if a.baseline else None,
                    baseline_trajectory_sha256=sha(a.baseline / 'trajectory.json') if a.baseline else None,
                    report_root=str(a.report_root or a.output_root / 'reports'))
    if a.dry_run:
        print(json.dumps(dict(identity=identity, collection_command=preview, success_quota=1,
                             max_attempts=a.max_attempts, resume=a.resume,
                             gates=GATES, source_risk_illustration=True,
                             manual_visual_review_required=True, physics_started=False), indent=2))
        return 0
    manifest = a.output_root / (a.label + '_' + a.variant + '_fixed_run.json')
    if manifest.exists():
        if not a.resume:
            p.error('Existing run requires --resume; no evidence will be overwritten')
        state = read(manifest)
        if state['identity'] != identity:
            p.error('Resume identity/config/policy hashes changed; use a separate output root')
        for attempt in state['attempts']:
            if attempt.get('automated_gates_passed'):
                episode = Path(attempt['episode'])
                if not all(gate_values(episode).values()):
                    raise RuntimeError('Previously accepted gates changed; inspect before resuming')
                for path, digest in attempt['gate_sha256'].items():
                    if sha(path) != digest:
                        raise RuntimeError('Stored accepted evidence changed: ' + path)
                print('ONE_ACCEPTED_FIXED_SAMPLE_ALREADY_PRESENT; manual visual review remains separate', episode)
                return 0
            if 'failure' not in attempt:
                raise RuntimeError('Interrupted partial attempt needs inspection before a new physical trial: ' + attempt['episode'])
    else:
        state = dict(identity=identity, attempts=[], success_quota=1, complete=False)
        a.output_root.mkdir(parents=True, exist_ok=True)
        atomic_save(manifest, state)
    report_root = a.report_root or a.output_root / 'reports'
    report_root.mkdir(parents=True, exist_ok=True)

    def run(name, script, args, gpu=False):
        attempt['active_stage'] = name
        atomic_save(manifest, state)
        logfile = report_root / (episode.name + '_' + name + '.log')
        with logfile.open('x') as stream:
            subprocess.run(command(script, args, gpu), stdin=subprocess.DEVNULL, stdout=stream,
                           stderr=subprocess.STDOUT, check=True)

    while len(state['attempts']) < a.max_attempts:
        if repeated_failure(state['attempts']):
            raise RuntimeError('Three identical failures retained; diagnosis is required before further collection')
        number = len(state['attempts']) + 1
        episode = a.output_root / (a.label + '_' + a.variant + '_attempt' + str(number).zfill(2))
        neutral = episode.with_name(episode.name + '_neutral')
        if episode.exists() or neutral.exists():
            raise FileExistsError('Partial evidence requires inspection, not automatic overwrite: ' + str(episode))
        attempt = dict(episode=str(episode), neutral=str(neutral), automated_gates_passed=False)
        state['attempts'].append(attempt)
        atomic_save(manifest, state)
        try:
            if a.variant != 'base':
                paired = report_root / (episode.name + '_paired')
                run('paired', 'paired_tasks_lwx.py', ['--baseline', a.baseline, '--variant', a.variant,
                                                   '--config', config, '--output', paired], True)
                if not paired_accepted(read(paired / 'report.json')):
                    raise RuntimeError('Paired source obstruction or small reroute rejected; redesign before physical collection')
                attempt['paired_report'] = str(paired / 'report.json')
            run('collect', 'collect_tasks_lwx.py', collection_arguments(tasks[a.label], a.variant, config, policy, review, episode, 'collect'), True)
            duration = max(40., read(episode / 'result.json')['frames'] * metadata['dt'] + 2.)
            run('neutral', 'collect_tasks_lwx.py', collection_arguments(tasks[a.label], a.variant, config, policy, review, neutral, 'neutral') + ['--neutral-seconds', str(duration)], True)
            run('neutral_review', 'audit_task_review_lwx.py', [neutral])
            run('task_review', 'audit_task_review_lwx.py', [episode, '--neutral', neutral])
            run('full', 'validate_task_trial_lwx.py', [episode])
            run('source_contact', 'audit_task_contact_lwx.py', [episode])
            run('self', 'audit_self_collision.py', [episode], True)
            if not all(gate_values(episode).values()):
                raise RuntimeError('Independent gates rejected')
            run('frames', 'review_task_success_lwx.py', [episode, '--output-dir', report_root])
            if a.variant != 'base':
                run('avoidance', 'audit_task_avoidance_lwx.py', [episode, '--baseline', a.baseline])
                run('comparison', 'make_task_comparison_lwx.py', [episode, '--baseline', a.baseline, '--run-label', '03'])
                run('source_risk', 'make_source_risk_illustration_lwx.py', [episode, '--baseline', a.baseline, '--output', report_root / (episode.name + '_source_risk01.jpg')])
            evidence = [episode / name for name, _ in GATES] + [episode / 'metadata.json', neutral / 'result.json', neutral / 'task_review_audit.json']
            if a.variant != 'base':
                evidence += [Path(attempt['paired_report']), episode / 'avoidance_audit.json',
                             episode / 'baseline_comparison_external_camera_2_03.json']
            attempt.update(automated_gates_passed=True, manual_visual_review_pending=True,
                           gate_sha256={str(path): sha(path) for path in evidence})
            state['complete'] = True
            atomic_save(manifest, state)
            print('ONE_ACCEPTED_FIXED_SAMPLE; manual visual review remains required', episode)
            return 0
        except Exception as exc:
            result = read(episode / 'result.json') if (episode / 'result.json').exists() else {}
            attempt['failure'] = result.get('failure') or repr(exc)
            attempt['failure_signature'] = failure_signature(attempt.get('active_stage', 'prepare'),
                                                              result.get('failure'), getattr(exc, 'returncode', None))
            atomic_save(manifest, state)
            # A failed geometric pair must be redesigned, never retried as physical sampling.
            if not episode.exists():
                raise
    raise RuntimeError('Bounded attempt limit reached; retained failures require inspection')


if __name__ == '__main__':
    sys.exit(main())
