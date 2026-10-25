"""Recollect only revised human episodes; preserve all previous evidence."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--sides', nargs='+', choices=['left', 'right'], default=['left', 'right'])
    parser.add_argument('--attempt', default='01')
    parser.add_argument('--right-config', type=Path)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=False)
    progress = {'accepted': False, 'status': 'running', 'steps': []}

    def save():
        progress['updated_utc'] = datetime.now(timezone.utc).isoformat()
        (args.output_root/'progress.json').write_text(json.dumps(progress, indent=2)+'\n')

    def run(name, script, arguments, gpu=False):
        command = (['bash', 'scripts/extensions/controls/run.sh', script] if gpu else
                   ['bash', 'scripts/arena_python.sh', 'scripts/extensions/controls/'+script])
        command += [str(x) for x in arguments]
        entry = {'name': name, 'command': command, 'status': 'running'}
        progress['steps'].append(entry)
        save()
        print('START', name, flush=True)
        with (args.output_root/(name+'.log')).open('x') as log:
            result = subprocess.run(command, cwd=ROOT, env=os.environ, stdout=log, stderr=subprocess.STDOUT)
        entry.update(status='passed' if result.returncode == 0 else 'failed', exit_code=result.returncode)
        save()
        if result.returncode:
            raise RuntimeError(name+' failed; inspect its log')
        print('PASS', name, flush=True)

    save()
    try:
        for side, old_name, stage in (
            ('left', 'ZXW_LEFT_BUTTON_REVIEWED_RESULTS.json', 'reports/skill_trials/controls/inspection/stage.usda'),
            ('right', 'ZXW_RIGHT_BUTTON_RESULTS.json', 'reports/controls_right_zxw_20261023/inspection/stage.usda'),
        ):
            if side not in args.sides:
                continue
            old = json.loads((ROOT/'docs/extensions/controls'/old_name).read_text())
            episodes = {row['variant']: Path(row['path']) for row in old['episodes']}
            baseline = episodes['base']
            config = Path('configs/extensions/controls')/(side+'_button_anatomical_candidate01.json')
            if side == 'right' and args.right_config:
                config = args.right_config
            paired = Path('reports/controls_hand_revision_20261024')/(side+'_paired'+args.attempt)
            if not paired.exists():
                run(side+'_paired', 'paired_planning.py', ['--config', config, '--baseline', baseline,
                    '--stage', stage, '--output', paired], gpu=True)
            from summarize_acceptance import paired_gates
            pairs = json.loads((paired/'paired_planning.json').read_text())
            if pairs['config_sha256'] != hashlib.sha256(config.read_bytes()).hexdigest():
                raise ValueError(side+' paired config fingerprint mismatch')
            if Path(pairs['baseline_state_source']).resolve() != baseline.resolve():
                raise ValueError(side+' paired baseline mismatch')
            gates = paired_gates(pairs)
            if not all(gates.values()):
                raise ValueError(side+' paired gates rejected: '+str(gates))
            ep = Path('datasets/controls_hand_revision_20261024')/side/('human'+args.attempt)
            run(side+'_collect', 'collect_trial.py', ['--controls-task', side, '--config', config,
                '--variant', 'human', '--output', ep, '--max-steps', '650', '--device', 'cpu'], gpu=True)
            run(side+'_data', 'validate_trial.py', [ep])
            run(side+'_contact', 'review_contact_causality.py', [ep])
            run(side+'_self', 'audit_self_collision.py', [ep], gpu=True)
            run(side+'_fixed', 'audit_fixed_base.py', [ep])
            run(side+'_strict', 'audit_augmentation.py', [ep, '--baseline', baseline/'records.json',
                '--stage', stage, '--report-name', 'strict_clearance_audit.json'])
            run(side+'_avoidance', 'audit_avoidance.py', [ep, '--baseline', baseline, '--stage', stage])
            run(side+'_comparison', 'render_avoidance_comparison.py', [ep, '--camera', 'external_camera'])
            run(side+'_first_violation', 'render_first_violation.py', [ep, '--output-stem', 'first_violation_links'])
            output = Path('docs/extensions/controls/hand_revision')/(side+'_results.json')
            run(side+'_summary', 'summarize_acceptance.py', ['--baseline', baseline, '--episodes', ep,
                episodes['property'], episodes['robot'], '--neutral', old['neutral'],
                '--faucet-mobility', old['faucet_mobility'], '--paired', paired, '--output', output])
        progress.update(status='complete', accepted=True)
        save()
    except Exception as exc:
        progress.update(status='failed', accepted=False, error=str(exc))
        save()
        raise


if __name__ == '__main__':
    main()
