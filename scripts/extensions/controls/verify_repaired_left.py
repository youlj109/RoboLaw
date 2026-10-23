"""Run ordered independent acceptance for the repaired left-button task only."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--episodes-root', type=Path, required=True)
parser.add_argument('--reports-root', type=Path, required=True)
parser.add_argument('--neutral', type=Path, required=True)
parser.add_argument('--faucet-mobility', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--wait-seconds', type=float, default=0.)
args = parser.parse_args()


def relative(path):
    return str(path.resolve().relative_to(ROOT))


episodes = args.episodes_root.resolve()
reports = args.reports_root.resolve()
for path in (episodes, reports, args.neutral, args.faucet_mobility, args.output):
    relative(path)
reports.mkdir(parents=True, exist_ok=False)
deadline = time.monotonic() + args.wait_seconds
progress = {'task': 'repaired_left_cancel_button', 'complete': False, 'steps': []}


def save():
    temp = reports/'verification_progress.tmp'
    temp.write_text(json.dumps(progress, indent=2)+'\n')
    temp.replace(reports/'verification_progress.json')


def run(label, script, arguments, gpu=False):
    command = (['bash', 'scripts/extensions/controls/run.sh', script] if gpu else
               ['bash', 'scripts/arena_python.sh', 'scripts/extensions/controls/'+script])
    command.extend(map(str, arguments))
    progress['current'] = label
    row = {'label': label, 'command': command, 'status': 'running'}
    progress['steps'].append(row)
    save()
    print('VERIFY_START', label, flush=True)
    with (reports/(label+'.log')).open('x') as log:
        process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    row.update(status='passed' if process.returncode == 0 else 'failed', exit_code=process.returncode)
    save()
    print('VERIFY_RESULT', label, process.returncode, flush=True)
    if process.returncode:
        raise RuntimeError('Acceptance command failed: '+label)


try:
    for variant in ('base', 'human', 'property', 'robot'):
        episode = episodes/variant
        progress['current'] = 'wait_'+variant
        save()
        while True:
            try:
                result = json.loads((episode/'result.json').read_text())
                break
            except (FileNotFoundError, json.JSONDecodeError):
                if time.monotonic() >= deadline:
                    raise TimeoutError('Missing complete result: '+str(episode))
                time.sleep(1.)
        if result.get('accepted') is not True:
            raise RuntimeError('Collector rejected '+variant+': '+str(result.get('failure')))
        for name in ('independent_validation.json', 'contact_causality_review.json',
                     'self_collision_detailed_audit.json', 'fixed_base_hazard_audit.json',
                     'strict_clearance_audit.json', 'avoidance_audit.json',
                     'comparison_validation.json', 'comparison.mp4'):
            if (episode/name).exists():
                raise FileExistsError('Refusing to overwrite historical acceptance evidence: '+str(episode/name))
        ep = relative(episode)
        base = relative(episodes/'base')
        run(variant+'_data', 'validate_trial.py', [ep])
        run(variant+'_contact', 'review_contact_causality.py', [ep])
        run(variant+'_self_collision', 'audit_self_collision.py', [ep], gpu=True)
        if variant != 'base':
            run(variant+'_fixed_base', 'audit_fixed_base.py', [ep])
            run(variant+'_strict_clearance', 'audit_augmentation.py',
                [ep, '--baseline', base+'/records.json', '--report-name', 'strict_clearance_audit.json'])
            run(variant+'_avoidance', 'audit_avoidance.py', [ep, '--baseline', base])
            run(variant+'_comparison', 'render_avoidance_comparison.py', [ep, '--camera', 'external_camera'])
    paired = relative(reports/'paired')
    run('paired_planning', 'paired_planning.py', [
        '--config', 'configs/extensions/controls/repaired_left_button.json', '--baseline', base,
        '--stage', 'reports/skill_trials/controls/inspection/stage.usda', '--output', paired,
        '--seeds', '42', '43', '44', '--variants', 'base', 'human', 'property', 'robot'], gpu=True)
    run('summary', 'summarize_acceptance.py', [
        '--baseline', base, '--episodes', *(relative(episodes/v) for v in ('human','property','robot')),
        '--neutral', relative(args.neutral), '--faucet-mobility', relative(args.faucet_mobility),
        '--paired', paired, '--output', relative(args.output)])
    progress.update(complete=True, current='complete')
    save()
except Exception as exc:
    progress['error'] = str(exc)
    save()
    print('VERIFY_FAILED', exc, flush=True)
    sys.exit(2)
