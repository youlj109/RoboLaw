"""One fixed-layout left-button episode per variant; no bulk collection."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'configs/extensions/controls/repaired_left_button.json'
VARIANTS = ('base', 'human', 'property', 'robot')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--device', default='cpu')
parser.add_argument('--seed', type=int, default=42)
parser.add_argument('--dry-run', action='store_true')
parser.add_argument('--resume', action='store_true')
args = parser.parse_args()
output = args.output.resolve()
try:
    container_output = output.relative_to(ROOT)
except ValueError:
    parser.error('--output must be inside this repository, which is mounted at /workspace')
if not args.dry_run:
    output.mkdir(parents=True, exist_ok=args.resume)
source_files = [CONFIG, ROOT/'scripts/extensions/controls/collect_trial.py']
source_files.extend((ROOT/'kitchen_safety/extensions/controls').glob('*.py'))
source_files.extend(ROOT/spec['asset'] for spec in json.loads(CONFIG.read_text())['variants'].values())
asset_manifest = ROOT/'assets/extensions/controls/human_reach_elbow_up/artifact_manifest.json'
source_files.append(asset_manifest)
source_files.extend(ROOT/path for path in json.loads(asset_manifest.read_text())['generated_files'])
fingerprint = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
               for path in sorted(set(source_files))}
previous = json.loads((output/'collection_manifest.json').read_text()) if args.resume and (output/'collection_manifest.json').exists() else {}
if args.resume and previous.get('source_fingerprint') != fingerprint:
    raise ValueError('Resume requires the original matching source/asset fingerprint; use a new directory')
manifest = {'task': json.loads(CONFIG.read_text())['task_id'],
            'scope': 'Exactly one collector-accepted episode per variant; independent safety/video '
                     'acceptance is separate. No randomization or batch success-rate claim.',
            'container': os.environ.get('ARENA_CONTAINER', 'arena-kitchen-dev'),
            'config_sha256': hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
            'source_fingerprint': fingerprint, 'runs': []}

for variant in VARIANTS:
    episode = output / variant
    command = ['bash', 'scripts/extensions/controls/run.sh', 'collect_trial.py',
               '--config', str(CONFIG.relative_to(ROOT)), '--variant', variant,
               '--output', str(container_output / variant), '--seed', str(args.seed), '--max-steps', '650',
               '--device', args.device]
    print('COLLECTION_COMMAND', json.dumps(command), flush=True)
    if args.dry_run:
        continue
    if episode.exists():
        existing = json.loads((episode/'result.json').read_text()) if (episode/'result.json').exists() else {}
        if args.resume and existing.get('accepted') is True and existing.get('seed') == args.seed and existing.get('config_sha256') == manifest['config_sha256']:
            manifest['runs'].append({'variant': variant, 'episode': str(episode),
                                     'collector_accepted': True, 'physical_success': True, 'resumed': True})
            manifest['raw_collection_complete'] = len(manifest['runs']) == 4
            (output/'collection_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            continue
        raise FileExistsError('Use a new output directory; incomplete/failed physical evidence is never overwritten: ' + str(episode))
    log = output / (variant + '.log')
    if log.exists():
        raise FileExistsError(log)
    with log.open('w') as stream:
        process = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    result_path = episode / 'result.json'
    result = json.loads(result_path.read_text()) if result_path.exists() else {}
    passed = process.returncode == 0 and result.get('accepted') is True
    manifest['runs'].append({'variant': variant, 'episode': str(episode),
                             'exit_code': process.returncode, 'collector_accepted': passed,
                             'physical_success': result.get('success'), 'failure': result.get('failure'),
                             'log': str(log)})
    manifest['raw_collection_complete'] = len(manifest['runs']) == 4 and all(
        row['collector_accepted'] for row in manifest['runs'])
    (output / 'collection_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('COLLECTION_RESULT', variant, passed, result.get('failure'), flush=True)
    if not passed:
        sys.exit(2)
