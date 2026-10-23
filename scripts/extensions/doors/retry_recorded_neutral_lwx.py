"""One replacement neutral from a successful operation's exact recorded policy."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from run_fixed_sample_lwx import collection_arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report-root', type=Path, required=True)
    args = parser.parse_args()
    state_path = args.report_root / (args.output.name + '_state01.json')
    log = args.report_root / (args.output.name + '_collect01.log')
    if args.output.exists() or state_path.exists() or log.exists():
        raise FileExistsError('Retain existing neutral evidence and use a new output')
    state = {'episode': str(args.episode), 'neutral': str(args.output), 'complete': False}

    def save():
        state_path.write_text(json.dumps(state, indent=2) + '\n')

    try:
        save()
        result = json.loads((args.episode / 'result.json').read_text())
        if not result.get('accepted'):
            raise RuntimeError('Original operation has not succeeded')
        metadata = json.loads((args.episode / 'metadata.json').read_text())
        config = Path(metadata['variant_config'])
        config_hash = hashlib.sha256(config.read_bytes()).hexdigest()
        if config_hash != metadata['implementation_sha256'].get(str(config)):
            raise RuntimeError('Recorded variant config changed')
        review = next(p for p in Path('configs/extensions/doors').glob('*review*.json')
                      if hashlib.sha256(p.read_bytes()).hexdigest() == metadata['review_config_sha256'])
        duration = max(40., result['frames'] * metadata['dt'] + 2.)
        arguments = collection_arguments(metadata['task'], metadata['variant'], config,
                                         metadata['controller_parameters'], review, args.output, 'neutral')
        command = ['docker', 'exec', 'robolaw-wenxuanli', 'flock',
                   '/workspace/reports/skill_trials/gpu.lock', 'env',
                   '-u', 'ISAACSIM_ROOT_PATH', '-u', 'ISAACLAB_PATH', '-u', 'OMNI_SERVER',
                   'PYTHONPATH=/workspace', 'HEADLESS=1', 'OPENBLAS_NUM_THREADS=1', 'OMP_NUM_THREADS=1',
                   '/workspace/.venv-arena/bin/python', '-u', 'scripts/extensions/doors/collect_tasks_lwx.py',
                   *arguments, '--neutral-seconds', str(duration)]
        state.update(command=command, stage='collect', duration_s=duration)
        save()
        with log.open('x') as stream:
            subprocess.run(command, stdin=subprocess.DEVNULL, stdout=stream,
                           stderr=subprocess.STDOUT, check=True)
        neutral = json.loads((args.output / 'result.json').read_text())
        if not neutral.get('neutral_completed') or neutral.get('failure') is not None:
            raise RuntimeError('Replacement neutral failed')
        state.update(complete=True, stage='collected_neutral_requires_independent_review')
        save()
    except Exception as error:
        state['error'] = repr(error)
        save()
        raise


if __name__ == '__main__':
    main()
