"""Sequential native-environment throughput sweep with whole-GPU telemetry.

Run from the host with Python 3. It invokes the existing Arena container once
per candidate; Kit processes never overlap. The benchmark owns workload timing.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--counts', type=int, nargs='+', required=True)
    parser.add_argument('--steps', type=int, default=180)
    parser.add_argument('--source-episode', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record-rgb', action='store_true')
    parser.add_argument('--render-spp', type=int, choices=[1, 2, 4, 8])
    args = parser.parse_args()
    if any(count < 1 for count in args.counts):
        parser.error('Environment counts must be positive')
    root = Path(__file__).resolve().parents[1]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'counts': args.counts, 'steps': args.steps,
              'record_rgb': args.record_rgb, 'render_spp': args.render_spp, 'runs': [],
              'semantics': 'Same source action replay, sequential native environment counts; '
                           'short runs are throughput diagnostics, not completed collection episodes'}

    def save():
        (out / 'sweep.json').write_text(json.dumps(report, indent=2))

    try:
        for count in args.counts:
            target = out / f'envs_{count:02d}'
            relative = target.relative_to(root)
            command = ['bash', 'scripts/arena_python.sh', 'scripts/benchmark_collection_vectorized.py',
                       '--device', 'cuda:0', '--num-envs', str(count), '--steps', str(args.steps),
                       '--source-episode', args.source_episode, '--output', str(relative)]
            if args.record_rgb:
                command.append('--record-rgb')
            if args.render_spp is not None:
                command += ['--render-spp', str(args.render_spp)]
            row = {'num_envs': count, 'output': str(relative), 'command': command, 'status': 'running'}
            report['runs'].append(row)
            save()
            print('SWEEP_START', count, flush=True)
            tick = time.perf_counter()
            samples = []
            with (out / f'envs_{count:02d}.log').open('w') as log:
                process = subprocess.Popen(command, cwd=str(root), stdout=log, stderr=subprocess.STDOUT)
                try:
                    while process.poll() is None:
                        measured = subprocess.run(['nvidia-smi',
                            '--query-gpu=memory.used,memory.total,utilization.gpu,utilization.memory',
                            '--format=csv,noheader,nounits'], capture_output=True, text=True)
                        if measured.returncode == 0:
                            fields = measured.stdout.strip().splitlines()[0].split(',')
                            samples.append({'elapsed_s': time.perf_counter()-tick,
                                            **dict(zip(('memory_used_mib', 'memory_total_mib',
                                                        'gpu_utilization_percent', 'memory_utilization_percent'),
                                                       [int(value.strip()) for value in fields]))})
                        time.sleep(2.)
                except BaseException:
                    # Stop only the host launch handle we own; interruption is
                    # surfaced rather than silently starting another candidate.
                    report['status'] = 'interrupted'
                    save()
                    raise
                row.update(status='completed' if process.returncode == 0 else 'failed',
                           exit_code=process.returncode, launch_to_shutdown_s=time.perf_counter()-tick)
            (out / f'envs_{count:02d}_gpu.json').write_text(json.dumps(samples, indent=2))
            row['whole_gpu_peak_sampled_memory_mib'] = max((x['memory_used_mib'] for x in samples), default=None)
            row['gpu_sample_interval_s'] = 2.
            row['gpu_memory_scope'] = 'Whole GPU including desktop and non-benchmark allocations; sampled peak, not continuous peak'
            artifact = target / 'benchmark.json'
            if artifact.exists():
                benchmark = json.loads(artifact.read_text())
                row['benchmark_completed'] = benchmark.get('completed', False)
                row['aggregate_control_steps_per_second'] = benchmark.get('aggregate_control_steps_per_second')
                row['timings_seconds'] = benchmark.get('timings_seconds')
            save()
            print('SWEEP_END', json.dumps(row), flush=True)
            if process.returncode:
                report['status'] = 'stopped_after_failure'
                break
        else:
            report['status'] = 'completed'
    finally:
        save()


if __name__ == '__main__':
    main()
