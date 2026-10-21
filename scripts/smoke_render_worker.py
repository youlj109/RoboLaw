#!/usr/bin/env python3
"""Host-only launcher for sequential requests against one real renderer Kit.

The report measures service lifecycle, not independently validated RGB quality.
Run validate_collection --render-output separately after this smoke test.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.render_worker import RenderWorker


def run(sources, output, *, refresh_render_products=False, worker_factory=RenderWorker):
    output = Path(output).resolve()
    output.relative_to(ROOT)
    output.mkdir(parents=True, exist_ok=False)
    report = {'completed': False, 'production_quota_eligible': False,
              'scope': 'Sequential persistent renderer lifecycle smoke; full RGB validation is separate',
              'grouped_camera': False, 'refresh_render_products': bool(refresh_render_products),
              'episodes': []}
    started = time.perf_counter()
    worker = worker_factory(cwd=ROOT, directory=output / 'worker',
                            refresh_render_products=refresh_render_products)

    def save():
        temporary = output / 'summary.tmp'
        temporary.write_text(json.dumps(report, indent=2) + '\n')
        temporary.replace(output / 'summary.json')

    save()
    try:
        for index, source in enumerate(sources):
            row = {'index': index, 'source_episode': str(Path(source).resolve()),
                   'output': str(output / f'env_{index:03d}'),
                   'request_start_relative_s': time.perf_counter() - started}
            report['episodes'].append(row)
            save()
            row['request'] = worker.execute(row['source_episode'], row['output'])
            row['request_finish_relative_s'] = time.perf_counter() - started
            row['worker_alive_after_request'] = worker.process is not None and worker.process.poll() is None
            report['worker'] = worker.report()
            save()
            if row['request'].get('worker_unavailable'):
                break
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
        raise
    finally:
        report['shutdown_start_relative_s'] = time.perf_counter() - started
        try:
            worker.close()
        except BaseException as error:
            report['shutdown_error'] = repr(error)
            raise
        finally:
            report['worker'] = worker.report()
            report['total_wall_s'] = time.perf_counter() - started
            report['completed'] = (len(report['episodes']) == len(sources)
                and all(row.get('request', {}).get('status') == 'completed' for row in report['episodes'])
                and report['worker'].get('process_reaped') is True
                and report['worker'].get('exit_code') == 0
                and 'error' not in report and 'shutdown_error' not in report)
            save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-episodes', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--refresh-render-products', action='store_true')
    args = parser.parse_args()
    report = run(args.source_episodes, args.output, refresh_render_products=args.refresh_render_products)
    print(json.dumps({key: report[key] for key in ('completed', 'total_wall_s', 'worker')}, indent=2))
    return 0 if report['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
