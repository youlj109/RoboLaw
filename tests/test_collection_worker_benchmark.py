"""Check work distribution and output isolation without a simulator."""
import json
from pathlib import Path
import tempfile
import unittest

from scripts.benchmark_collection_workers import benchmark


class FakeWorker:
    def __init__(self, **kwargs):
        self.kit_log_path = Path(kwargs['kit_log_path'])
        self.shutdown = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.shutdown = {'exit_code': 0, 'forced': False}

    def execute(self, command, log_path):
        output = Path(command[command.index('--output') + 1])
        assert output.parent == self.kit_log_path.parent
        log_path.write_text('fake collector\n')
        (output / 'result.json').write_text(json.dumps({'success': True, 'steps': 3}))
        return {'exit_code': 0, 'started_at': 'test', 'elapsed_seconds': .01}


class DistributionTests(unittest.TestCase):
    def test_shard_uneven_and_repeat_preserve_all_specs_and_isolate_outputs(self):
        for mode, count, workers, expected in [('shard', 7, 3, [3, 2, 2]), ('repeat', 3, 2, [3, 3])]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                source = [{'id': f'episode_{index}', 'argv': ['--output', f'old/{index}', '--seed', str(index)]}
                          for index in range(count)]
                path = root / 'input.json'
                path.write_text(json.dumps(source))
                constructors = []
                def factory(**kwargs):
                    constructors.append(kwargs)
                    return FakeWorker(**kwargs)
                concurrent_startup = mode == 'repeat'
                report = benchmark(path, root / 'outputs', workers=workers,
                                   distribution=mode, worker_factory=factory,
                                   concurrent_startup=concurrent_startup)
                self.assertTrue(report['passed'])
                self.assertEqual(report['concurrent_startup'], concurrent_startup)
                locks = [kwargs['startup_lock'] for kwargs in constructors]
                self.assertTrue(all(lock is None for lock in locks) if concurrent_startup
                                else all(lock is not None and lock is locks[0] for lock in locks))
                self.assertFalse(report['production_quota_eligible'])
                self.assertEqual(report['episodes_requested_per_worker'], expected)
                self.assertEqual(report['requested_total'], sum(expected))
                all_outputs = []
                for worker in report['workers']:
                    indices = [row['source_episode_index'] for row in worker['episodes']]
                    self.assertEqual(indices, list(range(worker['worker_index'], count, workers))
                                     if mode == 'shard' else list(range(count)))
                    all_outputs.extend(row['output'] for row in worker['episodes'])
                    self.assertEqual(worker['shutdown']['exit_code'], 0)
                self.assertEqual(len(set(all_outputs)), sum(expected))
                self.assertEqual(json.loads(path.read_text()), source)


if __name__ == '__main__':
    unittest.main()
