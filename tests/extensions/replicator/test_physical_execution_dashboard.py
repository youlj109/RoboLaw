"""Physical counts and videos require bound execution evidence, not a plan flag."""
from copy import deepcopy
import hashlib
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from scripts.extensions.replicator.progress_dashboard import (
    PHYSICAL_DEMOS, PHYSICAL_SOURCE, Progress, allowed_artifact, handler_for,
)


class PhysicalDashboardTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.progress = Progress(self.root)
        self.sid = 'g_shape_banana_bowl__human_default_mpfb'
        self.identity = dict(scene_id=self.sid, task_id='kitchen_bench_replicator_g_shape_banana_bowl',
                             variant_id='human_default_mpfb', variant_name='人体')
        config = self.write('configs/extensions/replicator/current.json', {'task_id': self.identity['task_id']})
        trajectory = self.write(PHYSICAL_SOURCE + 'scenes/' + self.sid + '/trajectory.npz', b'bound NPZ')
        self.planning = dict(self.identity, status='generated', config_path=self.rel(config),
            config_sha256=self.sha(config), artifacts={'trajectory': dict(path=self.rel(trajectory), sha256=self.sha(trajectory))},
            scene_adjustment={'scene_quality_issues': ['历史人体衣服与橱柜相交，需复核']})
        result = self.write(PHYSICAL_SOURCE + 'scenes/' + self.sid + '/result.json', self.planning)
        self.source = self.write(PHYSICAL_SOURCE + 'progress.json', {'stage': 'complete', 'entries': [self.planning]})
        self.row = dict(self.identity, config_path=self.rel(config), config_sha256=self.sha(config),
            result_path=self.rel(result), result_sha256=self.sha(result),
            trajectory_path=self.rel(trajectory), trajectory_sha256=self.sha(trajectory),
            planning_status='generated', execution_status='pending', attempts=0)
        self.manifest = dict(kind='physical_execution', source_run=PHYSICAL_SOURCE.rstrip('/'),
                             source_progress_sha256=self.sha(self.source), entries=[self.row])
        self.note = self.write('reports/replicator_dashboard/trajectory_annotations.json', b'unchanged manual notes')

    def write(self, relative, value):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else json.dumps(value).encode())
        return path

    def rel(self, path):
        return path.relative_to(self.root).as_posix()

    def sha(self, path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def save_manifest(self):
        self.write(PHYSICAL_DEMOS + 'manifest.json', self.manifest)

    def terminal(self, status='failed', executed=True):
        video = self.write(PHYSICAL_DEMOS + 'videos/' + self.sid + '.mp4', b'0123456789-video-bytes')
        self.row.update(execution_status=status, executed=executed, task_success=status == 'success', attempts=1,
            execution_result_path='executions/' + self.sid + '/result.json',
            video_path='videos/' + self.sid + '.mp4', video_sha256=self.sha(video))
        self.result = dict(self.row, kind='physical_execution', planning_result_path=self.row['result_path'],
            planning_result_sha256=self.row['result_sha256'], failure_reason='实际执行记录中的原因')
        self.save_result()

    def save_result(self):
        path = self.write(PHYSICAL_DEMOS + self.row['execution_result_path'], self.result)
        self.row['execution_result_sha256'] = self.sha(path)
        self.save_manifest()

    def snapshot(self):
        return self.progress.physical_execution()

    def test_planning_counts_remain_separate_before_physical_manifest_exists(self):
        data = self.snapshot()
        self.assertEqual(data['counts']['planned'], 1)
        self.assertEqual(data['counts']['pending'], 1)
        self.assertEqual(data['counts']['success'], 0)
        self.assertEqual(data['counts']['video_count'], 0)
        self.assertEqual(data['entries'][0]['historical_quality_issues'], self.planning['scene_adjustment']['scene_quality_issues'])

    def test_running_is_not_success_and_polling_leaves_notes_unchanged(self):
        before = self.note.read_bytes()
        self.row.update(execution_status='running', attempts=1)
        self.save_manifest()
        data = self.snapshot()
        self.assertEqual(data['counts']['running'], 1)
        self.assertFalse(data['entries'][0]['result_valid'])
        self.assertIsNone(data['entries'][0]['video_url'])
        self.assertEqual(self.note.read_bytes(), before)

    def test_valid_failed_trial_exposes_actual_result_and_video(self):
        self.terminal()
        data = self.snapshot()
        self.assertEqual(data['counts']['failed'], 1)
        self.assertEqual(data['counts']['video_count'], 1)
        row = data['entries'][0]
        self.assertTrue(row['executed'])
        self.assertFalse(row['task_success'])
        self.assertTrue(row['result_valid'])
        self.assertTrue(row['result_url'].endswith('/result.json'))
        self.assertTrue(row['video_url'].endswith('/' + self.sid + '.mp4'))

    def test_initialization_error_is_neither_task_failure_nor_success(self):
        self.terminal(status='error', executed=False)
        data = self.snapshot()
        self.assertEqual(data['counts']['error'], 1)
        self.assertEqual(data['counts']['initialization_error'], 1)
        self.assertEqual(data['counts']['failed'], 0)
        self.assertEqual(data['counts']['success'], 0)
        self.assertTrue(data['entries'][0]['initialization_error'])
        self.assertFalse(data['entries'][0]['executed'])

    def test_unexecuted_success_and_second_attempt_are_not_counted(self):
        for changes in ({'executed': False}, {'attempts': 2}):
            with self.subTest(changes=changes):
                self.terminal(status='success')
                self.row.update(changes)
                self.result.update(changes)
                self.save_result()
                data = self.snapshot()
                self.assertEqual(data['counts']['success'], 0)
                self.assertEqual(data['counts']['stale'], 1)
                self.assertIsNone(data['entries'][0]['video_url'])

    def test_stale_execution_result_hides_video_even_when_video_hash_matches(self):
        self.terminal()
        self.write(PHYSICAL_DEMOS + self.row['execution_result_path'], dict(self.result, task_success=True))
        data = self.snapshot()
        self.assertEqual(data['counts']['failed'], 0)
        self.assertEqual(data['counts']['stale'], 1)
        self.assertFalse(data['entries'][0]['result_valid'])
        self.assertIsNone(data['entries'][0]['video_url'])

    def test_changed_video_does_not_hide_valid_failed_result(self):
        self.terminal()
        self.write(PHYSICAL_DEMOS + self.row['video_path'], b'changed video')
        data = self.snapshot()
        self.assertEqual(data['counts']['failed'], 1)
        self.assertEqual(data['counts']['video_count'], 0)
        self.assertTrue(data['entries'][0]['result_valid'])
        self.assertIsNone(data['entries'][0]['video_url'])

    def test_duplicate_manifest_scene_and_changed_planning_source_are_stale(self):
        self.terminal()
        self.manifest['entries'].append(deepcopy(self.row))
        self.save_manifest()
        self.assertEqual(self.snapshot()['counts']['stale'], 1)
        self.manifest['entries'].pop()
        self.save_manifest()
        self.write(self.row['result_path'], dict(self.planning, changed=True))
        self.assertEqual(self.snapshot()['counts']['stale'], 1)

    def test_traversal_and_other_scene_video_are_not_served(self):
        self.terminal()
        for video_path in ('../../secret.mp4', 'videos/another_scene.mp4'):
            with self.subTest(video_path=video_path):
                self.row['video_path'] = self.result['video_path'] = video_path
                self.save_result()
                self.assertIsNone(self.snapshot()['entries'][0]['video_url'])
        self.assertFalse(allowed_artifact('exports/another/secret.mp4'))
        self.assertFalse(allowed_artifact(PHYSICAL_DEMOS + 'secrets.json'))

    def test_http_range_and_unbound_video_rejection(self):
        self.terminal()
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(self.progress))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            url = 'http://127.0.0.1:' + str(server.server_port) + self.snapshot()['entries'][0]['video_url']
            request = urllib.request.Request(url, headers={'Range': 'bytes=2-5'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(response.status, 206)
                self.assertEqual(response.read(), b'2345')
            self.write(PHYSICAL_DEMOS + self.row['execution_result_path'], b'changed result')
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(url)
            self.assertEqual(caught.exception.code, 404)
            caught.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
