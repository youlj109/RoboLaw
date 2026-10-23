"""Verify progress semantics and read-only HTTP artifact boundaries."""
import hashlib
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from scripts.extensions.replicator.progress_dashboard import Progress, handler_for, physical_success


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.tid = 'kitchen_bench_replicator_g_shape_banana_bowl'
        self.write('student_skills/robolaw-replicator/references/assignment.json', {'tasks': [{'id': self.tid}]})
        self.progress = Progress(self.root)
        self.progress.processes = lambda tasks: []

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, rel, value):
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(value))
        return p

    def test_physical_success_does_not_complete_task(self):
        rel = 'datasets/replicator/g_shape_banana_bowl/human/attempt_01/result.json'
        self.write(rel, {'task_id': self.tid, 'variant': 'human', 'accepted': True,
                         'physical_trial_passed': True, 'original_task_success': True})
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 1)
        self.assertEqual(data['tasks'][0]['status'], 'partial')
        self.write(rel, {'task_id': self.tid, 'variant': 'human', 'status': 'failed', 'error': 'real failure'})
        self.progress.cached_at = 0
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['successful_variants'], 0)
        self.assertEqual(data['tasks'][0]['latest']['reason'], 'real failure')

    def test_legacy_pass_flag_not_delivery(self):
        self.assertFalse(self.progress.manifest_valid({'task_id': self.tid, 'passed': True, 'accepted': True}, {self.tid: {}}))

    def test_original_collector_success_and_conflicting_flags(self):
        raw = dict(success=True, arena_success=True, physics_grasp=True, final_on_destination=True)
        self.assertTrue(physical_success(raw))
        for field in raw:
            self.assertFalse(physical_success({**raw, field: False}))
        for changed in ({'physical_trial_passed': False}, {'original_task_success': False},
                        {'status': 'failed'}, {'error': 'aborted'},
                        {'physical_failure_reasons': ['contact']}, {'clearance_failure': 'collision'}):
            self.assertFalse(physical_success({**raw, **changed}))
        self.assertFalse(physical_success({'accepted': True, 'success': True}))

    def test_neutral_check_and_overview_do_not_certify_task(self):
        p = self.write('reports/replicator_execution_test/g_banana/neutral/neutral_probe.json',
                       {'task_id': self.tid, 'passed': True})
        video = p.parent / 'overview/overview_camera.mp4'
        video.parent.mkdir()
        video.write_bytes(b'video fixture')
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)
        self.assertEqual(data['recent_events'][0]['status'], 'check_passed')
        evidence = data['tasks'][0]['evidence'][0]
        self.assertTrue(any(a['url'].endswith('/overview/overview_camera.mp4') for a in evidence['artifacts']))

    def test_certification_updates_only_bound_episode_and_event(self):
        manifest = self.delivery_fixture()
        for variant in ('base', 'human', 'property', 'robot'):
            rel = f'datasets/replicator/demo/{variant}/one/result.json'
            raw = dict(task_id=self.tid, variant=variant, success=True, arena_success=True,
                       physics_grasp=True, final_on_destination=True)
            p = self.write(rel, raw)
            manifest['artifacts']['/workspace/' + rel] = {'bytes': p.stat().st_size,
                                                         'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
        self.write('reports/replicator_execution_test/delivery_manifest_test.json', manifest)
        failed = 'datasets/replicator/demo/human/failed/result.json'
        self.write(failed, {'task_id': self.tid, 'variant': 'human', 'error': 'real failure'})
        data = self.progress.snapshot()
        task = data['tasks'][0]
        self.assertTrue(task['completed'])
        episodes = {e['path']: e for e in task['variants']['human']['episodes']}
        self.assertEqual(episodes[failed]['status'], 'failed')
        certified = episodes['datasets/replicator/demo/human/one/result.json']
        self.assertEqual(certified['status'], 'complete')
        self.assertTrue(certified['physical_success'])
        events = {e['label']: e for e in data['recent_events']}
        self.assertEqual(events[failed]['status'], 'failed')
        self.assertEqual(events['datasets/replicator/demo/property/one/result.json']['status'], 'complete')

    def delivery_fixture(self):
        gates = ['physical_and_rgb', 'independent_actual', 'sampled_visual', 'complete_startup_physics',
                 'absolute_motion_and_provenance', 'causal_and_source_geometry', 'three_seed_free_transfer',
                 'comparison_and_explicit_visual', 'source_explanation_and_explicit_visual']
        spec = {'task_id': self.tid, 'planning_seeds': [42, 43, 44],
                'baselines': {'base': {'episode': 'datasets/replicator/demo/base/one'}},
                'variants': {k: {'baseline': 'base', 'episode': f'datasets/replicator/demo/{k}/one'} for k in ('human', 'property', 'robot')}}
        files = [self.write('configs/extensions/replicator/delivery/test.json', spec)]
        for entry in [spec['baselines']['base'], *spec['variants'].values()]:
            files.append(self.write(entry['episode'] + '/result.json', {'status': 'success'}))
        checks = {name: {'passed': True} for name in ['complete_specification', 'source_assets_and_implementation', 'neutral_and_mobility']}
        checks.update({f'baseline.base.{k}': {'passed': True} for k in gates[:5]})
        checks.update({f'{v}.{k}': {'passed': True} for v in ('human', 'property', 'robot') for k in gates})
        return {'task_id': self.tid, 'passed': True, 'specification': spec, 'checks': checks,
                'coverage': {'current_task_complete': True, 'variants': {v: {k: True for k in gates} for v in ('human', 'property', 'robot')}},
                'artifacts': {'/workspace/' + p.relative_to(self.root).as_posix(): {'bytes': p.stat().st_size,
                              'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}}

    def test_complete_report_requires_named_gates_and_bound_spec(self):
        manifest = self.delivery_fixture()
        tasks = {self.tid: {}}
        self.assertTrue(self.progress.manifest_valid(manifest, tasks))
        bad = copy.deepcopy(manifest)
        bad['checks'] = {'unrelated_check': {'passed': True}}
        self.assertFalse(self.progress.manifest_valid(bad, tasks))
        bad = copy.deepcopy(manifest)
        bad['specification']['variants']['human']['episode'] = '/etc'
        self.assertFalse(self.progress.manifest_valid(bad, tasks))
        bad = copy.deepcopy(manifest)
        del bad['specification']['variants']
        self.assertFalse(self.progress.manifest_valid(bad, tasks))
        bad = copy.deepcopy(manifest)
        bad['coverage']['variants']['human'] = None
        self.assertFalse(self.progress.manifest_valid(bad, tasks))

    def test_inventory_detects_changed_same_length_file(self):
        p = self.write('reports/replicator_test/evidence.json', {'value': 1})
        manifest = {'artifacts': {'/workspace/reports/replicator_test/evidence.json': {
            'bytes': p.stat().st_size, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}}}
        self.assertTrue(self.progress.manifest_inventory_present(manifest))
        self.write('reports/replicator_test/evidence.json', {'value': 2})
        self.assertFalse(self.progress.manifest_inventory_present(manifest))

    def test_http_range_and_path_escape(self):
        p = self.root / 'datasets/replicator/demo/clip.mp4'
        p.parent.mkdir(parents=True)
        p.write_bytes(b'0123456789')
        (p.parent / 'escape.txt').symlink_to('/etc/passwd')
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(self.progress))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            request = urllib.request.Request(base + '/artifact/datasets/replicator/demo/clip.mp4', headers={'Range': 'bytes=2-5'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(response.status, 206)
                self.assertEqual(response.headers['Content-Range'], 'bytes 2-5/10')
                self.assertEqual(response.read(), b'2345')
            for path in ('/artifact/../../etc/passwd', '/artifact/datasets/replicator/demo/escape.txt', '/artifact/.git/config'):
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(base + path)
                self.assertEqual(error.exception.code, 404)
                error.exception.close()
            with urllib.request.urlopen(base + '/api/progress') as response:
                self.assertEqual(json.load(response)['summary']['total_tasks'], 1)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
