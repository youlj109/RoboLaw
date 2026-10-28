"""Current-config planning results and isolated autosaved trajectory notes."""
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from scripts.extensions.replicator.progress_dashboard import (
    AnnotationError, Progress, TRAJECTORY_REPORTS, TRAJECTORY_RETRY_REPORTS, TRAJECTORY_RETRY2_REPORTS, allowed_artifact, handler_for,
)


class TrajectoryDashboardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.progress = Progress(self.root)
        self.task = 'kitchen_bench_replicator_g_shape_banana_bowl'
        self.variant = 'human_default_mpfb'
        self.sid = 'g_shape_banana_bowl__' + self.variant
        self.config = self.write('configs/extensions/replicator/visual_only_review_v1/current.json', {'task_id': self.task})
        self.folder = TRAJECTORY_REPORTS + 'scenes/' + self.sid + '/'
        self.scene = dict(scene_id=self.sid, task_id=self.task, variant_id=self.variant, variant_name='人体',
                          category='human', config_present=True, config_sha256=self.sha(self.config), views=[])
        self.scenes = {'entries': [self.scene]}
        self.progress.visual_only_review = lambda tasks: self.scenes
        self.write('student_skills/robolaw-replicator/references/assignment.json', {'tasks': [{'id': self.task}]})

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, relative, data):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data if isinstance(data, bytes) else json.dumps(data).encode())
        return path

    def sha(self, path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def artifact(self, name='trajectory.npz', data=b'trajectory'):
        path = self.write(self.folder + name, data)
        return {'path': path.relative_to(self.root).as_posix(), 'sha256': self.sha(path)}

    def result(self, **changes):
        row = {key: self.scene[key] for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256')}
        row.update(status='generated', stage='complete', planning_only=True, artifacts={'trajectory': self.artifact()})
        row.update(changes)
        self.write(self.folder + 'result.json', row)
        return row

    def snapshot(self):
        return self.progress.trajectory_generation(self.scenes)

    def payload(self, **changes):
        entry = self.snapshot()['entries'][0]
        payload = dict(scene_id=self.sid, status='adjust', note='先抬升，再绕过杯沿', expected_revision=0,
                       config_sha256=entry['config_sha256'], result_sha256=entry['result_sha256'])
        payload.update(changes)
        return payload

    def test_pending_and_full_current_generation_are_separate_from_execution(self):
        self.assertEqual(self.snapshot()['counts']['pending'], 1)
        self.result(executed=True, task_success_verified=True, accepted=True)
        snapshot = self.snapshot()
        row = snapshot['entries'][0]
        self.assertEqual(snapshot['counts']['generated'], 1)
        self.assertTrue(row['trajectory_url'].endswith('/trajectory.npz'))
        self.assertEqual(row['label'], '人体')
        self.assertFalse(row['executed'])
        self.assertFalse(row['task_success_verified'])
        self.assertNotIn('accepted', row)

    def test_terminal_progress_without_saved_result_is_never_generated(self):
        row = self.result()
        (self.root / (self.folder + 'result.json')).unlink()
        self.write(TRAJECTORY_REPORTS + 'progress.json', {'entries': [row]})
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        self.assertEqual(self.snapshot()['entries'][0]['stage'], 'awaiting_result')
        row['status'] = 'running'
        self.write(TRAJECTORY_REPORTS + 'progress.json', {'entries': [row]})
        self.assertEqual(self.snapshot()['counts']['running'], 1)

    def test_mismatched_config_identity_and_duplicate_sources_never_count(self):
        for change in ({'config_sha256': 'f' * 64}, {'task_id': 'another'}, {'scene_id': 'another'}, {'variant_id': 'another'}):
            with self.subTest(change=change):
                self.result(**change)
                self.assertEqual(self.snapshot()['counts']['generated'], 0)
                self.assertTrue(self.snapshot()['entries'][0]['stale'])
        row = self.result()
        self.write(TRAJECTORY_REPORTS + 'progress.json', {'entries': [row, row]})
        self.assertEqual(self.snapshot()['counts']['generated'], 0)

    def test_missing_changed_foreign_and_partial_artifacts_do_not_prove_generation(self):
        row = self.result()
        (self.root / row['artifacts']['trajectory']['path']).write_bytes(b'changed')
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        foreign = self.write(TRAJECTORY_REPORTS + 'scenes/other/trajectory.npz', b'foreign')
        self.result(artifacts={'trajectory': {'path': foreign.relative_to(self.root).as_posix(), 'sha256': self.sha(foreign)}})
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        self.result(artifacts={'partial_trajectory': self.artifact('partial.npz')})
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        self.result(planning_only=False)
        self.assertEqual(self.snapshot()['counts']['generated'], 0)

    def test_metadata_json_cannot_substitute_for_missing_or_changed_npz(self):
        metadata = self.artifact('trajectory.json', b'{"complete": true, "npz_path": "trajectory.npz"}')
        self.result(artifacts={'trajectory_json': metadata})
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        row = self.result(artifacts={'trajectory': self.artifact(), 'trajectory_json': metadata})
        self.assertEqual(self.snapshot()['counts']['generated'], 1)
        (self.root / row['artifacts']['trajectory']['path']).unlink()
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        self.assertIn('trajectory_json', self.snapshot()['entries'][0]['paths'])

    def test_failed_results_expose_stage_reason_suggestion_and_partial_download(self):
        self.result(status='failed', stage='place', failure_code='IK_FAIL', failure_reason='目标不可达',
                    suggestion='抬高中间点', artifacts={'partial_trajectory': self.artifact('partial.npz')})
        row = self.snapshot()['entries'][0]
        self.assertEqual(row['status'], 'failed')
        self.assertEqual(row['reason'], '目标不可达')
        self.assertEqual(row['suggestion'], '抬高中间点')
        self.assertIn('partial_trajectory', row['paths'])
        self.assertIsNone(row['trajectory_url'])

    def test_later_start_failure_hint_uses_phase_origin_without_altering_evidence(self):
        initial_hint = '检查截图初始机械臂与台面/增广物间隙'
        self.result(status='failed', stage='lift', failure_code='MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION',
                    suggestion=initial_hint, elapsed_seconds=4.631)
        result_path = self.root / (self.folder + 'result.json')
        original = result_path.read_bytes()
        row = self.snapshot()['entries'][0]
        self.assertIn('【抬升】阶段起点（上一阶段末端）', row['suggestion'])
        self.assertIn('该报错不定位具体碰撞物', row['suggestion'])
        self.assertEqual(row['suggestion_scope'], 'manual_adjustment_hint_not_diagnosis')
        self.assertEqual(row['elapsed_seconds'], 4.631)
        self.assertEqual(row['timing']['elapsed_seconds'], 4.631)
        self.assertEqual(result_path.read_bytes(), original)
        self.result(status='failed', stage='pregrasp', failure_code='INVALID_START_STATE_WORLD_COLLISION',
                    suggestion=initial_hint)
        initial_suggestion = self.snapshot()['entries'][0]['suggestion']
        self.assertIn('布局固定', initial_suggestion)
        self.assertIn('单改抓取点或抓取姿态无法消除', initial_suggestion)
        self.assertNotIn(initial_hint, initial_suggestion)
        self.result(status='failed', stage='lift', failure_code='IK_FAIL', suggestion='检查目标姿态')
        self.assertEqual(self.snapshot()['entries'][0]['suggestion'], '检查目标姿态')

    def test_preview_requires_result_config_and_trajectory_hashes(self):
        row = self.result()
        bound = self.snapshot()['entries'][0]
        preview = dict(scene_id=self.sid, config_sha256=self.scene['config_sha256'], result_sha256=bound['result_sha256'],
                       trajectory_sha256=row['artifacts']['trajectory']['sha256'],
                       artifacts={'preview': self.artifact('trajectory_preview.png', b'PNG'),
                                  'interactive': self.artifact('trajectory_preview.html', b'<html></html>')})
        self.write(self.folder + 'preview.json', preview)
        entry = self.snapshot()['entries'][0]
        self.assertTrue(entry['preview_url'].endswith('.png'))
        self.assertTrue(entry['interactive_url'].endswith('.html'))
        for field in ('config_sha256', 'result_sha256', 'trajectory_sha256'):
            with self.subTest(field=field):
                self.write(self.folder + 'preview.json', {**preview, field: 'e' * 64})
                self.assertIsNone(self.snapshot()['entries'][0]['preview_url'])
        self.assertTrue(allowed_artifact(self.folder + 'trajectory.npz'))
        self.assertTrue(allowed_artifact(self.folder + 'trajectory_preview.html'))
        self.assertFalse(allowed_artifact('reports/replicator_unrelated/arbitrary.html'))
        self.assertFalse(allowed_artifact('reports/replicator_unrelated/secret.npz'))

    def test_notes_are_independent_atomic_and_conflict_protected(self):
        original = self.write('reports/replicator_dashboard/scene_annotations.json', b'untouched scene notes')
        self.result(status='failed')
        payload = self.payload()
        saved = self.progress.save_trajectory_annotation(payload)
        self.assertEqual(saved['revision'], 1)
        self.assertEqual(original.read_bytes(), b'untouched scene notes')
        self.assertEqual(Progress(self.root).trajectory_annotations()['entries'], [saved])
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_trajectory_annotation(payload)
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(caught.exception.entry, saved)
        self.assertFalse(list(self.progress.trajectory_annotation_path.parent.glob('.trajectory_annotations-*.tmp')))

    def test_result_change_rejects_notes_and_preserves_saved_text(self):
        self.result(status='failed', failure_reason='previous')
        payload = self.payload()
        saved = self.progress.save_trajectory_annotation(payload)
        self.result(status='failed', failure_reason='new failure')
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_trajectory_annotation({**payload, 'expected_revision': 1})
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(self.progress.trajectory_annotations()['entries'], [saved])
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_trajectory_annotation(self.payload(expected_revision=1, config_sha256='f' * 64))
        self.assertEqual(caught.exception.status, 409)

    def retry_progress(self, *, carry_forward=False, **changes):
        path = self.folder + 'result.json'
        row = {key: self.scene[key] for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256')}
        row.update(status='generated' if carry_forward else 'pending', stage='complete' if carry_forward else 'queued',
                   retry_round=0 if carry_forward else 1, carry_forward=carry_forward,
                   prior_result_path=path, prior_result_sha256=self.sha(self.root / path),
                   recipe={'summary': '保持布局，调整碗沿抓点与抬升高度'})
        row.update(changes)
        self.write(TRAJECTORY_RETRY_REPORTS + 'progress.json', {'entries': [row], 'retry_round': 1})
        return row

    def test_scene_adjustment_uses_new_config_but_retains_old_result_binding(self):
        old = {'task_id': self.task, 'obstacle': {'position': [0,0,1], 'yaw_deg': 0, 'usd_path': 'original.usd'}}
        self.config.write_text(json.dumps(old));self.scene['config_sha256']=self.sha(self.config)
        prior=self.result(status='failed',stage='pregrasp',config_path=str(self.config.relative_to(self.root)))
        new=json.loads(json.dumps(old));new['obstacle']['position'][0]=.1
        path=self.write('configs/extensions/replicator/scene_retry_once_20261028/new.json',new)
        change=dict(adjustments=1,source_config_path=str(self.config.relative_to(self.root)),
                    source_config_sha256=self.sha(self.config),config_path=str(path.relative_to(self.root)),config_sha256=self.sha(path))
        recipe=dict(layout_fixed=False,action='plan',scene_adjustment=change)
        self.retry_progress(config_path=change['config_path'],config_sha256=change['config_sha256'],
                            scene_adjustment=change,recipe=recipe)
        row=self.snapshot()['entries'][0]
        self.assertFalse(row['stale']);self.assertTrue(row['previous_evidence_valid'])
        self.assertEqual(row['config_sha256'],self.sha(path));self.assertEqual(row['views'],[])
        self.assertEqual(row['previous_result_sha256'],self.sha(self.root/self.folder/'result.json'))
        path.write_text('{}')
        self.assertTrue(self.snapshot()['entries'][0]['stale'])

    def test_retry_pending_does_not_inherit_previous_failure_or_its_binding(self):
        self.result(status='failed', stage='lift', failure_code='IK_FAIL')
        previous = self.snapshot()['entries'][0]
        self.retry_progress()
        data = self.snapshot()
        row = data['entries'][0]
        self.assertEqual(data['source_progress_path'], TRAJECTORY_RETRY_REPORTS + 'progress.json')
        self.assertEqual(data['retry_round'], 1)
        self.assertEqual(data['round_counts']['pending'], 1)
        self.assertEqual(row['status'], 'pending')
        self.assertIsNone(row['result_sha256'])
        self.assertEqual(row['previous_result_sha256'], previous['result_sha256'])
        self.assertEqual(row['previous_stage'], 'lift')
        self.assertEqual(row['previous_status'], 'failed')
        self.assertIn('调整碗沿抓点', row['change_description'])
        self.assertIsNone(row['trajectory_url'])

    def test_carried_success_is_verified_and_not_counted_as_a_retry(self):
        self.result()
        previous = self.snapshot()['entries'][0]
        row = self.retry_progress(carry_forward=True)
        data = self.snapshot()
        self.assertEqual(data['counts']['generated'], 1)
        self.assertEqual(data['carried_generated'], 1)
        self.assertEqual(data['round_counts']['total'], 0)
        self.assertEqual(data['entries'][0]['result_sha256'], previous['result_sha256'])
        self.assertEqual(data['entries'][0]['trajectory_url'], previous['trajectory_url'])
        self.retry_progress(carry_forward=True, prior_result_sha256='f' * 64)
        self.assertEqual(self.snapshot()['counts']['generated'], 0)
        self.assertTrue(self.snapshot()['entries'][0]['stale'])
        self.retry_progress(carry_forward=True, prior_result_path='reports/replicator_other/result.json')
        self.assertEqual(self.snapshot()['counts']['generated'], 0)

    def test_retry_result_changes_binding_preserving_original_notes_and_files(self):
        self.result(status='failed', failure_reason='first attempt')
        payload = self.payload()
        saved = self.progress.save_trajectory_annotation(payload)
        original_path = self.root / (self.folder + 'result.json')
        original_bytes = original_path.read_bytes()
        self.retry_progress()
        self.folder = TRAJECTORY_RETRY_REPORTS + 'scenes/' + self.sid + '/'
        self.result(status='generated')
        data = self.snapshot()
        self.assertEqual(data['counts']['generated'], 1)
        self.assertEqual(data['round_counts']['generated'], 1)
        self.assertEqual(data['carried_generated'], 0)
        self.assertNotEqual(data['entries'][0]['result_sha256'], saved['result_sha256'])
        self.assertTrue(data['entries'][0]['trajectory_url'].startswith('/artifact/' + TRAJECTORY_RETRY_REPORTS))
        self.assertEqual(original_path.read_bytes(), original_bytes)
        self.assertEqual(self.progress.trajectory_annotations()['entries'], [saved])
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_trajectory_annotation({**payload, 'expected_revision': 1})
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(self.progress.trajectory_annotations()['entries'], [saved])
        self.assertTrue(allowed_artifact(self.folder + 'trajectory.npz'))
        self.assertTrue(allowed_artifact(self.folder + 'trajectory_preview.html'))

    def test_retry_rejects_changed_prior_result_and_successful_scene_replanning(self):
        self.result(status='generated')
        self.retry_progress()
        self.folder = TRAJECTORY_RETRY_REPORTS + 'scenes/' + self.sid + '/'
        self.result()
        self.assertTrue(self.snapshot()['entries'][0]['stale'])
        self.assertEqual(self.snapshot()['counts']['generated'], 0)

    def test_second_retry_retains_original_artifacts_through_previous_copy(self):
        result = self.result()
        original = self.snapshot()['entries'][0]
        previous_path = self.write(TRAJECTORY_RETRY_REPORTS+'scenes/'+self.sid+'/result.json', result)
        row = dict(result, carry_forward=True, retry_round=0,
            prior_result_path=previous_path.relative_to(self.root).as_posix(),
            prior_result_sha256=self.sha(previous_path))
        self.write(TRAJECTORY_RETRY2_REPORTS+'progress.json', dict(entries=[row], retry_round=2,
            source_run=TRAJECTORY_RETRY_REPORTS.rstrip('/')))
        data = self.snapshot()
        self.assertEqual(data['retry_round'], 2)
        self.assertEqual(data['carried_generated'], 1)
        self.assertEqual(data['round_counts']['total'], 0)
        self.assertEqual(data['entries'][0]['trajectory_url'], original['trajectory_url'])

    def test_second_retry_uses_immediate_previous_result_binding(self):
        self.folder = TRAJECTORY_RETRY_REPORTS+'scenes/'+self.sid+'/'
        result = self.result(status='failed', stage='lower')
        previous_path = self.root/(self.folder+'result.json')
        row = dict(result, status='pending', stage='queued', carry_forward=False, retry_round=2,
            prior_result_path=previous_path.relative_to(self.root).as_posix(),
            prior_result_sha256=self.sha(previous_path), artifacts={})
        self.write(TRAJECTORY_RETRY2_REPORTS+'progress.json', dict(entries=[row], retry_round=2,
            source_run=TRAJECTORY_RETRY_REPORTS.rstrip('/')))
        data = self.snapshot()
        self.assertEqual(data['counts']['stale'], 0)
        self.assertEqual(data['round_counts']['pending'], 1)
        self.assertEqual(data['entries'][0]['previous_stage'], 'lower')
        self.assertTrue(allowed_artifact(TRAJECTORY_RETRY2_REPORTS+'scenes/'+self.sid+'/trajectory.npz'))

    def test_corrupt_notes_reject_write_and_keep_file_bytes(self):
        self.result(status='failed')
        self.progress.trajectory_annotation_path.parent.mkdir(parents=True)
        self.progress.trajectory_annotation_path.write_bytes(b'broken notes')
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_trajectory_annotation(self.payload())
        self.assertEqual(caught.exception.status, 503)
        self.assertEqual(self.progress.trajectory_annotation_path.read_bytes(), b'broken notes')

    def test_future_round_discovery_exposes_annotation_without_replacing_exact_diagnosis(self):
        previous_root = 'reports/replicator_trajectory_retry3_20261026/'
        current_root = 'reports/replicator_trajectory_retry10_20261026/'
        self.folder = previous_root+'scenes/'+self.sid+'/'
        prior = self.result(status='failed', stage='lift', failure_code='INVALID_START_STATE_WORLD_COLLISION')
        prior_path = self.root/(self.folder+'result.json')
        row = dict(prior, retry_round=10, carry_forward=False,
                   prior_result_path=prior_path.relative_to(self.root).as_posix(),
                   prior_result_sha256=self.sha(prior_path))
        self.write(current_root+'progress.json', dict(entries=[row], retry_round=10,
            source_run=previous_root.rstrip('/'), round_actions={'planned':0,'annotated':1}))
        # Analysis directories must never shadow a real numbered round.
        self.write('reports/replicator_trajectory_retry99_analysis_20261026/progress.json', {})
        self.folder = current_root+'scenes/'+self.sid+'/'
        note = '原始网格没有相交；保守携物球在固定物体位置受阻，本轮只标注。'
        self.result(status='failed', stage='lift', failure_code='INVALID_START_STATE_WORLD_COLLISION',
                    annotation_only=True, suggestion=note,
                    artifacts={'partial_trajectory': self.artifact('partial.npz')})
        data = self.snapshot()
        self.assertEqual(data['retry_round'], 10)
        self.assertEqual(data['source_progress_path'], current_root+'progress.json')
        self.assertEqual(data['round_actions'], {'planned':0,'annotated':1})
        self.assertEqual(data['counts']['failed'], 1)
        self.assertTrue(data['entries'][0]['annotation_only'])
        self.assertEqual(data['entries'][0]['suggestion'], note)
        self.assertIn('partial_trajectory', data['entries'][0]['paths'])
        self.assertTrue(allowed_artifact(self.folder+'partial.npz'))
        self.assertFalse(allowed_artifact('reports/replicator_trajectory_retry99_analysis_20261026/scenes/s/partial.npz'))

    def test_http_independent_endpoint_and_cross_site_protection(self):
        self.result(status='failed')
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(self.progress))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = 'http://127.0.0.1:' + str(server.server_port) + '/api/trajectory-annotations'
        try:
            with urllib.request.urlopen(url) as response:
                self.assertEqual(json.load(response)['entries'], [])
            request = urllib.request.Request(url, data=json.dumps(self.payload()).encode(), headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(json.load(response)['entry']['revision'], 1)
            bad = urllib.request.Request(url, data=json.dumps(self.payload(expected_revision=1)).encode(),
                                         headers={'Content-Type': 'application/json', 'Origin': 'http://evil.example'})
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(bad)
            self.assertEqual(caught.exception.code, 403)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
