"""Dashboard evidence remains bound across consecutive hazard-pose retry rounds."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.extensions.replicator.progress_dashboard import Progress


class TrajectoryDashboardLineageTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.progress = Progress(self.root)
        self.sid = 'l_shape_bowl_sink__robot_projecting_crossbar'
        self.identity = dict(scene_id=self.sid,
            task_id='kitchen_bench_replicator_l_shape_bowl_sink',
            variant_id='robot_projecting_crossbar')
        self.runs = {round_: f'reports/replicator_trajectory_retry{round_}_20261028/'
                     for round_ in (7, 8, 9)}
        self.configs = {}
        for round_, yaw in ((7, 15), (8, 45), (9, 90)):
            path = self.write(f'configs/extensions/replicator/round{round_}/scene.json',
                dict(task_id=self.identity['task_id'], robot_position=[1, 2, 0],
                     obstacle=dict(position=[0.1, 0.2, 0.9], yaw_deg=yaw,
                                   usd_path='original_hazard.usd', scale=[1, 1, 1])))
            self.configs[round_] = dict(config_path=self.relative(path), config_sha256=self.sha(path))
        self.scenes = {'entries': [dict(self.identity, **self.configs[7],
            variant_name='突出横杆', category='robot', config_present=True,
            views=[{'view_id': 'original', 'preview_url': '/original.png'}])]}
        self.write_result(7, dict(self.identity, **self.configs[7], status='failed', stage='pregrasp'))
        self.change8 = self.adjustment(7, 8)
        self.result8 = dict(self.identity, **self.configs[8], status='failed', stage='pregrasp',
            planning_only=True, scene_adjustment=self.change8,
            **self.prior_binding(7))
        self.write_result(8, self.result8)
        self.source8 = self.write_progress(8, [self.result8])
        self.views8 = [{'view_id': 'round8', 'preview_url': '/new_round8.png'}]
        self.write_views(8, self.source8, self.configs[8]['config_sha256'], self.views8)

    def write(self, relative, data):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data if isinstance(data, bytes) else json.dumps(data).encode())
        return path

    def relative(self, path):
        return path.relative_to(self.root).as_posix()

    def sha(self, path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def result_path(self, round_):
        return self.runs[round_] + f'scenes/{self.sid}/result.json'

    def write_result(self, round_, data):
        return self.write(self.result_path(round_), data)

    def prior_binding(self, round_):
        return dict(prior_result_path=self.result_path(round_),
                    prior_result_sha256=self.sha(self.root / self.result_path(round_)))

    def adjustment(self, old_round, new_round):
        old, new = self.configs[old_round], self.configs[new_round]
        return dict(adjustments=1, source_config_path=old['config_path'],
                    source_config_sha256=old['config_sha256'], **new)

    def write_progress(self, round_, entries):
        recipe = self.write(f'configs/trajectory_retry_v{round_}.json', {'round': round_})
        source = dict(retry_round=round_, source_run=self.runs[round_ - 1], entries=entries,
                      recipe_path=self.relative(recipe), recipe_sha256=self.sha(recipe))
        self.write(self.runs[round_] + 'progress.json', source)
        return source

    def write_views(self, round_, source, config_sha, views, **changes):
        document = dict(recipe_path=source['recipe_path'], recipe_sha256=source['recipe_sha256'],
                        entries=[dict(scene_id=self.sid, config_sha256=config_sha, views=views)])
        document.update(changes)
        return self.write(self.runs[round_] + 'scene_views.json', document)

    def start_round9(self, *, carry=False, change=False):
        row = dict(self.identity, **self.configs[9 if change else 8],
            status='generated' if carry else 'pending', stage='complete' if carry else 'queued',
            retry_round=8 if carry else 9, carry_forward=carry, **self.prior_binding(8),
            recipe={} if carry else dict(action='plan', layout_fixed=True, summary='Adjust target only'))
        if change:
            row['scene_adjustment'] = self.adjustment(8, 9)
            row['recipe'] = dict(action='plan', layout_fixed=False, scene_adjustment=row['scene_adjustment'])
        return row, self.write_progress(9, [row])

    def snapshot(self):
        return self.progress.trajectory_generation(self.scenes)

    def assert_round8_binding(self, row):
        self.assertFalse(row['stale'])
        self.assertTrue(row['previous_evidence_valid'])
        self.assertEqual(row['config_sha256'], self.configs[8]['config_sha256'])
        self.assertEqual(row['views'], self.views8)
        self.assertEqual(row['previous_result_sha256'], self.prior_binding(8)['prior_result_sha256'])
        self.assertTrue(row['previous_result_url'].endswith(self.result_path(8)))

    def test_target_only_round_inherits_last_changed_layout_and_images(self):
        row, _ = self.start_round9()
        self.assert_round8_binding(self.snapshot()['entries'][0])
        # Once a new result exists it still binds R8, with unchanged scene images.
        result = dict(row, status='failed', stage='transfer', planning_only=True, artifacts={})
        path = self.write_result(9, result)
        entry = self.snapshot()['entries'][0]
        self.assert_round8_binding(entry)
        self.assertEqual(entry['status'], 'failed')
        self.assertEqual(entry['result_sha256'], self.sha(path))

    def test_carried_success_keeps_round8_images_result_and_artifact_binding(self):
        trajectory = self.write(self.runs[8] + f'scenes/{self.sid}/trajectory.npz', b'saved R8 trajectory')
        self.result8.update(status='generated', stage='complete', artifacts={'trajectory': dict(
            path=self.relative(trajectory), sha256=self.sha(trajectory))})
        self.write_result(8, self.result8)
        self.start_round9(carry=True)
        snapshot = self.snapshot()
        row = snapshot['entries'][0]
        self.assert_round8_binding(row)
        self.assertTrue(row['carry_forward'])
        self.assertEqual(snapshot['carried_generated'], 1)
        self.assertEqual(snapshot['round_counts']['total'], 0)
        self.assertEqual(row['result_sha256'], self.prior_binding(8)['prior_result_sha256'])
        self.assertTrue(row['trajectory_url'].endswith(self.relative(trajectory)))

    def test_second_layout_change_waits_for_new_bound_images_never_reuses_round8(self):
        _, source = self.start_round9(change=True)
        row = self.snapshot()['entries'][0]
        self.assertFalse(row['stale'])
        self.assertTrue(row['previous_evidence_valid'])
        self.assertEqual(row['config_sha256'], self.configs[9]['config_sha256'])
        self.assertEqual(row['views'], [])
        # Publishing a previous-config image under the current manifest is insufficient.
        self.write_views(9, source, self.configs[8]['config_sha256'], self.views8)
        self.assertEqual(self.snapshot()['entries'][0]['views'], [])
        views9 = [{'view_id': 'round9', 'preview_url': '/new_round9.png'}]
        self.write_views(9, source, self.configs[9]['config_sha256'], views9)
        self.assertEqual(self.snapshot()['entries'][0]['views'], views9)
        self.write_views(9, source, self.configs[9]['config_sha256'], views9, recipe_sha256='0' * 64)
        self.assertEqual(self.snapshot()['entries'][0]['views'], [])

    def test_tampered_ancestor_result_is_stale_even_with_valid_immediate_binding(self):
        self.start_round9()
        self.assert_round8_binding(self.snapshot()['entries'][0])
        path = self.root / self.result_path(7)
        prior = json.loads(path.read_text())
        prior['failure_reason'] = 'tampered ancestor'
        self.write_result(7, prior)
        row = self.snapshot()['entries'][0]
        self.assertTrue(row['stale'])
        self.assertFalse(row['previous_evidence_valid'])
        self.assertIsNone(row['previous_result_sha256'])
        self.assertIsNone(row['previous_result_url'])
        self.assertEqual(self.snapshot()['counts']['generated'], 0)

    def test_tampered_changed_config_invalidates_descendant_lineage(self):
        self.start_round9()
        self.assert_round8_binding(self.snapshot()['entries'][0])
        config = self.root / self.configs[8]['config_path']
        data = json.loads(config.read_text())
        data['obstacle']['yaw_deg'] += 1
        self.write(self.configs[8]['config_path'], data)
        row = self.snapshot()['entries'][0]
        self.assertTrue(row['stale'])
        self.assertFalse(row['previous_evidence_valid'])


if __name__ == '__main__':
    unittest.main()
