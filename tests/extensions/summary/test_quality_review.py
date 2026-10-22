"""A historical successful attempt cannot bypass a later category review.

Tiny placeholder media exercise selection/export gating, not video validation.
No Isaac, GPU or actual trial artifacts are required.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/extensions/summarize_trials.py'
spec = importlib.util.spec_from_file_location('summarize_trials_review_tests', SOURCE)
trials = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trials)


class QualityReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.patch = patch.object(trials, 'ROOT', self.root)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def write(self, path, value=None):
        path = self.root / path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value) if value is not None else 'fixture')
        return path

    def review(self, *, resolved=False):
        report = 'docs/extensions/AVOIDANCE_REVIEW_20261022.md'
        self.write(report)
        value = {'schema_version': 1, 'reviewed_utc': '2026-10-22T00:00:00+00:00',
                 'source_head': 'a' * 40, 'report': report,
                 'status': 'resolved' if resolved else 'requires_revision',
                 'withheld_categories': [] if resolved else [
                     {'group': group, 'variant': variant, 'reason': 'Physical necessity or control causality needs revision'}
                     for group, variant in [('replicator', 'human'), ('controls', 'human'),
                                            ('controls', 'property'), ('controls', 'robot')]]}
        if resolved:
            value['resolution_evidence'] = ['docs/extensions/review_resolution.json']
            self.write(value['resolution_evidence'][0], {'reviewed': True})
        self.write(trials.QUALITY_REVIEW, value)
        return value

    def episode(self, group, variant, attempt='attempt_01', trial_root='avoidance_trials'):
        directory = Path('datasets') / trial_root / group / variant / attempt
        result = self.write(directory / 'result.json', {'variant': variant, 'accepted': True})
        self.write(directory / 'metadata.json', {'variant': variant})
        base = Path('datasets') / trial_root / group / 'base' / 'baseline'
        baseline_records = self.write(base / 'records.json', [])
        actual_records = self.write(directory / 'records.json', [])
        causal = {'accepted': True, 'same_control_parameters': True, 'gates': {'fixture': True},
                  'baseline_episode': str(base),
                  'baseline_sha256': hashlib.sha256(baseline_records.read_bytes()).hexdigest(),
                  'episode_sha256': hashlib.sha256(actual_records.read_bytes()).hexdigest()}
        self.write(directory / 'avoidance_audit.json', causal)
        for name in ['fixed_base_hazard_audit.json', 'background_consistency_audit.json',
                     'source_background_audit.json', 'independent_validation.json',
                     'self_collision_detailed_audit.json', 'self_collision_audit.json',
                     'validation.json', 'hazard_clearance.json', 'source_relation_audit.json',
                     'overview_validation.json']:
            self.write(directory / name, {'passed': True})
        self.write(directory / 'validation_full.json', {'accepted': True})
        self.write(directory / 'strict_clearance_audit.json', {'executed_augmented_trajectory': {'passed': True}})
        for name in ['full_world_audit.json', 'contacts.json', 'clearances.json']:
            self.write(directory / name, {})
        for name in (*trials.CAMERAS, 'comparison', 'overview_camera'):
            self.write(directory / (name + '.mp4'))
        self.write(directory / 'comparison_validation.json',
                   {'frames': 1, 'video': 1, 'baseline_video': str(base / 'external_camera_2.mp4')})
        for name in ['trajectory.h5', 'trajectory.npz', 'images.h5', 'overview/images.h5']:
            self.write(directory / name)
        for name in ['overview/cameras.json', 'overview/frame_alignment.json']:
            self.write(directory / name, {})
        runner = Path('reports') / trial_root / 'replicator' / variant / attempt
        self.write(runner / 'summary.json', {'passed': True, 'exit_codes': dict.fromkeys(
            ['collection', 'export', 'validation', 'audit'], 0)})
        for name in ['validation.json', 'audit.json', 'videos.json']:
            self.write(runner / name, {'passed': True})
        return result

    def test_missing_review_fails_closed_before_attempt_scan(self):
        with self.assertRaisesRegex(trials.QualityReviewError, 'missing or unreadable'):
            trials.summarize('avoidance_trials')
        file = self.episode('doors', 'robot')
        with self.assertRaises(trials.QualityReviewError):
            trials.collect_attempt('doors', file, 'avoidance_trials')
        with self.assertRaises(trials.QualityReviewError):
            trials.export({'completed': 12, 'trial_root': 'avoidance_trials'})
        self.assertFalse((self.root / 'exports').exists())

    def test_malformed_reviews_fail_closed(self):
        self.review()
        (self.root / trials.QUALITY_REVIEW).write_text('{bad JSON')
        with self.assertRaises(trials.QualityReviewError):
            trials.load_quality_review()
        cases = [[], {'schema_version': True}, {'status': 'approved'},
                 {'withheld_categories': []}, {'withheld_categories': [{'group': [], 'variant': 'human', 'reason': 'x'}]},
                 {'report': '../outside.md'}, {'reviewed_utc': '2026-10-22T00:00:00'}]
        for update in cases:
            with self.subTest(update=update):
                value = self.review()
                if isinstance(update, dict):
                    value.update(update)
                else:
                    value = update
                self.write(trials.QUALITY_REVIEW, value)
                with self.assertRaises(trials.QualityReviewError):
                    trials.load_quality_review()

    def test_four_categories_withheld_even_when_previous_gates_pass(self):
        review = self.review()
        for item in review['withheld_categories']:
            with self.subTest(item=item):
                file = self.episode(item['group'], item['variant'])
                row = trials.collect_attempt(item['group'], file, 'avoidance_trials')
                self.assertTrue(row['collector_accepted'])
                self.assertTrue(all(v for k, v in row['evidence'].items() if k != 'quality_review'))
                self.assertFalse(row['evidence']['quality_review'])
                self.assertFalse(row['passed'])
                self.assertEqual(row['evidence_files']['avoidance_quality_review.json'], str(trials.QUALITY_REVIEW))

    def test_no_fallback_to_older_or_renamed_accepted_attempt(self):
        self.review()
        self.episode('controls', 'human', 'old_success')
        self.episode('controls', 'human', 'new_name_cannot_unblock')
        self.episode('doors', 'robot')
        report = trials.summarize('avoidance_trials')
        blocked = next(r for r in report['episodes'] if (r['group'], r['variant']) == ('controls', 'human'))
        self.assertFalse(blocked['passed'])
        self.assertNotIn('episode', blocked)
        self.assertEqual(report['completed'], 1)
        self.assertEqual(len([r for r in report['attempts'] if r['group'] == 'controls']), 2)
        self.assertFalse(report['strengthened_avoidance_acceptance_established'])
        self.assertIn('previous', report['scope'])

    def test_export_rereads_current_blockers_and_preserves_existing_exports(self):
        self.review()
        sentinel = self.write('exports/avoidance-trial-videos/index.html')
        before = sentinel.read_bytes()
        # Simulate an old 12/12 summary, including a forged resolved snapshot.
        old = {'completed': 12, 'trial_root': 'avoidance_trials',
               'quality_review': {'status': 'resolved', 'withheld_categories': []}}
        with self.assertRaisesRegex(SystemExit, 'unresolved quality review blockers'):
            trials.export(old)
        self.assertEqual(sentinel.read_bytes(), before)
        self.assertEqual(list((self.root / 'exports').rglob('*')), [sentinel.parent, sentinel])

    def test_resolution_requires_evidence_and_export_rejects_stale_snapshot(self):
        value = self.review(resolved=True)
        value.pop('resolution_evidence')
        self.write(trials.QUALITY_REVIEW, value)
        with self.assertRaisesRegex(trials.QualityReviewError, 'resolution_evidence'):
            trials.load_quality_review()
        self.review(resolved=True)
        current = trials.load_quality_review()
        file = self.episode('controls', 'human')
        self.assertTrue(trials.collect_attempt('controls', file, 'avoidance_trials')['passed'])
        with self.assertRaisesRegex(SystemExit, 'stale or missing quality review'):
            trials.export({'completed': 12, 'trial_root': 'avoidance_trials', 'quality_review': {}})
        with self.assertRaisesRegex(SystemExit, 'fewer than 12'):
            trials.export({'completed': 11, 'trial_root': 'avoidance_trials', 'quality_review': current})

    def test_skill_trials_do_not_require_or_apply_avoidance_review(self):
        self.episode('controls', 'human', trial_root='skill_trials')
        report = trials.summarize('skill_trials')
        self.assertEqual(report['completed'], 1)
        self.assertNotIn('quality_review', report)
        self.assertNotIn('quality_review', report['attempts'][0]['evidence'])
        self.assertNotIn('avoidance_quality_review.json', report['attempts'][0]['evidence_files'])
        self.write(trials.QUALITY_REVIEW, ['malformed'])
        self.assertEqual(trials.summarize('skill_trials')['completed'], 1)


if __name__ == '__main__':
    unittest.main()
