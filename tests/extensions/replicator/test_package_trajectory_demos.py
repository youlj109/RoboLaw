"""Actual CPU video decoding and physical-result provenance packaging guards."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
from unittest.mock import patch

import cv2
import numpy as np

from scripts.extensions.replicator import package_trajectory_demos as package


class PhysicalDemoPackagingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        root_patch = patch.object(package, 'ROOT', self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)
        self.source = self.root / 'reports/run10'
        self.demos = self.root / 'exports/physical'
        self.sid = 'test_task__test_variant'
        self.row = dict(scene_id=self.sid, task_id='test_task', variant_id='test_variant',
                        status='generated', variant_name='测试变体')
        config = self.write('configs/layout.json', {'robot': [0, 0, 0]})
        self.row.update(config_path='configs/layout.json', config_sha256=package.sha(config))
        trajectory = self.root / 'reports/trajectory.npz'
        trajectory.parent.mkdir(parents=True, exist_ok=True)
        np.savez(trajectory, q=np.zeros((6, 7)), tcp_world=np.repeat(np.eye(4)[None], 6, axis=0),
                 gripper=np.zeros(6), phase=np.array(package.PHASES), dt=np.array(.1))
        result_path = self.source / 'scenes' / self.sid / 'result.json'
        self.result = dict(self.row, stage='complete', trajectory_complete=True, planning_only=True,
            executed=False, task_success_verified=False, trajectory_samples=6,
            phases=[dict(phase=name, status='generated', solver_success=True) for name in package.PHASES],
            artifacts={'trajectory': dict(path='reports/trajectory.npz', sha256=package.sha(trajectory))})
        self.write(result_path, self.result)
        video_path = self.demos / 'videos' / (self.sid + '.mp4')
        video_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*'mp4v'), 10., (32, 24))
        self.assertTrue(writer.isOpened(), 'CPU MP4 encoder is required for meaningful decode tests')
        for index in range(6):
            frame = np.zeros((24, 32, 3), dtype=np.uint8)
            frame[:, index:index + 5, :] = 220
            writer.write(frame)
        writer.release()
        self.entry = dict(self.row, result_path=str(result_path.relative_to(self.root)),
            result_sha256=package.sha(result_path), trajectory_path='reports/trajectory.npz',
            trajectory_sha256=package.sha(trajectory), execution_result_path='executions/' + self.sid + '/result.json',
            planning_status='generated', execution_status='failed', task_success=False, attempts=1, executed=True,
            video_path='videos/' + self.sid + '.mp4', video_sha256=package.sha(video_path),
            fps=10., frame_count=6, width=32, height=24, duration_seconds=.6)
        self.physical = dict(self.entry, kind='physical_execution',
            planning_result_path=self.entry['result_path'], planning_result_sha256=self.entry['result_sha256'],
            failure_reason='Synthetic test: object missed the bowl')
        self.bind_physical()

    def write(self, path, value):
        path = self.root / path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))
        return path

    def bind_physical(self):
        path = self.write(self.demos / self.entry['execution_result_path'], self.physical)
        self.entry['execution_result_sha256'] = package.sha(path)

    def validate(self):
        return package.validate_entry(self.entry, self.row, self.source, self.demos)

    def test_failed_execution_keeps_real_video_and_source_bindings(self):
        checked = self.validate()
        self.assertEqual(checked['execution_status'], 'failed')
        self.assertFalse(checked['task_success'])
        self.assertTrue(checked['executed'])
        self.assertEqual(checked['video']['decoded_frames'], 6)
        self.assertEqual(checked['trajectory']['phases'], list(package.PHASES))

    def test_initialization_error_is_explicit_and_cannot_be_promoted_to_success(self):
        self.entry.update(execution_status='error', executed=False)
        self.physical.update(execution_status='error', executed=False)
        self.bind_physical()
        self.assertFalse(self.validate()['executed'])
        self.entry.update(execution_status='success', task_success=True)
        self.physical.update(execution_status='success', task_success=True)
        self.bind_physical()
        with self.assertRaisesRegex(ValueError, 'did not execute'):
            self.validate()

    def test_real_decoded_frame_count_must_match_both_records(self):
        self.entry['frame_count'] = self.physical['frame_count'] = 7
        self.bind_physical()
        with self.assertRaisesRegex(ValueError, 'Decoded frame count mismatch'):
            self.validate()

    def test_changed_source_npz_and_second_attempt_are_rejected(self):
        self.entry['attempts'] = self.physical['attempts'] = 2
        self.bind_physical()
        with self.assertRaisesRegex(ValueError, 'exactly one physical attempt'):
            self.validate()
        self.entry['attempts'] = self.physical['attempts'] = 1
        self.bind_physical()
        (self.root / self.entry['trajectory_path']).write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
            self.validate()

    def test_missing_real_video_is_rejected_even_for_initialization_error(self):
        self.entry.update(execution_status='error', executed=False)
        self.physical.update(execution_status='error', executed=False)
        self.bind_physical()
        (self.demos / self.entry['video_path']).unlink()
        with self.assertRaisesRegex(ValueError, 'missing/empty file'):
            self.validate()

    def test_incomplete_collection_does_not_create_archive_or_gallery(self):
        self.write(self.source / 'progress.json', dict(stage='complete', entries=[self.row]))
        self.write(self.demos / 'manifest.json', dict(schema_version=1, kind='physical_execution',
            source_run='reports/run10', source_progress_sha256=package.sha(self.source / 'progress.json'),
            entries=[self.entry]))
        output = self.root / 'demos.zip'
        with self.assertRaisesRegex(ValueError, 'exactly 154'):
            package.package(self.demos, self.source, output)
        self.assertFalse(output.exists())
        self.assertFalse((self.demos / 'index.html').exists())

    def test_gallery_labels_physical_failure_and_preserves_warning_text(self):
        checked = self.validate()
        checked['historical_quality_issues'] = ['测试历史质量提示']
        page = package.index_html([checked])
        self.assertIn('物理执行结果', page)
        self.assertIn('任务失败', page)
        self.assertIn('测试历史质量提示', page)
        self.assertIn('每景一次实际物理仿真尝试', page)

    def test_complete_small_fixture_zip_contains_real_video_result_and_valid_checksums(self):
        self.write(self.source / 'progress.json', dict(stage='complete', entries=[self.row]))
        self.write(self.demos / 'manifest.json', dict(schema_version=1, kind='physical_execution',
            source_run='reports/run10', source_progress_sha256=package.sha(self.source / 'progress.json'),
            entries=[self.entry]))
        output = self.root / 'demos.zip'
        # Lower only the collection size in this isolated fixture; real CLI requires 154.
        with patch.object(package, 'TOTAL', 1), patch.object(package, 'KNOWN_WARNINGS', set()):
            package.package(self.demos, self.source, output, workers=1)
        with zipfile.ZipFile(output) as archive:
            names = set(archive.namelist())
            self.assertEqual(names, {'manifest.json', 'index.html', 'README.md', 'validation.json',
                                    'SHA256SUMS.txt', self.entry['video_path'], 'results/' + self.sid + '.json'})
            lines = archive.read('SHA256SUMS.txt').decode().splitlines()
            self.assertEqual(len(lines), len(names) - 1)
            for line in lines:
                digest, name = line.split('  ', 1)
                self.assertEqual(hashlib.sha256(archive.read(name)).hexdigest(), digest)
            report = json.loads(archive.read('validation.json'))
            self.assertEqual(report['execution_counts'], {'failed': 1})
            self.assertTrue(report['entries'][0]['video']['fully_decoded'])


if __name__ == '__main__':
    unittest.main()
