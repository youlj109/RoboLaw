"""Small CPU-only media fixtures verify provenance, timing and overwrite guards."""
import json
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np
from PIL import Image

from kitchen_safety.dataset import close_encoders, video_encoder
from scripts.extensions.replicator.render_comparison import frame_reader, prepare, render, sample_index
from scripts.extensions.replicator.review_episode import sha


class ComparisonVideoTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.base, self.episode = self.root / 'base', self.root / 'human'
        self.camera = 'external_camera_2'
        self.output = self.root / 'media' / 'comparison.mp4'
        self.make_recording(self.base, 'base', 2, [(240, 0, 0), (0, 0, 240)])
        self.make_recording(self.episode, 'human', 4, [(0, 220, 0)] * 4)
        self.audit = dict(accepted=True, gates={'physical': True}, baseline_episode=str(self.base),
                          same_source_layout=True, same_effective_controls=True,
                          baseline_sha256={key: sha(self.base / name) for key, name in
                              [('result', 'result.json'), ('trajectory', 'trajectory.h5'), ('physics_states', 'physics_states.npz')]})
        self.write_audit()

    def make_recording(self, directory, variant, count, colors):
        directory.mkdir()
        (directory / 'result.json').write_text(json.dumps(dict(success=True, arena_success=True,
            final_on_destination=True, task_id='test_source_task', variant=variant)))
        (directory / 'cameras.json').write_text(json.dumps({self.camera: dict(width=64, height=36,
            offset={'position': [0, 0, 0]}, intrinsics=[[10, 0, 32], [0, 10, 18], [0, 0, 1]])}))
        with h5py.File(directory / 'trajectory.h5', 'w') as h5:
            h5.attrs['dt'] = .5
            h5.create_dataset('action', data=np.zeros((count, 8)))
            h5.create_dataset('time', data=np.arange(count) * .5)
            h5.create_dataset('phase', data=['settle'] + ['above_bowl'] * (count-1), dtype=h5py.string_dtype())
        with h5py.File(directory / 'images.h5', 'w') as h5:
            h5.attrs['dt'], h5.attrs['frames'] = .5, count
            group = h5.create_group(self.camera)
            group.create_dataset('jpeg', data=np.zeros((count, 1), dtype=np.uint8))
            group.create_dataset('position_world', data=np.zeros((count, 3)))
            group.create_dataset('quaternion_world_ros_xyzw', data=np.tile([0, 0, 0, 1], (count, 1)))
        np.savez(directory / 'physics_states.npz', q=np.zeros((count, 1)))
        process = video_encoder(directory / (self.camera + '.mp4'), 64, 36, .5)
        for color in colors:
            process.stdin.write(Image.new('RGB', (64, 36), color).tobytes())
        close_encoders({'fixture': process})

    def write_audit(self):
        (self.episode / 'avoidance_audit.json').write_text(json.dumps(self.audit))

    def test_original_speed_render_decodes_and_holds_last_frame(self):
        report = render(self.base, self.episode, self.camera, self.output)
        self.assertTrue(report['passed'], report.get('error'))
        self.assertEqual(report['frames'], 4)
        self.assertEqual(report['dt'], .5)
        self.assertEqual(report['source_records'][0]['held_output_frames'], 2)
        self.assertIn('no resampling', report['timing'])
        self.assertIn('Pending', report['visual_review'])
        reader = frame_reader(self.output, (1280, 536))
        frames = [Image.frombytes('RGB', (1280, 536), frame) for frame in reader]
        reader.close()
        # The short blue baseline holds its actual last image on output frames 2,3.
        for frame in frames[1:]:
            red, green, blue = frame.getpixel((320, 220))
            self.assertGreater(blue, 200)
            self.assertLess(red + green, 30)
        self.assertTrue(self.output.with_suffix('.validation.json').is_file())

    def test_different_camera_calibration_is_rejected(self):
        path = self.episode / 'cameras.json'
        meta = json.loads(path.read_text())
        meta[self.camera]['offset']['position'][0] = .1
        path.write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError, 'calibration'):
            prepare(self.base, self.episode, self.camera)

    def test_wrong_or_modified_baseline_cannot_be_used(self):
        self.audit['baseline_episode'] = str(self.root / 'other')
        self.write_audit()
        with self.assertRaisesRegex(ValueError, 'Wrong baseline'):
            prepare(self.base, self.episode, self.camera)
        self.audit['baseline_episode'] = str(self.base)
        self.write_audit()
        (self.base / 'physics_states.npz').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            prepare(self.base, self.episode, self.camera)

    def test_noncontiguous_recording_and_corrupt_video_rejected(self):
        with h5py.File(self.episode / 'trajectory.h5', 'r+') as h5:
            h5['time'][2] = 1.01
        with self.assertRaisesRegex(ValueError, 'timeline'):
            prepare(self.base, self.episode, self.camera)
        with h5py.File(self.episode / 'trajectory.h5', 'r+') as h5:
            h5['time'][2] = 1.
        (self.episode / (self.camera + '.mp4')).write_bytes(b'not a video')
        with self.assertRaisesRegex(ValueError, 'decode failed'):
            prepare(self.base, self.episode, self.camera)

    def test_existing_video_or_report_is_never_overwritten(self):
        self.output.parent.mkdir()
        self.output.write_bytes(b'previous video')
        with self.assertRaises(FileExistsError):
            render(self.base, self.episode, self.camera, self.output)
        self.assertEqual(self.output.read_bytes(), b'previous video')
        self.output.unlink()
        sidecar = self.output.with_suffix('.validation.json')
        sidecar.write_text('previous report')
        with self.assertRaises(FileExistsError):
            render(self.base, self.episode, self.camera, self.output)
        self.assertEqual(sidecar.read_text(), 'previous report')

    def test_frame_mapping_does_not_phase_align_or_drop_samples(self):
        self.assertEqual([sample_index(i, 3) for i in range(5)],
                         [(0, False), (1, False), (2, False), (2, True), (2, True)])


if __name__ == '__main__':
    unittest.main()
