"""Prevent absent startup/main steps and changed execution inputs from passing."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from kitchen_safety.extensions.replicator.collection_evidence import (
    CollectionProvenance, MotionPhysicsIndex, audit_motion_alignment, sha256,
)


def streams():
    prefix = np.arange(100, 105)
    index = dict(physics_step=np.arange(105, 109), phase=np.asarray(['move'] * 4),
                 action_index=np.asarray([1, 1, 2, 2]))
    motion = dict(q=np.zeros((4, 13)), mustard=np.zeros((4, 7)),
                  phase=index['phase'].copy(), action_index=index['action_index'].copy(),
                  physics_check=np.arange(1, 5))
    return prefix, index, motion


class CollectionEvidenceTest(unittest.TestCase):
    def test_exact_prefix_to_main_continuity(self):
        prefix, index, motion = streams()
        self.assertTrue(audit_motion_alignment(prefix, index, motion, world_ready_step=104, final_step=108)['passed'])
        for invalid in ([104, 106, 107, 108], [106, 107, 108, 109], [105, 107, 107, 108]):
            index['physics_step'] = np.asarray(invalid)
            self.assertFalse(audit_motion_alignment(prefix, index, motion, world_ready_step=104, final_step=108)['passed'])

    def test_stream_row_labels_tail_and_presence_are_required(self):
        prefix, index, motion = streams()
        self.assertFalse(audit_motion_alignment(prefix, index, motion, world_ready_step=104, final_step=109)['passed'])
        self.assertFalse(audit_motion_alignment(prefix, index, None, world_ready_step=104, final_step=108)['passed'])
        motion['action_index'][2] = 1
        self.assertFalse(audit_motion_alignment(prefix, index, motion, world_ready_step=104, final_step=108)['passed'])
        prefix, index, motion = streams()
        motion['q'] = motion['q'][:-1]
        self.assertFalse(audit_motion_alignment(prefix, index, motion, world_ready_step=104, final_step=108)['passed'])

    def test_empty_main_cannot_claim_a_collected_task(self):
        prefix, _, _ = streams()
        index = dict(physics_step=np.asarray([], dtype=int), phase=np.asarray([], dtype=str),
                     action_index=np.asarray([], dtype=int))
        self.assertFalse(audit_motion_alignment(prefix, index, None, world_ready_step=104, final_step=104)['passed'])

    def test_evidence_binds_real_files_and_refuses_overwrite(self):
        prefix, index, motion = streams()
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            np.savez(output / 'startup_physics_states.npz', physics_step=prefix)
            np.savez(output / 'physics_states.npz', **motion)
            writer = MotionPhysicsIndex()
            for row in zip(index['physics_step'], index['phase'], index['action_index']):
                writer.append(*row)
            report = writer.finish(output, {'world_ready_physics_step': 104}, 108)
            self.assertTrue(report['passed'])
            self.assertEqual(report['input_sha256']['physics_states.npz'], sha256(output / 'physics_states.npz'))
            with self.assertRaises(FileExistsError):
                writer.finish(output, {'world_ready_physics_step': 104}, 108)

    def test_initial_source_not_replaced_by_changed_or_deleted_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'helper.py'
            source.write_text('original = 1\n')
            capture = CollectionProvenance(root, ['helper.py'])
            initial = sha256(source)
            self.assertTrue(capture.check()['passed'])
            source.write_text('changed = 2\n')
            report = capture.check()
            self.assertFalse(report['passed'])
            self.assertEqual(report['entries']['helper.py']['initial_sha256'], initial)
            self.assertEqual(report['entries']['helper.py']['source_text'], 'original = 1\n')
            source.unlink()
            self.assertFalse(capture.check()['passed'])

    def test_declared_source_recipe_or_overlay_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'helper.py'
            source.write_text('original = 1\n')
            capture = CollectionProvenance(root, ['helper.py'])
            capture.capture('helper.py', 'recipe', expected_sha256='0' * 64)
            self.assertFalse(capture.check()['passed'])


if __name__ == '__main__':
    unittest.main()
