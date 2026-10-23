import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.extensions.replicator.review_mobility import review, VIDEO_NAMES


class MobilityDecodeReviewTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.probe = dict(dt=1/15, decoded_frames={Path(name).stem: 90 for name in VIDEO_NAMES})
        (self.root/'mobility_probe.json').write_text(json.dumps(self.probe))
        for name in ['recorded_states.json', 'wrench_calls.json', *VIDEO_NAMES]:
            p = self.root/name; p.parent.mkdir(exist_ok=True); p.write_text('synthetic fixture')

    def test_independent_full_decode_and_hashes(self):
        with patch('scripts.extensions.replicator.review_mobility.audit_mobility', return_value={'passed': True}), \
             patch('scripts.extensions.replicator.review_mobility.decode_video', return_value=90) as decode:
            r = review(self.root)
        self.assertTrue(r['passed']); self.assertEqual(decode.call_count, 5)
        self.assertEqual(set(r['videos']), set(VIDEO_NAMES))
        for call in decode.call_args_list:
            self.assertEqual(call.args[1], 90); self.assertEqual(call.kwargs['expected_dt'], 1/15)

    def test_report_green_cannot_bypass_raw_failure(self):
        with patch('scripts.extensions.replicator.review_mobility.audit_mobility', return_value={'passed': False}), \
             patch('scripts.extensions.replicator.review_mobility.decode_video') as decode:
            with self.assertRaises(ValueError): review(self.root)
        decode.assert_not_called()

    def test_decode_failure_or_mid_decode_mutation_fails(self):
        for mutate in [False, True]:
            def decode(path, *_args, **_kwargs):
                if mutate:
                    path.write_text('changed while decoding'); return 90
                raise RuntimeError('truncated original stream')
            with self.subTest(mutate=mutate), \
                 patch('scripts.extensions.replicator.review_mobility.audit_mobility', return_value={'passed': True}), \
                 patch('scripts.extensions.replicator.review_mobility.decode_video', side_effect=decode):
                with self.assertRaises((ValueError, RuntimeError)): review(self.root)


if __name__ == '__main__':
    unittest.main()
