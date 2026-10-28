"""A failed final action may be diagnosed but cannot become a full episode."""
import copy
import unittest

import numpy as np

from scripts.extensions.replicator.audit_partial_task_episode import validate_partial


class PartialExtentTests(unittest.TestCase):
    def example(self):
        metadata = dict(reset_physics_step=0, physics_dt_s=1 / 120,
                        action_dt_s=8 / 120, joint_names=["joint"])
        data = dict(physics_step=np.arange(15), q=np.zeros((15, 1)),
                    action_index=np.array([-1] + [0] * 8 + [1] * 6),
                    phase=np.array(["reset"] + ["close"] * 8 + ["lift"] * 6))
        trajectory = dict(actions=np.zeros((2, 1)), phase=np.array(["close", "lift"]),
                          before_physics_step=np.array([0, 8]), after_physics_step=np.array([8, 14]))
        return metadata, data, trajectory

    def test_exact_partial_extent_never_certifies_episode(self):
        result = validate_partial(*self.example())
        self.assertEqual(result["actual_physics_samples"], 15)
        self.assertEqual(result["partial_action_recorded_substeps"], 6)
        self.assertEqual(result["unexecuted_substeps_not_inferred"], 2)
        self.assertFalse(result["complete_episode"])

    def test_missing_step_rejected(self):
        metadata, data, trajectory = self.example()
        data["physics_step"][3] += 1
        with self.assertRaisesRegex(ValueError, "continuous"):
            validate_partial(metadata, data, trajectory)

    def test_fabricated_complete_tail_rejected(self):
        metadata, data, trajectory = self.example()
        trajectory["after_physics_step"][-1] = 16
        with self.assertRaisesRegex(ValueError, "exactly one final partial"):
            validate_partial(metadata, data, trajectory)

    def test_incomplete_earlier_action_rejected(self):
        metadata, data, trajectory = self.example()
        trajectory["after_physics_step"][0] = 7
        with self.assertRaisesRegex(ValueError, "earlier action"):
            validate_partial(metadata, data, trajectory)

    def test_finite_and_phase_identity_required(self):
        metadata, data, trajectory = self.example()
        bad_data = copy.deepcopy(data)
        bad_data["q"][-1] = np.nan
        with self.assertRaisesRegex(ValueError, "Nonfinite"):
            validate_partial(metadata, bad_data, trajectory)
        data["phase"][-1] = "close"
        with self.assertRaisesRegex(ValueError, "phase/action"):
            validate_partial(metadata, data, trajectory)


if __name__ == "__main__":
    unittest.main()
