import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.banana_halfcenter_candidate import read_candidate, shift_source_xy
from kitchen_safety.extensions.replicator.task_collection_evidence import (
    BANANA_TASK, BOWL_TASK, BANANA_GRASP_CANDIDATE, TaskRunProvenance, ROOT,
    audit_grasp_candidate_binding)


class HalfcenterCandidateTests(unittest.TestCase):
    def test_observed_pose_changes_projection_without_changing_height_or_jaw(self):
        section, proxy = [-.03, 0., .002], [-.02, .012, -.001]
        base = np.eye(4)
        base[:3, 3] = [4., -2., 1.234]
        source = np.eye(4)
        a, selected = shift_source_xy(base, source, section, proxy)
        source[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
        b, _ = shift_source_xy(base, source, section, proxy)
        np.testing.assert_allclose(a[:2, 3] - base[:2, 3], [.005, .006], atol=1e-15)
        np.testing.assert_allclose(b[:2, 3] - base[:2, 3], [-.006, .005], atol=1e-15)
        np.testing.assert_array_equal(selected, [-.025, .006, .002])
        for value in (a, b):
            self.assertEqual(value[2, 3], base[2, 3])
            np.testing.assert_array_equal(value[:3, :3], base[:3, :3])
        np.testing.assert_array_equal(base[:3, 3], [4., -2., 1.234])

    def test_tilt_uses_actual_projection_but_never_lifts_to_escape_clearance(self):
        source, base = np.eye(4), np.eye(4)
        source[:3, :3] = Rotation.from_euler("y", 60, degrees=True).as_matrix()
        base[2, 3] = .94
        actual, selected = shift_source_xy(base, source, [0., 0., 0.], [.02, .01, .4])
        np.testing.assert_allclose(actual[:2, 3], [.005, .005], atol=1e-15)
        self.assertEqual(actual[2, 3], .94)
        self.assertEqual(selected[2], 0.)

    def test_bad_pose_or_source_geometry_rejected(self):
        for rotation in [np.diag([1., 1., -1.]), np.eye(3) * 1.01]:
            source = np.eye(4)
            source[:3, :3] = rotation
            with self.assertRaises(ValueError):
                shift_source_xy(np.eye(4), source, [0., 0., 0.], [0., 0., 0.])
        with self.assertRaises(ValueError):
            shift_source_xy(np.eye(4), np.eye(4), [float("nan"), 0., 0.], [0., 0., 0.])

    def test_selection_and_base_hash_are_bound(self):
        with tempfile.TemporaryDirectory() as tmp:
            base, path = Path(tmp) / "base.json", Path(tmp) / "candidate.json"
            base.write_text(json.dumps(dict(task_id="kitchen_bench_replicator_g_shape_banana_bowl", source_object_sha256={"banana": "original"})))
            profile = dict(schema_version=1, task_id="kitchen_bench_replicator_g_shape_banana_bowl", candidate_id="contactcenter1_jaw0", base_profile_sha256=hashlib.sha256(base.read_bytes()).hexdigest(), source_banana_sha256="original", geometric_centroid_fraction_xy=.5, additional_yaw_rad=0., additional_world_height_m=0., baseline_and_variants_share_candidate=True)
            path.write_text(json.dumps(profile))
            self.assertEqual(read_candidate(path, base)["candidate_id"], "contactcenter1_jaw0")
            for key, value in [("geometric_centroid_fraction_xy", 1.), ("additional_world_height_m", .001), ("additional_yaw_rad", .01), ("source_banana_sha256", "other"), ("base_profile_sha256", "other")]:
                bad = dict(profile, **{key: value})
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    read_candidate(path, base)

    def test_new_candidate_is_captured_before_use_without_rewriting_old_inventory(self):
        recorder = TaskRunProvenance(ROOT / "configs/extensions/replicator/g_shape_banana_bowl_source_layout.json")
        self.assertNotIn(BANANA_GRASP_CANDIDATE, recorder.entries)
        binding = recorder.bind_grasp_candidate(ROOT / "configs/extensions/replicator/g_shape_banana_bowl_halfcenter_candidate.json")
        self.assertIn(BANANA_GRASP_CANDIDATE, recorder.entries)
        self.assertEqual(recorder.entries[binding["profile_path"]]["kind"], "grasp_candidate_input")
        self.assertTrue(recorder.check()["passed"])
        recorder.scene_bound = True
        with self.assertRaises(ValueError):
            recorder.bind_grasp_candidate(ROOT / "configs/extensions/replicator/g_shape_banana_bowl_halfcenter_candidate.json")
        self.assertEqual(audit_grasp_candidate_binding({}, {"task_id": BANANA_TASK}, {}, []), {"required": False})
        self.assertEqual(audit_grasp_candidate_binding({}, {"task_id": BOWL_TASK}, {}, []), {"required": False})

    def test_runtime_candidate_missing_identity_or_changed_selection_rejected(self):
        from copy import deepcopy
        candidate = dict(read_candidate())
        text = json.dumps(candidate)
        h = hashlib.sha256(text.encode()).hexdigest()
        binding = dict(profile_path="candidate.json", profile_sha256=h, module_path=BANANA_GRASP_CANDIDATE, module_sha256="module")
        metadata = dict(task_id=BANANA_TASK, source_hashes={"banana": candidate["source_banana_sha256"]}, grasp_candidate_binding=binding)
        report = dict(grasp_candidate_binding=binding)
        entries = {"candidate.json": dict(kind="grasp_candidate_input", initial_sha256=h, source_text=text),
            BANANA_GRASP_CANDIDATE: dict(kind="execution_source", initial_sha256="module"),
            "scripts/extensions/replicator/task_collect.py": dict(kind="execution_source", source_text="provenance.bind_grasp_candidate(path)")}
        profiles = [dict(initial_sha256=candidate["base_profile_sha256"])]
        self.assertTrue(audit_grasp_candidate_binding(report, metadata, entries, profiles)["required"])
        for key in ("candidate.json", BANANA_GRASP_CANDIDATE):
            changed = deepcopy(entries)
            del changed[key]
            with self.assertRaises(ValueError):
                audit_grasp_candidate_binding(report, metadata, changed, profiles)
        with self.assertRaises(ValueError):
            audit_grasp_candidate_binding({}, {"task_id": BANANA_TASK}, {"scripts/extensions/replicator/task_collect.py": entries["scripts/extensions/replicator/task_collect.py"]}, profiles)
        for key, value in [("additional_world_height_m", .001), ("geometric_centroid_fraction_xy", 1.), ("source_banana_sha256", "other"), ("base_profile_sha256", "other")]:
            changed = deepcopy(entries)
            changed["candidate.json"]["source_text"] = json.dumps(dict(candidate, **{key: value}))
            with self.assertRaises(ValueError):
                audit_grasp_candidate_binding(report, metadata, changed, profiles)


if __name__ == "__main__":
    unittest.main()
