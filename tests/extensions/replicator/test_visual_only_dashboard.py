"""New image candidates stay source-bound and separate from older safety results."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.extensions.replicator.progress_dashboard import AnnotationError, Progress, allowed_artifact


class VisualOnlyDashboardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.task = "kitchen_bench_replicator_g_shape_banana_bowl"
        self.scene = "g_shape_banana_bowl__human_default_mpfb"
        self.variant = "human_default_mpfb"
        self.write("student_skills/robolaw-replicator/references/assignment.json", {"tasks": [{"id": self.task}]})
        self.old_config = self.write("configs/extensions/replicator/visual_scene_variants/old.json", {"task_id": self.task})
        old_image = self.binary("reports/replicator_safety_upgrade/visual_placement/old.png", b"old PNG")
        self.old_row = {"scene_id": self.scene, "config_path": self.rel(self.old_config), "config_sha256": self.sha(self.old_config), "status": "retained"}
        self.write("reports/replicator_safety_upgrade/visual_placement/progress.json", {"entries": [self.old_row]})
        self.write("reports/replicator_safety_upgrade/visual_placement/render_old_progress.json", {"entries": [
            {"scene_id": self.scene, "config_sha256": self.sha(self.old_config), "views": [
                {"view_id": "overview_00", "kind": "overview", "preview_path": self.rel(old_image), "preview_sha256": self.sha(old_image)}]}]})
        self.progress = Progress(self.root)
        self.progress.processes = lambda tasks: []
        self.tasks = {self.task: {}}
        self.config = self.write("configs/extensions/replicator/visual_only_review_v1/scene/human.json",
                                 {"task_id": self.task, "obstacle": {"position": [1, 2, 3]}})
        self.folder = "reports/replicator_visual_audit_20261025/render/" + self.scene
        self.image = self.binary(self.folder + "/overview_00.png", b"new actual image")
        self.report_data = {"scene_id": self.scene, "task_id": self.task, "variant_id": self.variant,
                            "config_path": self.rel(self.config), "config_sha256": self.sha(self.config),
                            "accepted": True, "readability": {"readability_passed": True},
                            "views": [{"view_id": "overview_00", "kind": "overview", "preview_path": self.rel(self.image),
                                       "preview_sha256": self.sha(self.image), "visibility": {"risk_verified": True}}]}
        self.report = self.write(self.folder + "/multi_view.json", self.report_data)
        self.overlay = {"scene_id": self.scene, "config_path": self.rel(self.config), "config_sha256": self.sha(self.config),
                        "render_report_path": self.rel(self.report), "render_report_sha256": self.sha(self.report)}
        self.publish()

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, rel, value):
        return self.binary(rel, json.dumps(value).encode())

    def binary(self, rel, value):
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value)
        return path

    def rel(self, path):
        return path.relative_to(self.root).as_posix()

    def sha(self, path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def publish(self, rows=None):
        self.write("reports/replicator_visual_audit_20261025/current_scenes.json", {"entries": rows or [self.overlay]})

    def fresh(self):
        # No cache is needed to make source-binding assertions.
        return Progress(self.root).visual_only_review(self.tasks)

    def record_review(self, status="passed"):
        path = "reports/replicator_visual_audit_20261025/reviews/" + self.scene + ".json"
        record = {"tool": "record_visual_only_review.py", "human_visual_only": True,
                  "scene_id": self.scene, "task_id": self.task, "variant_id": self.variant,
                  "status": status, "config_sha256": self.sha(self.config),
                  "observation": "已实际看图", "action": "保留" if status == "passed" else "移入操作区",
                  "viewed_images": [{"view_id": "overview_00", "preview_path": self.rel(self.image), "preview_sha256": self.sha(self.image)}]}
        self.write(path, record)
        counts = {"reviewed": 1, "passed": int(status == "passed"), "adjust": int(status == "adjust"), "unreviewed": 10, "stale": 0}
        row = {"scene_id": self.scene, "status": status, "review_path": path,
               "config_sha256": self.sha(self.config), "binding_current": True}
        self.write("reports/replicator_visual_audit_20261025/progress.json", {"counts": counts, "entries": [row]})
        return path, record, row, counts

    def test_overlay_replaces_only_current_sources_not_old_stage_or_acceptance(self):
        base = copy.deepcopy(self.progress.visual_placement(self.tasks))
        result = self.progress.visual_only_review(self.tasks, base)
        self.assertEqual(result["total"], 11)
        self.assertEqual(result["scope"], "visual only")
        self.assertEqual(result["overlay_applied"], 1)
        self.assertEqual(result["overlay_errors"], [])
        row = result["entries"][0]
        self.assertEqual(row["config_sha256"], self.sha(self.config))
        self.assertEqual(row["views"][0]["preview_sha256"], self.sha(self.image))
        self.assertEqual(row["visual_review_status"], "unreviewed")
        self.assertEqual(result["counts"]["passed"], 0)
        for name in ("accepted", "semantic_geometry_passed", "readable_pixels", "risk_verified", "new_visual_review_status"):
            self.assertNotIn(name, row)
        self.assertNotIn("visibility", row["views"][0])
        self.assertEqual(self.progress.visual_placement(self.tasks), base)
        snapshot = self.progress.snapshot()
        self.assertEqual(snapshot["visual_only_review"]["entries"][0], row)
        self.assertEqual(snapshot["safety_upgrade"]["counts"]["accepted"], 0)
        self.assertTrue(allowed_artifact(self.rel(self.image)))
        self.assertTrue(allowed_artifact(self.rel(self.config)))

    def test_sources_and_png_hashes_must_all_match_or_old_views_remain(self):
        originals = {p: p.read_bytes() for p in (self.config, self.report, self.image)}
        for path in originals:
            with self.subTest(path=path):
                path.write_bytes(b"changed source")
                result = self.fresh()
                self.assertEqual(result["overlay_applied"], 0)
                self.assertTrue(result["overlay_errors"])
                self.assertEqual(result["entries"][0]["config_sha256"], self.old_row["config_sha256"])
                path.write_bytes(originals[path])

    def test_report_cannot_relabel_another_scene_or_config_even_with_new_report_hash(self):
        for change in ({"scene_id": "another__human"}, {"task_id": "other"}, {"variant_id": "property_vase"},
                       {"config_sha256": "a" * 64}, {"config_path": self.rel(self.old_config)}, {"views": []}):
            with self.subTest(change=change):
                self.write(self.rel(self.report), {**self.report_data, **change})
                self.overlay["render_report_sha256"] = self.sha(self.report)
                self.publish()
                self.assertEqual(self.fresh()["overlay_applied"], 0)

    def test_image_must_be_current_report_view_in_own_output_directory(self):
        foreign = self.binary("reports/replicator_visual_audit_20261025/render/other/overview.png", b"foreign image")
        data = copy.deepcopy(self.report_data)
        data["views"][0].update(preview_path=self.rel(foreign), preview_sha256=self.sha(foreign))
        self.write(self.rel(self.report), data)
        self.overlay["render_report_sha256"] = self.sha(self.report)
        self.publish()
        self.assertEqual(self.fresh()["overlay_applied"], 0)
        # A renderer's retry path is not a published current view.
        data = copy.deepcopy(self.report_data)
        data["views"][0]["renderer_settings"] = {"original_preview_path": self.rel(foreign), "original_preview_sha256": self.sha(foreign)}
        self.write(self.rel(self.report), data)
        self.overlay["render_report_sha256"] = self.sha(self.report)
        self.publish()
        row = self.fresh()["entries"][0]
        self.assertEqual(len(row["views"]), 1)
        self.assertEqual(row["views"][0]["preview_sha256"], self.sha(self.image))

    def test_paths_unknown_and_duplicate_scene_cannot_override(self):
        for rows in ([{**self.overlay, "config_path": "../../outside.json"}],
                     [{**self.overlay, "config_path": self.rel(self.old_config), "config_sha256": self.sha(self.old_config)}],
                     [{**self.overlay, "scene_id": "unknown"}], [self.overlay, self.overlay]):
            with self.subTest(rows=rows):
                self.publish(rows)
                self.assertEqual(self.fresh()["overlay_applied"], 0)

    def test_counts_come_only_from_this_round_and_new_annotations_use_overlay(self):
        _, _, _, counts = self.record_review("adjust")
        result = self.fresh()
        self.assertEqual(result["counts"], counts)
        self.assertEqual(result["entries"][0]["visual_review_status"], "adjust")
        payload = {"scene_id": self.scene, "status": "adjust", "note": "本轮建议", "expected_revision": 0,
                   "config_sha256": self.sha(self.config),
                   "views": [{"view_id": "overview_00", "preview_sha256": self.sha(self.image)}]}
        saved = self.progress.save_annotation(payload)
        self.assertEqual(saved["config_sha256"], self.sha(self.config))
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_annotation({**payload, "expected_revision": 1, "config_sha256": self.old_row["config_sha256"]})
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(caught.exception.entry, saved)

    def test_same_config_new_render_image_invalidates_prior_passed_counts(self):
        self.record_review("passed")
        self.assertEqual(self.fresh()["counts"]["passed"], 1)
        self.image.write_bytes(b"new camera view of same config")
        self.report_data["views"][0]["preview_sha256"] = self.sha(self.image)
        self.write(self.rel(self.report), self.report_data)
        self.overlay["render_report_sha256"] = self.sha(self.report)
        self.publish()
        result = self.fresh()
        self.assertEqual(result["overlay_applied"], 1)
        self.assertEqual(result["counts"], {"reviewed": 0, "passed": 0, "adjust": 0, "unreviewed": 11, "stale": 1})
        self.assertEqual(result["entries"][0]["visual_review_status"], "unreviewed")
        self.assertFalse(result["review_entries"][0]["binding_current"])
        self.assertEqual(result["review_entries"][0]["previous_status"], "passed")

    def test_changed_config_invalidates_pass_and_counts_are_not_copied_from_progress(self):
        self.record_review("passed")
        self.write(self.rel(self.config), {"task_id": self.task, "obstacle": {"position": [2, 3, 4]}})
        self.report_data["config_sha256"] = self.sha(self.config)
        self.write(self.rel(self.report), self.report_data)
        self.overlay.update(config_sha256=self.sha(self.config), render_report_sha256=self.sha(self.report))
        self.publish()
        result = self.fresh()
        self.assertEqual(result["counts"]["passed"], 0)
        self.assertEqual(result["counts"]["stale"], 1)
        self.write("reports/replicator_visual_audit_20261025/progress.json", {"counts": {"passed": 11, "reviewed": 11}, "entries": []})
        self.assertEqual(self.fresh()["counts"], {"reviewed": 0, "passed": 0, "adjust": 0, "unreviewed": 11, "stale": 0})

    def test_explicit_current_record_is_required_and_duplicate_or_foreign_record_never_counts(self):
        path, record, row, counts = self.record_review("passed")
        for update in ({"tool": "old_geometry_review.py"}, {"human_visual_only": False}, {"scene_id": "other"},
                       {"status": "accepted"}, {"viewed_images": []},
                       {"viewed_images": [{**record["viewed_images"][0], "view_id": "another_view"}]}):
            with self.subTest(update=update):
                self.write(path, {**record, **update})
                self.assertEqual(self.fresh()["counts"]["passed"], 0)
        self.write(path, record)
        self.write("reports/replicator_visual_audit_20261025/progress.json", {"counts": counts, "entries": [row, row]})
        self.assertEqual(self.fresh()["counts"]["passed"], 0)
        self.write("reports/replicator_visual_audit_20261025/progress.json", {"counts": counts, "entries": [{**row, "review_path": "../../outside.json"}]})
        self.assertEqual(self.fresh()["counts"]["passed"], 0)

    def test_current_record_matches_tool_counts_even_when_saved_progress_flags_are_stale(self):
        _, _, row, counts = self.record_review("adjust")
        self.write("reports/replicator_visual_audit_20261025/progress.json", {"counts": {}, "entries": [
            {**row, "status": "unreviewed", "previous_status": "adjust", "binding_current": False}]})
        result = self.fresh()
        self.assertEqual(result["counts"], counts)
        self.assertTrue(result["review_entries"][0]["binding_current"])
        self.assertEqual(result["review_entries"][0]["status"], "adjust")


if __name__ == "__main__":
    unittest.main()
