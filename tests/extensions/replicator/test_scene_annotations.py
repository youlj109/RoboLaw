"""Exercise persistent scene notes separately from evidence/acceptance reports."""
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from scripts.extensions.replicator.progress_dashboard import AnnotationError, Progress, handler_for


class SceneAnnotationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.task_id = "kitchen_bench_replicator_g_shape_banana_bowl"
        self.write("student_skills/robolaw-replicator/references/assignment.json", {"tasks": [{"id": self.task_id}]})
        self.rows = []
        self.views = []
        render_rows = []
        digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        for variant in ("human_default_mpfb", "property_tall_glass"):
            scene_id = "g_shape_banana_bowl__" + variant
            config = self.write(f"configs/extensions/replicator/visual_scene_variants/{scene_id}.json",
                                {"task_id": self.task_id, "obstacle": variant})
            evidence = self.write(f"reports/replicator_safety_upgrade/visual_placement/{scene_id}_geometry.json", {"passed": True})
            image = self.root / f"reports/replicator_safety_upgrade/visual_placement/{scene_id}.png"
            image.write_bytes(b"fixture image " + variant.encode())
            view = {"view_id": "overview_00", "preview_path": image.relative_to(self.root).as_posix(),
                    "preview_sha256": digest(image), "kind": "overview"}
            row = dict(scene_id=scene_id, task_id=self.task_id, variant_id=variant, status="adjusted",
                       config_path=config.relative_to(self.root).as_posix(), config_sha256=digest(config),
                       evidence_path=evidence.relative_to(self.root).as_posix(), evidence_sha256=digest(evidence),
                       semantic_geometry_passed=True)
            render = self.write(f"reports/replicator_safety_upgrade/visual_placement/{scene_id}_render.json",
                                {"config_sha256": row["config_sha256"], "views": [view]})
            render_rows.append({"scene_id": scene_id, "config_sha256": row["config_sha256"], "views": [view],
                                "evidence_path": render.relative_to(self.root).as_posix(), "evidence_sha256": digest(render)})
            self.rows.append(row)
            self.views.append({"view_id": view["view_id"], "preview_sha256": view["preview_sha256"]})
        self.write("reports/replicator_safety_upgrade/visual_placement/progress.json", {"entries": self.rows})
        self.write("reports/replicator_safety_upgrade/visual_placement/render_test_progress.json", {"entries": render_rows})
        self.progress = self.instance()
        self.server = None

    def tearDown(self):
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            self.thread.join()
        self.tmp.cleanup()

    def instance(self):
        result = Progress(self.root)
        result.processes = lambda tasks: []
        return result

    def write(self, rel, data):
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data), encoding="utf-8")
        return path

    def payload(self, index=0, **changes):
        result = {"scene_id": self.rows[index]["scene_id"], "status": "adjust", "note": "人手向前移 5 cm，保持抓取区域畅通。\n重新检查侧视图。",
                  "expected_revision": 0, "config_sha256": self.rows[index]["config_sha256"], "views": [self.views[index]]}
        result.update(changes)
        return result

    def request(self, payload=None, headers=None, method=None, raw=None):
        if not self.server:
            self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(self.progress))
            self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        data = raw if raw is not None else (json.dumps(payload, ensure_ascii=False).encode() if payload is not None else None)
        combined = {"Content-Type": "application/json", "Origin": self.url}
        combined.update(headers or {})
        req = urllib.request.Request(self.url + "/api/annotations", data=data, headers=combined, method=method)
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as response:
            return response.status, json.load(response)

    def test_notes_persist_over_new_progress_instance_without_changing_evidence(self):
        before = copy.deepcopy(self.progress.snapshot())
        evidence = {p: p.read_bytes() for p in self.root.rglob("*.json")}
        saved = self.progress.save_annotation(self.payload(status="keep"))
        self.assertEqual(saved["revision"], 1)
        self.assertEqual(saved["task_id"], self.task_id)
        self.assertEqual(self.instance().annotations(), {"schema_version": 1, "entries": [saved]})
        self.assertEqual(self.progress.snapshot(), before)
        self.assertEqual(before["safety_upgrade"]["counts"]["accepted"], 0)
        self.assertFalse(before["visual_placement"]["entries"][0]["accepted"])
        self.assertEqual(before["visual_placement"]["counts"]["reviewed"], 0)
        for path, contents in evidence.items():
            self.assertEqual(path.read_bytes(), contents)

    def test_revision_conflict_retains_existing_note_and_reports_current_entry(self):
        saved = self.progress.save_annotation(self.payload())
        before = self.progress.annotation_path.read_bytes()
        with self.assertRaises(AnnotationError) as caught:
            self.instance().save_annotation(self.payload(note="旧页面覆盖"))
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(caught.exception.entry, saved)
        self.assertEqual(self.progress.annotation_path.read_bytes(), before)
        updated = self.instance().save_annotation(self.payload(note="新方案", expected_revision=1))
        self.assertEqual(updated["revision"], 2)
        self.assertEqual(updated["note"], "新方案")

    def test_concurrent_distinct_saves_from_separate_instances_preserve_both(self):
        barrier = threading.Barrier(2)
        def save(index):
            progress = self.instance()
            barrier.wait()
            return progress.save_annotation(self.payload(index))
        with ThreadPoolExecutor(max_workers=2) as pool:
            saved = list(pool.map(save, (0, 1)))
        self.assertEqual({r["scene_id"] for r in self.instance().annotations()["entries"]},
                         {r["scene_id"] for r in saved})
        self.assertFalse(list(self.progress.annotation_path.parent.glob("*.tmp")))

    def test_invalid_unknown_path_hash_and_view_payloads_never_write(self):
        invalid = [self.payload(scene_id="../../etc/passwd"), self.payload(scene_id="unknown__human"),
                   self.payload(scene_id="g_shape_banana_bowl__robot_steel_clamp"),
                   self.payload(status="accepted"), self.payload(status=[]), self.payload(status={}),
                   self.payload(note="x" * 20001), self.payload(note=123),
                   self.payload(expected_revision=True), self.payload(expected_revision=-1),
                   self.payload(config_sha256="wrong"), self.payload(path="/tmp/notes.json"),
                   self.payload(views=[{"view_id": "../../outside", "preview_sha256": "a" * 64}]),
                   self.payload(views=[{**self.views[0], "preview_path": "/tmp/image.png"}]),
                   self.payload(views=[self.views[0], self.views[0]]), []]
        for value in invalid:
            with self.subTest(value=value):
                with self.assertRaises(AnnotationError) as caught:
                    self.progress.save_annotation(value)
                self.assertEqual(caught.exception.status, 400)
                self.assertFalse(self.progress.annotation_path.exists())

    def test_changed_or_foreign_view_conflicts_preserve_saved_note(self):
        saved = self.progress.save_annotation(self.payload())
        before = self.progress.annotation_path.read_bytes()
        for views in ([self.views[1]], [{"view_id": "unrendered", "preview_sha256": "a" * 64}],
                      [{"view_id": self.views[0]["view_id"], "preview_sha256": "b" * 64}]):
            with self.subTest(views=views):
                status, result = self.request(self.payload(views=views, expected_revision=1, note="不得覆盖"))
                self.assertEqual(status, 409)
                self.assertEqual(result["entry"], saved)
                self.assertEqual(self.progress.annotation_path.read_bytes(), before)

    def test_config_mismatch_conflicts_and_tampered_config_cannot_receive_notes(self):
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_annotation(self.payload(config_sha256="a" * 64))
        self.assertEqual(caught.exception.status, 409)
        self.assertIsNone(caught.exception.entry)
        self.write(self.rows[0]["config_path"], {"task_id": self.task_id, "tampered": True})
        with self.assertRaises(AnnotationError) as caught:
            self.progress.save_annotation(self.payload())
        self.assertEqual(caught.exception.status, 400)
        self.assertFalse(self.progress.annotation_path.exists())

    def test_corrupt_storage_is_never_silently_replaced(self):
        saved = self.progress.save_annotation(self.payload())
        path = self.progress.annotation_path
        invalid = [b"{unfinished", b"\xff", b'{"schema_version":1,"entries":{}}',
                   b'{"schema_version":2,"entries":[]}', b'{"schema_version":true,"entries":[]}',
                   b'{"schema_version":1,"entries":[{"scene_id":"fake"}]}']
        invalid.extend(json.dumps({"schema_version": 1, "entries": [{**saved, "status": status}]}).encode()
                       for status in ([], {}, None))
        for raw in invalid:
            with self.subTest(raw=raw):
                path.write_bytes(raw)
                for method in (self.progress.annotations, lambda: self.progress.save_annotation(self.payload())):
                    with self.assertRaises(AnnotationError) as caught:
                        method()
                    self.assertEqual(caught.exception.status, 503)
                    self.assertEqual(path.read_bytes(), raw)

    def test_http_save_get_conflict_and_foreign_origin_rejection(self):
        self.assertEqual(self.request()[1], {"schema_version": 1, "entries": []})
        for headers in ({"Origin": "https://evil.test"}, {"Origin": "null"},
                        {"Origin": "http://127.0.0.1:1"}, {"Sec-Fetch-Site": "cross-site"},
                        {"Sec-Fetch-Site": "same-site"}, {"Host": "evil.test", "Origin": "http://evil.test"}):
            with self.subTest(headers=headers):
                status, result = self.request(self.payload(), headers=headers)
                self.assertEqual(status, 403)
                self.assertIn("error", result)
        self.assertFalse(self.progress.annotation_path.exists())
        status, result = self.request(self.payload())
        self.assertEqual(status, 200)
        self.assertEqual(self.request()[1]["entries"], [result["entry"]])
        status, result = self.request(self.payload(note="conflict"))
        self.assertEqual(status, 409)
        self.assertEqual(result["entry"]["revision"], 1)

    def test_http_rejects_invalid_json_content_type_and_large_body(self):
        for status_value in ([], {}, None):
            status, _ = self.request(self.payload(status=status_value))
            self.assertEqual(status, 400)
        status, _ = self.request(self.payload(), headers={"Content-Type": "text/plain"})
        self.assertEqual(status, 415)
        status, _ = self.request(raw=b"{broken")
        self.assertEqual(status, 400)
        status, _ = self.request(raw=b"x" * (128 * 1024 + 1))
        self.assertEqual(status, 413)
        self.assertFalse(self.progress.annotation_path.exists())


if __name__ == "__main__":
    unittest.main()
