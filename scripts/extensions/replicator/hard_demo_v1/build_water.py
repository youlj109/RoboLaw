#!/usr/bin/env python3
"""Build only the new water mechanism demo, preserving all easy/hard originals."""
from pathlib import Path
import argparse
import json
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.hard_demo_v1.water import create_demo, sha


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="Replace only this demo's generated USD/JSON")
    args = parser.parse_args()
    config = ROOT / "configs/extensions/replicator/hard_v1/build02/robot/task.json"
    task = json.loads(config.read_text())
    selected = task["source"]
    checks = []
    for item in [selected["source_config"], selected["source_review"], selected["source_original_yaml"], *selected["reviewed_images"]]:
        path = ROOT / item["path"]
        actual = sha(path)
        assert actual == item["sha256"], "Reviewed source changed: " + str(path)
        checks.append({"path": item["path"], "sha256": actual, "matches": True})
    review = json.loads((ROOT / selected["source_review"]["path"]).read_text())
    assert review["status"] == "passed" and review["config_sha256"] == selected["source_config"]["sha256"]
    source = ROOT / task["scene"]
    source_sha = sha(source)
    assert source_sha == task["scene_sha256"], "Hard source changed since build02"
    layout = json.loads((ROOT / selected["source_config"]["path"]).read_text())
    out = ROOT / "assets/extensions/replicator/hard_demo_v1/water"
    out.mkdir(parents=True, exist_ok=True)
    for name in ("scene.usda", "source_scene.usda"):
        path = out / name
        if path.exists():
            if not args.force:
                raise FileExistsError(f"{path}; use --force to replace this demo's output")
            path.unlink()
    result = create_demo(source, layout, out / "scene.usda")
    assert sha(source) == source_sha
    result["provenance"] = {"source_scene": task["scene"], "source_scene_sha256": source_sha,
                            "source_task_config": str(config.relative_to(ROOT)), "source_task_config_sha256": sha(config),
                            "easy_scene_id": selected["source_scene_id"], "easy_review_status": review["status"],
                            "easy_review_scope": review["scope"], "current_reviewed_hash_checks": checks,
                            "hard_demo_visual_reviewed": False, "source_unchanged_after_build": True}
    write(ROOT / "configs/extensions/replicator/hard_demo_v1/water/timeline.json", result["specification"])
    write(ROOT / "configs/extensions/replicator/hard_demo_v1/water/easy_layout_snapshot.json", layout)
    write(ROOT / "configs/extensions/replicator/hard_demo_v1/water/demo.json", result)
    write(ROOT / "reports/replicator_hard_demo_v1/water/build.json", result)
    print(json.dumps({"scene": result["scene"], "checks": result["checks"], "camera": result["specification"]["camera"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
