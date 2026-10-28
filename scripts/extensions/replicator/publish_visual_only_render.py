#!/usr/bin/env python3
"""Publish an existing render's source bindings, without making a visual judgment."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[3]
CONFIGS = Path("configs/extensions/replicator/visual_only_review_v1")
REPORTS = Path("reports/replicator_visual_audit_20261025")
RENDERS = REPORTS / "renders"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def checked_file(root: Path, value: str, directory: Path, suffix: str) -> Path:
    if not isinstance(value, str):
        raise ValueError("缺少文件路径")
    path = Path(value)
    if ".." in path.parts:
        raise ValueError(f"不接受上级目录路径：{value}")
    path = (root / path).resolve()
    if not path.is_relative_to((root / directory).resolve()) or path.suffix != suffix or not path.is_file():
        raise ValueError(f"文件不在指定目录或扩展名不符：{value}")
    return path


def object_json(data: bytes) -> dict:
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("JSON 顶层必须是对象")
    return value


def entry_from_report(root: Path, report_value: str) -> tuple[dict, int]:
    report_path = checked_file(root, report_value, RENDERS, ".json")
    if report_path.name != "multi_view.json":
        raise ValueError("--report 必须指向实际 multi_view.json")
    report_bytes = report_path.read_bytes()
    report = object_json(report_bytes)
    scene_id, task_id, variant_id = (report.get(k) for k in ("scene_id", "task_id", "variant_id"))
    if (not isinstance(scene_id, str) or not re.fullmatch(r"[a-z0-9_]{1,160}", scene_id)
            or not isinstance(task_id, str) or not task_id.startswith("kitchen_bench_replicator_")
            or not isinstance(variant_id, str)
            or scene_id != task_id.removeprefix("kitchen_bench_replicator_") + "__" + variant_id):
        raise ValueError("报告中的场景、任务或变体标识不一致")
    config_path = checked_file(root, report.get("config_path"), CONFIGS, ".json")
    config_bytes = config_path.read_bytes()
    config = object_json(config_bytes)
    if sha256(config_bytes) != report.get("config_sha256"):
        raise ValueError("当前配置 SHA 与渲染报告不一致")
    if config.get("task_id") != task_id or config.get("obstacle", {}).get("id") != variant_id:
        raise ValueError("报告与配置的任务或变体标识不一致")
    views = report.get("views")
    if not isinstance(views, list) or not views:
        raise ValueError("报告没有实际视图")
    seen_ids, seen_paths = set(), set()
    for view in views:
        if not isinstance(view, dict):
            raise ValueError("视图格式异常")
        view_id = view.get("view_id")
        if not isinstance(view_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}", view_id) or view_id in seen_ids:
            raise ValueError("视图标识为空、无效或重复")
        image = checked_file(root, view.get("preview_path"), RENDERS, ".png")
        if image.parent != report_path.parent or image in seen_paths:
            raise ValueError("PNG 不在此报告目录，或视图重复引用同一 PNG")
        image_bytes = image.read_bytes()
        if not image_bytes.startswith(b"\x89PNG\r\n\x1a\n") or sha256(image_bytes) != view.get("preview_sha256"):
            raise ValueError(f"PNG 文件或 SHA 与报告不一致：{image.relative_to(root)}")
        seen_ids.add(view_id)
        seen_paths.add(image)
    return {"scene_id": scene_id, "config_path": config_path.relative_to(root).as_posix(),
            "config_sha256": sha256(config_bytes), "render_report_path": report_path.relative_to(root).as_posix(),
            "render_report_sha256": sha256(report_bytes)}, len(views)


def publish(report_value: str, root: Path = ROOT) -> dict:
    root = root.resolve()
    output = root / REPORTS / "current_scenes.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.with_suffix(".lock").open("a+b") as lockfile:
        fcntl.flock(lockfile.fileno(), fcntl.LOCK_EX)
        entry, count = entry_from_report(root, report_value)
        document = object_json(output.read_bytes()) if output.exists() else {"entries": []}
        entries = document.get("entries")
        if (not isinstance(entries, list)
                or any(not isinstance(row, dict) or not isinstance(row.get("scene_id"), str) for row in entries)
                or len({row["scene_id"] for row in entries}) != len(entries)):
            raise ValueError("现有 current_scenes.json 格式异常；停止以保护原记录")
        previous = next((row for row in entries if row["scene_id"] == entry["scene_id"]), None)
        changed = previous != entry
        if changed:
            document["entries"] = [entry if row["scene_id"] == entry["scene_id"] else row for row in entries]
            if previous is None:
                document["entries"].append(entry)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output.parent,
                                                 prefix=".current-scenes-", delete=False) as stream:
                    temporary = Path(stream.name)
                    json.dump(document, stream, ensure_ascii=False, indent=2)
                    stream.write("\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, output)
                directory = os.open(output.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
    return {"scene_id": entry["scene_id"], "published": changed, "views": count,
            "manifest": output.relative_to(root).as_posix(),
            "scope": "只发布图片与配置来源绑定；没有写入视觉审核结论或修改标注。"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, help="本轮 renders 目录中实际 multi_view.json 的仓库相对路径")
    args = parser.parse_args()
    try:
        print(json.dumps(publish(args.report), ensure_ascii=False))
        return 0
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        print(f"未发布：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
