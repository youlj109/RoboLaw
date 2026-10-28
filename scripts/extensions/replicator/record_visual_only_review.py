#!/usr/bin/env python3
"""Record an explicitly supplied visual judgment; never judge a scene automatically.

Run this only after actually viewing every image passed with --view. Hash and
scene-membership checks bind that judgment to its sources; they do not assess
geometry, pixels, physics, task success, or safety.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlsplit
from urllib.request import Request, urlopen
import uuid


ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / "reports/replicator_visual_audit_20261025"
TOOL = "record_visual_only_review.py"
PREFIX = "[逐景视觉审核]"
TOTAL = 154


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def relative_file(value: str) -> Path:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"必须使用仓库内相对路径：{value}")
    result = (ROOT / path).resolve()
    if not result.is_relative_to(ROOT):
        raise ValueError(f"路径越出仓库：{value}")
    return result


def relative(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def api_base(value: str) -> str:
    url = urlsplit(value)
    if (url.scheme != "http" or url.hostname not in ("localhost", "127.0.0.1", "::1")
            or url.username or url.password or url.query or url.fragment
            or url.path not in ("", "/")):
        raise argparse.ArgumentTypeError("--api 只允许本机 HTTP 地址，如 http://127.0.0.1:8765")
    try:
        url.port
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return value.rstrip("/")


def request_json(api: str, route: str, payload: dict | None = None) -> dict:
    body = None if payload is None else json.dumps(payload, ensure_ascii=False).encode("utf-8")
    headers = {"Accept": "application/json", "Cache-Control": "no-cache"}
    if body is not None:
        headers["Content-Type"] = "application/json"
    request = Request(api + route, data=body, headers=headers)
    with urlopen(request, timeout=15) as response:
        result = json.load(response)
    if not isinstance(result, dict):
        raise ValueError(f"{route} 未返回 JSON 对象")
    return result


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"记录格式异常，停止以保护文件：{path}")
    return value


def write_new(path: Path, data: dict) -> None:
    """Exclusive creation: an attempt or sync receipt is never overwritten."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def write_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".visual-review-", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


@contextmanager
def locked():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT / ".record.lock").open("a+b") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def view_sources(scene: dict) -> dict[str, dict]:
    sources = {}
    for view in scene.get("views", []):
        url = urlsplit(view.get("image", {}).get("url", ""))
        if url.scheme or url.netloc or not url.path.startswith("/artifact/"):
            continue
        path = relative_file(unquote(url.path.removeprefix("/artifact/")))
        sources[relative(path)] = view
    return sources


def selected_images(scene: dict, requested: list[str]) -> list[dict]:
    sources = view_sources(scene)
    selected = []
    seen = set()
    for value in requested:
        path = relative_file(value)
        name = relative(path)
        if path.suffix.lower() != ".png" or name not in sources:
            raise ValueError(f"选图不属于此场景的当前 PNG 视图：{value}")
        view = sources[name]
        if view.get("view_id") in seen:
            raise ValueError(f"重复视角：{view.get('view_id')}")
        actual = digest(path)
        if actual != view.get("preview_sha256"):
            raise ValueError(f"图片已变化，请重新打开并审核：{value}")
        selected.append({"view_id": view["view_id"], "kind": view.get("kind"),
                         "preview_path": name, "preview_sha256": actual})
        seen.add(view["view_id"])
    return selected


def annotation_note(record: dict) -> str:
    label = "视觉通过" if record["status"] == "passed" else "待调整"
    return (f"{PREFIX} {label}\n观察：{record['observation']}\n"
            f"调整方案：{record['action']}\n"
            "仅按所列图片判断静态场景；不代表物理、任务执行或安全验证。")


def sync_annotation(api: str, record: dict, previous: dict | None) -> dict:
    """Only replace an exact annotation previously saved by this tool."""
    result = {"recorded_at": now(), "scene_id": record["scene_id"], "status": "skipped"}
    try:
        annotations = request_json(api, "/api/annotations")
        entries = annotations.get("entries")
        if not isinstance(entries, list):
            raise ValueError("标注 API 格式异常")
        current = next((row for row in entries if row.get("scene_id") == record["scene_id"]), None)
        if current is not None:
            receipt = None
            if previous and previous.get("annotation_sync_path"):
                path = relative_file(previous["annotation_sync_path"])
                if path.is_file():
                    receipt = load_json(path)
            if (not current.get("note", "").startswith(PREFIX) or not receipt
                    or receipt.get("status") != "synced" or receipt.get("entry") != current):
                result["reason"] = "保留现有标注：无法确认其仍是本工具上次保存的原文和版本。"
                return result
        note = annotation_note(record)
        if len(note) > 20000:
            result["reason"] = "本地记录已保存；摘要超过网页标注字数限制，跳过同步。"
            return result
        payload = {"scene_id": record["scene_id"],
                   "status": "keep" if record["status"] == "passed" else "adjust",
                   "note": note, "expected_revision": current["revision"] if current else 0,
                   "config_sha256": record["config_sha256"],
                   "views": [{"view_id": view["view_id"], "preview_sha256": view["preview_sha256"]}
                             for view in record["viewed_images"]]}
        response = request_json(api, "/api/annotations", payload)
        entry = response.get("entry")
        if not isinstance(entry, dict) or any(entry.get(key) != payload[key]
                                              for key in ("scene_id", "status", "note", "config_sha256", "views")):
            raise ValueError("网页未确认收到相同标注，保留本地记录")
        result.update(status="synced", entry=entry)
    except HTTPError as exc:
        result["reason"] = ("网页版本或图片绑定冲突，安全跳过；未覆盖其他标注。" if exc.code == 409
                            else f"网页同步失败 HTTP {exc.code}；本地记录已保存。")
    except (OSError, ValueError, KeyError, TypeError, URLError) as exc:
        result["reason"] = f"网页同步跳过，本地记录已保存：{exc}"
    return result


def build_progress(scenes: list[dict], source_time: str | None) -> dict:
    entries = []
    counts = {"reviewed": 0, "passed": 0, "adjust": 0, "unreviewed": 0, "stale": 0}
    for scene in scenes:
        path = OUTPUT / "reviews" / f"{scene['scene_id']}.json"
        row = {"scene_id": scene["scene_id"], "status": "unreviewed"}
        if path.is_file():
            record = load_json(path)
            if record.get("tool") != TOOL or record.get("human_visual_only") is not True:
                raise ValueError(f"非本工具审核记录，停止统计以避免误计：{path}")
            current = view_sources(scene)
            bound = (record.get("config_sha256") == scene.get("config_sha256")
                     and bool(record.get("viewed_images"))
                     and all(current.get(v["preview_path"], {}).get("preview_sha256") == v["preview_sha256"]
                             for v in record["viewed_images"]))
            row.update(review_path=relative(path), reviewed_at=record["reviewed_at"],
                       observation=record["observation"], action=record["action"],
                       config_sha256=record["config_sha256"], binding_current=bound)
            if bound:
                if record["status"] not in ("passed", "adjust"):
                    raise ValueError(f"未知审核状态：{path}")
                row["status"] = record["status"]
                counts["reviewed"] += 1
                counts[record["status"]] += 1
            else:
                row["previous_status"] = record["status"]
                counts["stale"] += 1
        if row["status"] == "unreviewed":
            counts["unreviewed"] += 1
        entries.append(row)
    return {"schema_version": 1, "tool": TOOL, "human_visual_only": True,
            "updated_at": now(), "source_api_generated_at": source_time, "total": TOTAL,
            "stage": "visual_review_complete" if counts["passed"] == TOTAL else "visual_review_in_progress",
            "scope": "仅记录实际看图后手工指定的判断；来源核对不产生通过结论。变更后的旧记录不计入当前已审。",
            "counts": counts, "entries": entries}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, help="当前场景 scene_id")
    parser.add_argument("--status", required=True, choices=("passed", "adjust"), help="实际看图后明确给出的判断")
    parser.add_argument("--observation", required=True, help="图片中实际观察到的内容")
    parser.add_argument("--action", required=True, help="调整方案；通过时说明保留理由")
    parser.add_argument("--view", required=True, action="append", help="已实际看过的仓库相对 PNG 路径；可重复")
    parser.add_argument("--api", type=api_base, default="http://127.0.0.1:8765", help="本机进度网页地址")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-z0-9_]{1,160}", args.scene):
        parser.error("--scene 格式无效")
    if not args.observation.strip() or not args.action.strip():
        parser.error("--observation 和 --action 不能为空")
    try:
        snapshot = request_json(args.api, "/api/progress")
        visual_inventory = snapshot.get("visual_only_review")
        if not isinstance(visual_inventory, dict) or "entries" not in visual_inventory:
            visual_inventory = snapshot["visual_placement"]
        scenes = visual_inventory["entries"]
        if not isinstance(scenes, list) or len(scenes) != TOTAL or len({s["scene_id"] for s in scenes}) != TOTAL:
            raise ValueError("API 不是完整的 154 个唯一场景，停止记录")
        scene = next((row for row in scenes if row["scene_id"] == args.scene), None)
        if scene is None or not scene.get("config_present") or not re.fullmatch(r"[0-9a-f]{64}", scene.get("config_sha256", "")):
            raise ValueError("当前场景或配置 SHA 不可用")
        images = selected_images(scene, args.view)
        with locked():
            latest = OUTPUT / "reviews" / f"{args.scene}.json"
            previous = load_json(latest) if latest.exists() else None
            if previous and previous.get("tool") != TOOL:
                raise ValueError(f"拒绝覆盖非本工具记录：{latest}")
            attempt_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "_" + uuid.uuid4().hex[:8]
            attempt = OUTPUT / "attempts" / args.scene / f"{attempt_id}.json"
            receipt = attempt.with_name(f"{attempt_id}.annotation_sync.json")
            record = {"schema_version": 1, "tool": TOOL, "human_visual_only": True,
                      "scene_id": args.scene, "task_id": scene["task_id"], "variant_id": scene["variant_id"],
                      "status": args.status, "reviewed_at": now(), "observation": args.observation.strip(),
                      "action": args.action.strip(), "config_sha256": scene["config_sha256"],
                      "viewed_images": images, "attempt_id": attempt_id, "attempt_path": relative(attempt),
                      "previous_attempt_path": previous.get("attempt_path") if previous else None,
                      "annotation_sync_path": relative(receipt), "source_api": args.api,
                      "source_api_generated_at": snapshot.get("generated_at"),
                      "scope": "操作者实际看图后提供的静态视觉判断；不含几何、像素或物理评分，也不证明执行或安全成功。"}
            write_new(attempt, record)
            write_atomic(latest, record)
            progress = build_progress(scenes, snapshot.get("generated_at"))
            write_atomic(OUTPUT / "progress.json", progress)
            sync = sync_annotation(args.api, record, previous)
            write_new(receipt, sync)
        print(json.dumps({"saved": relative(latest), "attempt": relative(attempt),
                          "counts": progress["counts"], "annotation_sync": sync["status"],
                          "annotation_sync_reason": sync.get("reason", "已同步至网页")}, ensure_ascii=False))
        return 0
    except (OSError, ValueError, KeyError, TypeError, URLError) as exc:
        print(f"记录失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
