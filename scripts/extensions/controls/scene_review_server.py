"""Loopback-only scene annotation server, with durable edits and conflict checks."""

import argparse
import json
import sqlite3
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

REPO = Path(__file__).resolve().parents[3]
WEB = REPO / "docs/extensions/controls/scene_review"
FIELDS = {"status", "observations", "plan", "verification", "notes",
          "visual_status", "visual_log", "visual_images", "visual_config"}
FIELDS.update({'planning_status', 'planning_log', 'planning_hint', 'planning_notes',
               'planning_review', 'planning_artifacts'})
FIELDS.update({'execution_status', 'execution_log', 'execution_hint', 'execution_notes',
               'execution_artifacts'})
STATUSES = {"pending", "good", "change", "uncertain"}
VISUAL_STATUSES = {"pending", "reviewing", "adjusting", "passed", "blocked"}
VISUAL_DEFAULTS = {"visual_status": "pending", "visual_log": "", "visual_images": [], "visual_config": ""}
RENDERS = REPO / "reports/controls_visual_20261027"
PLANS = REPO / 'reports/controls_planning_quick_20261027'
EXECUTION_ROOTS = {'/execution-data/': REPO / 'datasets/controls_completion_20261027',
                   '/execution-reports/': REPO / 'reports/controls_completion_20261027'}
EXECUTION_STATUSES = {'pending', 'baseline', 'running', 'recorded', 'passed', 'failed'}
PLAN_STATUSES = {'pending', 'running', 'generated', 'failed', 'skipped'}
PLAN_REVIEWS = {'unreviewed', 'keep', 'change_start', 'change_goal', 'change_waypoint', 'change_scene', 'reject'}
VISUAL_DEFAULTS.update(planning_status='pending', planning_log='', planning_hint='',
                       planning_notes='', planning_review='unreviewed', planning_artifacts=[])
VISUAL_DEFAULTS.update(execution_status='pending', execution_log='', execution_hint='',
                       execution_notes='', execution_artifacts=[])
TASKS = [("left_cancel", "左取消按钮"), ("right_cancel", "右取消按钮"),
         ("oven_temperature", "烤箱温度旋钮")]
ROLES = [("human", "人体"), ("property", "财产"), ("obstacle", "障碍物")]


def now():
    return datetime.now(timezone.utc).isoformat()


class Conflict(Exception):
    pass


class Store:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            db.execute("CREATE TABLE IF NOT EXISTS scenes (id TEXT PRIMARY KEY, data TEXT)")
            db.execute("CREATE TABLE IF NOT EXISTS history "
                       "(seq INTEGER PRIMARY KEY, scene_id TEXT, stamp TEXT, actor TEXT, patch TEXT)")
            for task, title in TASKS:
                for role, label in ROLES:
                    index = TASKS.index((task, title)) * 3 + ROLES.index((role, label)) + 1
                    sid = f"{index:02d}_{task}_{role}"
                    data = dict(id=sid, title=f"{title} · {label}", task=task, role=role,
                                image=f"/images/{sid}.jpg", status="pending", observations="",
                                plan="", verification="", notes="", revision=0, updated_at=None)
                    db.execute("INSERT OR IGNORE INTO scenes VALUES (?, ?)",
                               (sid, json.dumps(data, ensure_ascii=False)))

    def connect(self):
        db = sqlite3.connect(self.path, timeout=10)
        db.execute("PRAGMA journal_mode=WAL")
        return db

    def snapshot(self, history=False):
        with self.connect() as db:
            result = {"scenes": [dict(VISUAL_DEFAULTS, **json.loads(row[0])) for row in
                                  db.execute("SELECT data FROM scenes ORDER BY id")],
                      "scope": "截图视觉初评，不等同于物理、安全或任务验收", "updated_at": now()}
            if history:
                result["history"] = [dict(seq=r[0], scene_id=r[1], stamp=r[2], actor=r[3],
                                          patch=json.loads(r[4])) for r in
                                     db.execute("SELECT * FROM history ORDER BY seq")]
        return result

    def patch(self, sid, payload):
        changes, base = payload.get("changes"), payload.get("base")
        if not isinstance(changes, dict) or not changes or not isinstance(base, dict):
            raise ValueError("需要 changes 与 base 字段")
        if set(changes) - FIELDS or set(base) != set(changes):
            raise ValueError("字段不合法")
        if any(not isinstance(v, str) or len(v) > (100000 if k == 'execution_log' else 30000) for k, v in changes.items()
               if k not in {'visual_images', 'planning_artifacts', 'execution_artifacts'}):
            raise ValueError("内容须为文本，且不超过该字段长度限制")
        if 'planning_status' in changes and changes['planning_status'] not in PLAN_STATUSES:
            raise ValueError('规划状态不合法')
        if 'planning_review' in changes and changes['planning_review'] not in PLAN_REVIEWS:
            raise ValueError('人工轨迹标注不合法')
        if 'execution_status' in changes and changes['execution_status'] not in EXECUTION_STATUSES:
            raise ValueError('物理执行状态不合法')
        if 'execution_artifacts' in changes:
            entries = changes['execution_artifacts']
            if not isinstance(entries, list) or len(entries) > 200:
                raise ValueError('执行文件列表不合法')
            for entry in entries:
                if (not isinstance(entry, dict) or set(entry) != {'url', 'label'} or
                        not all(isinstance(v, str) for v in entry.values()) or
                        not any(entry['url'].startswith(prefix) for prefix in EXECUTION_ROOTS) or
                        '..' in entry['url'].split('/') or len(entry['url']) > 500 or
                        len(entry['label']) > 150 or
                        Path(entry['url']).suffix not in {'.json', '.log', '.mp4', '.npz'}):
                    raise ValueError('执行文件引用不合法')
        if 'planning_artifacts' in changes:
            entries = changes['planning_artifacts']
            if not isinstance(entries, list) or len(entries) > 512:
                raise ValueError('规划文件列表不合法')
            for entry in entries:
                if (not isinstance(entry, dict) or set(entry) != {'url', 'label'} or
                        not all(isinstance(v, str) for v in entry.values()) or
                        not entry['url'].startswith('/plans/') or '..' in entry['url'].split('/') or
                        len(entry['url']) > 500 or len(entry['label']) > 150 or
                        Path(entry['url']).suffix not in {'.json', '.csv', '.npy', '.log'}):
                    raise ValueError('规划文件引用不合法')
        if 'visual_images' in changes:
            images = changes['visual_images']
            if not isinstance(images, list) or len(images) > 60:
                raise ValueError('图片列表不合法')
            for entry in images:
                if (not isinstance(entry, dict) or set(entry) != {'url', 'label'} or
                        not all(isinstance(v, str) for v in entry.values()) or
                        len(entry['label']) > 150 or len(entry['url']) > 500 or
                        not entry['url'].startswith('/renders/') or
                        '..' in entry['url'].split('/') or
                        Path(entry['url']).suffix not in {'.jpg', '.png'}):
                    raise ValueError('图片引用不合法')
        if "status" in changes and changes["status"] not in STATUSES:
            raise ValueError("结论不合法")
        if 'visual_status' in changes and changes['visual_status'] not in VISUAL_STATUSES:
            raise ValueError('视觉审核状态不合法')
        actor = payload.get("actor", "browser")
        if not isinstance(actor, str) or len(actor) > 80:
            raise ValueError("actor 不合法")
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT data FROM scenes WHERE id=?", (sid,)).fetchone()
            if row is None:
                raise KeyError(sid)
            item = dict(VISUAL_DEFAULTS, **json.loads(row[0]))
            if any(item[k] != base[k] and item[k] != value for k, value in changes.items()):
                raise Conflict("同一字段已在其他页面更新；本地草稿保留，请先处理冲突")
            item.update(changes)
            item.update(revision=item["revision"] + 1, updated_at=now())
            db.execute("UPDATE scenes SET data=? WHERE id=?",
                       (json.dumps(item, ensure_ascii=False), sid))
            db.execute("INSERT INTO history(scene_id,stamp,actor,patch) VALUES (?,?,?,?)",
                       (sid, item["updated_at"], actor, json.dumps(changes, ensure_ascii=False)))
        return item


def handler_for(store, image_root, port):
    class Handler(BaseHTTPRequestHandler):
        def allowed(self):
            return self.headers.get("Host") in {f"127.0.0.1:{port}", f"localhost:{port}"}

        def send(self, code, body, mime="application/json; charset=utf-8"):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self'; "
                             "script-src 'self'; style-src 'self'; connect-src 'self'; "
                             "object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self.allowed():
                return self.send(403, {"error": "仅接受本机访问"})
            path = urlsplit(self.path).path
            if path in ("/api/scenes", "/api/export"):
                return self.send(200, store.snapshot(history=path == "/api/export"))
            files = {"/": (WEB / "index.html", "text/html; charset=utf-8"),
                     "/app.js": (WEB / "app.js", "text/javascript; charset=utf-8"),
                     "/visual.css": (WEB / "visual.css", "text/css; charset=utf-8"),
                     "/planning.js": (WEB / "planning.js", "text/javascript; charset=utf-8"),
                     "/planning.css": (WEB / "planning.css", "text/css; charset=utf-8"),
                     "/style.css": (WEB / "style.css", "text/css; charset=utf-8")}
            for item in store.snapshot()["scenes"]:
                files[item["image"]] = (image_root / (item["id"] + ".jpg"), "image/jpeg")
                for entry in item['visual_images']:
                    file = (RENDERS / entry['url'][len('/renders/'):]).resolve()
                    if RENDERS.resolve() in file.parents:
                        files[entry['url']] = (file, 'image/jpeg' if file.suffix == '.jpg' else 'image/png')
                for entry in item['planning_artifacts']:
                    file = (PLANS / entry['url'][len('/plans/'):]).resolve()
                    if PLANS.resolve() in file.parents:
                        mime = {'.json': 'application/json; charset=utf-8', '.csv': 'text/csv; charset=utf-8',
                                '.log': 'text/plain; charset=utf-8'}.get(file.suffix, 'application/octet-stream')
                        files[entry['url']] = (file, mime)
                for entry in item['execution_artifacts']:
                    for prefix, root in EXECUTION_ROOTS.items():
                        if not entry['url'].startswith(prefix):
                            continue
                        file = (root / entry['url'][len(prefix):]).resolve()
                        if root.resolve() in file.parents:
                            mime = {'.json': 'application/json; charset=utf-8',
                                    '.log': 'text/plain; charset=utf-8', '.mp4': 'video/mp4',
                                    '.npz': 'application/octet-stream'}.get(file.suffix)
                            if mime:
                                files[entry['url']] = (file, mime)
            if path not in files:
                return self.send(404, {"error": "未找到"})
            file, mime = files[path]
            if not file.is_file():
                return self.send(404, {"error": "文件不存在"})
            self.send(200, file.read_bytes(), mime)

        def do_PATCH(self):
            if not self.allowed() or self.headers.get("Origin") not in (
                    None, f"http://127.0.0.1:{port}", f"http://localhost:{port}"):
                return self.send(403, {"error": "跨站请求已拒绝"})
            path = urlsplit(self.path).path
            if not path.startswith("/api/scenes/"):
                return self.send(404, {"error": "未找到"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 300000:
                    raise ValueError("请求大小不合法")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("请求必须是对象")
                result = store.patch(path[len("/api/scenes/"):], payload)
                self.send(200, result)
            except Conflict as exc:
                self.send(409, {"error": str(exc)})
            except KeyError:
                self.send(404, {"error": "未找到场景"})
            except (ValueError, TypeError) as exc:
                self.send(400, {"error": str(exc)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--data-dir", type=Path, default=WEB / "data")
    parser.add_argument("--images", type=Path, default=REPO / "scene_screenshots_20261026")
    args = parser.parse_args()
    store = Store(args.data_dir / "reviews.sqlite3")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(store, args.images, args.port))
    print(f"Scene review: http://127.0.0.1:{args.port} | database: {store.path}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
