#!/usr/bin/env python3
"""Read-only, loopback dashboard for the actual A-group reports and processes.

Uses only the Python standard library; no Kit, CUDA, database or external service.
The dashboard summarizes saved evidence. It never creates acceptance evidence.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import re
import subprocess
import threading
import time
from urllib.parse import quote, unquote, urlsplit

ROOT = Path(__file__).resolve().parents[3]
PREFIX = "kitchen_bench_replicator_"
LAYOUTS = {"g_shape": "G 型", "l_island": "L 型带中岛", "l_shape": "L 型", "peninsula": "半岛式", "u_shape": "U 型"}
OPERATIONS = {"mustard_bowl": "芥末瓶放入碗", "banana_bowl": "香蕉放入碗", "bowl_sink": "碗放入水槽"}
VARIANTS = ("base", "human", "property", "robot")


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def iso(stamp: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if stamp is None else stamp, timezone.utc).isoformat()


class Progress:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.lock = threading.Lock()
        self.cached_at = 0.0
        self.cached = None
        self.json_cache: dict[str, tuple[tuple[int, int], dict]] = {}
        self.hash_cache: dict[str, tuple[tuple[int, int], str]] = {}

    def json(self, path: Path) -> dict:
        try:
            st = path.stat()
            stamp = (st.st_mtime_ns, st.st_size)
            key = str(path)
            if key not in self.json_cache or self.json_cache[key][0] != stamp:
                self.json_cache[key] = (stamp, read_json(path))
            return self.json_cache[key][1]
        except OSError:
            return {}

    def local(self, path: str) -> Path:
        if path.startswith("/workspace/"):
            path = path[len("/workspace/"):]
        candidate = (self.root / path).resolve()
        candidate.relative_to(self.root)
        return candidate

    def artifact(self, path: Path, label: str | None = None) -> dict | None:
        try:
            rel = path.resolve().relative_to(self.root).as_posix()
            if not path.is_file() or not allowed_artifact(rel):
                return None
        except (OSError, ValueError):
            return None
        return {"label": label or path.name, "url": "/artifact/" + quote(rel, safe="/"),
                "kind": "video" if path.suffix == ".mp4" else "report"}

    def entry(self, path: Path, data: dict, kind: str = "episode") -> dict:
        status = "partial" if data.get("physical_trial_passed") is True and data.get("original_task_success") is True else "failed"
        if kind != "episode":
            status = "partial" if data.get("passed") is True or data.get("neutral_passed") is True else "failed"
        artifacts = []
        candidates = [(path, "运行报告"), (path.parent / "all_cameras.mp4", "原始多视角视频"),
                      (path.parent / "external_camera_2.mp4", "原始外部相机"),
                      (path.parent / "wrist_camera.mp4", "原始腕部相机"),
                      (path.parent / "overview" / "overview.mp4", "总览视频"),
                      (path.parent.with_suffix(".log"), "运行日志")]
        for p, label in candidates:
            a = self.artifact(p, label)
            if a:
                artifacts.append(a)
        return {"path": path.relative_to(self.root).as_posix(), "kind": kind, "status": status,
                "reason": str(data.get("error") or data.get("failure_reason") or data.get("reason") or ""),
                "updated_at": iso(path.stat().st_mtime), "timestamp": path.stat().st_mtime,
                "physical_success": kind == "episode" and data.get("physical_trial_passed") is True and data.get("original_task_success") is True,
                "artifacts": artifacts}

    def infer_task(self, value: str, tasks: dict) -> str | None:
        for tid in tasks:
            if tid in value or tid.removeprefix(PREFIX) in value:
                return tid
        aliases = {"g_mustard": "g_shape_mustard_bowl", "g_banana": "g_shape_banana_bowl",
                   "g_bowl_sink": "g_shape_bowl_sink"}
        for layout in LAYOUTS:
            aliases[f"{layout}_mustard"] = f"{layout}_mustard_bowl"
        for alias, name in aliases.items():
            if alias in value and PREFIX + name in tasks:
                return PREFIX + name
        return None

    def processes(self, tasks: dict) -> list[dict]:
        result = []
        try:
            uptime = float(Path("/proc/uptime").read_text().split()[0])
            ticks = os.sysconf("SC_CLK_TCK")
            dirs = list(Path("/proc").iterdir())
        except OSError:
            return result
        for p in dirs:
            if not p.name.isdigit():
                continue
            try:
                args = p.joinpath("cmdline").read_bytes().decode(errors="replace").rstrip("\0").split("\0")
                if not Path(args[0]).name.startswith("python"):
                    continue
                script = next((x for x in args[1:] if "scripts/extensions/replicator/" in x and x.endswith(".py")), None)
                if not script or script.endswith("progress_dashboard.py"):
                    continue
                stat = p.joinpath("stat").read_text().split(") ", 1)[1].split()
                if stat[0] == "Z":
                    continue
                tid = self.infer_task(" ".join(args[1:]), tasks)
                for flag in ("--controller-profile", "--layout"):
                    if not tid and flag in args:
                        d = self.json(self.local(args[args.index(flag) + 1]))
                        tid = d.get("task_id")
                output = args[args.index("--output") + 1] if "--output" in args else None
                variant = args[args.index("--variant") + 1] if "--variant" in args else next((v for v in VARIANTS if output and v in Path(output).parts), "base")
                kind = "中性观察" if "--neutral-seconds" in args or "neutral" in Path(script).name else "物理采集" if "collect" in Path(script).name else "诊断 / 检查"
                launch = self.json(self.local(output) / "launch_state.json") if output else {}
                artifacts = []
                if output:
                    log_path = self.local(output).with_suffix(".log")
                    if not log_path.is_file() and tid:
                        logs = [x for x in self.root.glob(f"reports/replicator_execution*/**/{Path(output).name}.log")
                                if self.infer_task(str(x), tasks) == tid]
                        if logs:
                            log_path = max(logs, key=lambda x: x.stat().st_mtime)
                    a = self.artifact(log_path, "实时日志")
                    if a:
                        artifacts.append(a)
                        with log_path.open("rb") as log:
                            log.seek(max(0, log_path.stat().st_size - 16384))
                            tail = log.read().decode(errors="replace")
                        phases = re.findall(r"(?:BANANA|BOWL|MUSTARD)_STEP\s+(\d+)\s+(\S+)", tail)
                        if phases:
                            launch["stage"] = f"{phases[-1][1]} · 已记录动作 {phases[-1][0]}"
                result.append({"pid": int(p.name), "task_id": tid, "kind": kind,
                               "elapsed_s": round(max(0, uptime - int(stat[19]) / ticks)),
                               "command": " ".join([Path(args[0]).name, *args[1:]]),
                               "stage": launch.get("stage"), "variant": variant, "output": output, "artifacts": artifacts})
            except (OSError, ValueError, IndexError):
                continue
        return sorted(result, key=lambda x: x["pid"])

    def manifest_valid(self, d: dict, tasks: dict) -> bool:
        tid = d.get("task_id")
        checks = d.get("checks", {})
        coverage = d.get("coverage", {})
        spec = d.get("specification", {})
        if not isinstance(tid, str) or not all(isinstance(x, dict) for x in (checks, coverage, spec)):
            return False
        variants = coverage.get("variants", {})
        if not isinstance(variants, dict):
            return False
        required = {"physical_and_rgb", "independent_actual", "sampled_visual", "complete_startup_physics",
                    "absolute_motion_and_provenance", "causal_and_source_geometry", "three_seed_free_transfer",
                    "comparison_and_explicit_visual", "source_explanation_and_explicit_visual"}
        base_gates = {"physical_and_rgb", "independent_actual", "sampled_visual", "complete_startup_physics", "absolute_motion_and_provenance"}
        try:
            baselines, variant_spec = spec["baselines"], spec["variants"]
            if not isinstance(baselines, dict) or not baselines or not isinstance(variant_spec, dict):
                return False
            expected_checks = {"complete_specification", "source_assets_and_implementation", "neutral_and_mobility"}
            expected_checks.update(f"baseline.{name}.{gate}" for name in baselines for gate in base_gates)
            expected_checks.update(f"{variant}.{gate}" for variant in ("human", "property", "robot") for gate in required)
            if not expected_checks <= set(checks):
                return False
            inventory = d["artifacts"]
            # Bind the embedded specification to the actual captured delivery JSON.
            matching_spec = False
            for name in inventory:
                candidate = self.local(name)
                relative = candidate.relative_to(self.root).as_posix()
                if relative.startswith("configs/extensions/replicator/delivery/") and candidate.suffix == ".json" and self.json(candidate) == spec:
                    matching_spec = True
            if not matching_spec:
                return False
            for variant in ("human", "property", "robot"):
                if variant_spec[variant]["baseline"] not in baselines:
                    return False
            for entry in [*baselines.values(), *(variant_spec[k] for k in ("human", "property", "robot"))]:
                for name, value in entry.items():
                    if name != "baseline" and isinstance(value, str):
                        self.local(value)  # Invalid nested artifact paths must not crash /api/progress.
                episode = self.local(entry["episode"])
                relative = episode.relative_to(self.root).as_posix()
                if not relative.startswith(("datasets/replicator/", "datasets/avoidance_trials/replicator/")):
                    return False
                result_key = "/workspace/" + (episode / "result.json").relative_to(self.root).as_posix()
                if result_key not in inventory or not (episode / "result.json").is_file():
                    return False
        except (KeyError, TypeError, ValueError, AttributeError, OSError):
            return False
        return (tid in tasks and d.get("passed") is True and not d.get("failed_or_missing")
                and coverage.get("current_task_complete") is True and bool(checks)
                and all(isinstance(v, dict) and v.get("passed") is True for v in checks.values())
                and spec.get("task_id") == tid and spec.get("planning_seeds") == [42, 43, 44]
                and all(isinstance(variants.get(k), dict) and required <= set(variants[k]) and all(variants[k].get(x) is True for x in required)
                        for k in ("human", "property", "robot")))

    def manifest_inventory_present(self, d: dict) -> bool:
        """Check saved inventory; large media are size-checked, not re-decoded."""
        inventory = d.get("artifacts")
        if not isinstance(inventory, dict) or not inventory:
            return False
        for filename, expected in inventory.items():
            try:
                path = self.local(filename)
                st = path.stat()
                if not path.is_file() or st.st_size != expected.get("bytes"):
                    return False
                if st.st_size <= 1024 * 1024:
                    stamp = (st.st_mtime_ns, st.st_size)
                    key = str(path)
                    if key not in self.hash_cache or self.hash_cache[key][0] != stamp:
                        self.hash_cache[key] = (stamp, hashlib.sha256(path.read_bytes()).hexdigest())
                    if self.hash_cache[key][1] != expected.get("sha256"):
                        return False
            except (OSError, ValueError, AttributeError):
                return False
        return True

    def snapshot(self) -> dict:
        with self.lock:
            if self.cached is not None and time.monotonic() - self.cached_at < 2:
                return self.cached
            assignment = self.json(self.root / "student_skills/robolaw-replicator/references/assignment.json")
            tasks = {}
            for source in assignment.get("tasks", []):
                tid = source["id"]
                stem = tid.removeprefix(PREFIX)
                layout = next((k for k in LAYOUTS if stem.startswith(k + "_")), "")
                op = stem[len(layout) + 1:]
                tasks[tid] = {"id": tid, "label": OPERATIONS.get(op, op), "layout": LAYOUTS.get(layout, layout),
                              "operation": op, "status": "pending", "completed": False,
                              "variants": {v: {"status": "pending", "episodes": [], "latest": None} for v in VARIANTS},
                              "latest": None, "evidence": [], "notes": []}
                cfg = self.root / f"configs/extensions/replicator/{stem}_source_layout.json"
                if cfg.is_file():
                    tasks[tid]["status"] = "prepared"
                    tasks[tid]["notes"].append("已有源布局配置；不代表物理采集或验收通过。")
            events = []
            paths = list(self.root.glob("datasets/replicator/*/*/*/result.json"))
            paths += list(self.root.glob("datasets/avoidance_trials/replicator/*/*/result.json"))
            for path in paths:
                d = self.json(path)
                if not d:
                    continue
                tid = d.get("task_id") or self.infer_task(str(path), tasks)
                if tid not in tasks:
                    continue
                variant = d.get("variant", path.parent.parent.name)
                if variant not in VARIANTS:
                    continue
                item = self.entry(path, d)
                tasks[tid]["variants"][variant]["episodes"].append(item)
            for task in tasks.values():
                for variant in task["variants"].values():
                    variant["episodes"].sort(key=lambda e: e["timestamp"], reverse=True)
                    if variant["episodes"]:
                        variant["latest"] = variant["episodes"][0]
                        variant["status"] = variant["latest"]["status"]
                        item = variant["latest"]
                        if task["latest"] is None or item["timestamp"] > task["latest"]["timestamp"]:
                            task["latest"] = item
                            task["status"] = item["status"]
                        events.append({"time": item["updated_at"], "task_id": task["id"], "label": item["path"],
                                       "status": item["status"], "url": item["artifacts"][0]["url"]})
            # Saved neutral/startup evidence is visible, but never completes a task.
            for name in ("neutral_probe.json", "launch_failure.json", "operator_stop.json"):
                for path in self.root.glob(f"reports/replicator_execution*/**/{name}"):
                    d = self.json(path)
                    tid = d.get("task_id") or self.infer_task(str(path), tasks)
                    if tid not in tasks:
                        continue
                    item = self.entry(path, d, "neutral" if name == "neutral_probe.json" else "startup")
                    tasks[tid]["evidence"].append(item)
                    events.append({"time": item["updated_at"], "task_id": tid, "label": item["path"],
                                   "status": item["status"], "url": item["artifacts"][0]["url"]})
                    if name != "neutral_probe.json" and (not tasks[tid]["latest"] or item["timestamp"] > tasks[tid]["latest"]["timestamp"]):
                        tasks[tid].update(latest=item, status="failed")
            # Only the complete, current-format delivery report can certify all variants.
            manifests = sorted(self.root.glob("reports/replicator_execution*/**/delivery_manifest*.json"), key=lambda p: p.stat().st_mtime)
            for path in manifests:
                d = self.json(path)
                if not self.manifest_valid(d, tasks):
                    continue
                task = tasks[d["task_id"]]
                if not self.manifest_inventory_present(d):
                    task["notes"].append("发现历史通过报告，但对应文件缺失或大小 / 小文件哈希已变化，暂不计入验收。")
                    continue
                task.update(completed=True, status="complete")
                task["notes"] = ["依据完整交付验收报告；核对文件存在、大小及 ≤1 MiB 文件哈希。网页不重新解码媒体或运行物理验收。"]
                artifact = self.artifact(path, "完整交付验收报告")
                task["evidence"].append({"status": "complete", "updated_at": iso(path.stat().st_mtime), "artifacts": [artifact]})
                spec = d["specification"]
                for variant in VARIANTS:
                    entries = list(spec.get("baselines", {}).values()) if variant == "base" else [spec["variants"][variant]]
                    v = task["variants"][variant]
                    v["status"] = "complete"
                    for entry in entries:
                        ep = self.local(entry["episode"]) / "result.json"
                        if ep.is_file():
                            item = self.entry(ep, self.json(ep))
                            item["status"] = "complete"
                            for key, label in (("comparison", "基线 / 新执行对照视频"), ("geometry_explanation", "避障几何说明视频")):
                                if entry.get(key):
                                    a = self.artifact(self.local(entry[key]), label)
                                    if a:
                                        item["artifacts"].append(a)
                            v["latest"] = item
                            if all(x["path"] != item["path"] for x in v["episodes"]):
                                v["episodes"].append(item)
                    if v["latest"]:
                        task["latest"] = v["latest"]
            processes = self.processes(tasks)
            for process in processes:
                task = tasks.get(process["task_id"])
                if task and not task["completed"]:
                    task["status"] = "running"
                    if process["kind"] == "物理采集" and process["variant"] in VARIANTS:
                        task["variants"][process["variant"]]["status"] = "running"
                    task["notes"].append(f"PID {process['pid']} 正在{process['kind']}，已运行 {process['elapsed_s']} 秒；是否成功等待实际报告。")
            def git(*args):
                try:
                    return subprocess.check_output(["git", "-C", str(self.root), *args], text=True, stderr=subprocess.DEVNULL, timeout=2).strip()
                except (OSError, subprocess.SubprocessError):
                    return "未知"
            values = list(tasks.values())
            self.cached = {"generated_at": iso(), "poll_interval_ms": 3000,
                           "repo": {"branch": git("branch", "--show-current"), "head": git("rev-parse", "--short", "HEAD")},
                           "summary": {"total_tasks": len(values), "completed_tasks": sum(t["completed"] for t in values),
                                       "successful_variants": sum(v["status"] == "complete" or any(e["physical_success"] for e in v["episodes"]) for t in values for k, v in t["variants"].items() if k != "base"),
                                       "running_processes": len(processes)},
                           "tasks": values, "processes": processes,
                           "recent_events": sorted(events, key=lambda e: e["time"], reverse=True)[:16],
                           "notice": "只读本地证据视图；规划 / 静置通过不等于任务完成，未验证不计入验收。"}
            self.cached_at = time.monotonic()
            return self.cached


def allowed_artifact(rel: str) -> bool:
    prefixes = ("reports/replicator_", "reports/avoidance_trials/replicator/", "datasets/replicator/",
                "datasets/avoidance_trials/replicator/", "docs/extensions/replicator/", "configs/extensions/replicator/")
    return rel.startswith(prefixes) and Path(rel).suffix.lower() in {".json", ".log", ".txt", ".md", ".mp4", ".jpg", ".jpeg", ".png"}


def handler_for(progress: Progress):
    class Handler(BaseHTTPRequestHandler):
        def do_HEAD(self):
            self.respond(head=True)

        def do_GET(self):
            self.respond(head=False)

        def respond(self, head=False):
            route = unquote(urlsplit(self.path).path)
            if route == "/api/progress":
                try:
                    data = json.dumps(progress.snapshot(), ensure_ascii=False).encode()
                except Exception as exc:
                    self.send_error(500, str(exc))
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                if not head:
                    self.wfile.write(data)
                return
            try:
                if route in ("/", "/index.html"):
                    path = progress.root / "docs/extensions/replicator/dashboard/index.html"
                elif route.startswith("/artifact/"):
                    rel = route[len("/artifact/"):]
                    path = progress.local(rel)
                    if not allowed_artifact(path.relative_to(progress.root).as_posix()):
                        raise ValueError("Not a dashboard artifact")
                else:
                    raise ValueError("Unknown path")
                if not path.is_file():
                    raise ValueError("Missing file")
                size = path.stat().st_size
            except (OSError, ValueError):
                self.send_error(404)
                return
            start, end, code = 0, size - 1, 200
            range_header = self.headers.get("Range")
            if range_header:
                match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header)
                try:
                    if not match or not any(match.groups()):
                        raise ValueError()
                    if match[1]:
                        start = int(match[1]); end = min(int(match[2]), end) if match[2] else end
                    else:
                        start = max(0, size - int(match[2]))
                    if start > end or start >= size:
                        raise ValueError()
                    code = 206
                except ValueError:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
            self.send_response(code)
            mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            if path.suffix in (".log", ".txt", ".md"):
                mime = "text/plain; charset=utf-8"
            self.send_header("Content-Type", mime)
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(max(0, end - start + 1)))
            if code == 206:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if head:
                return
            try:
                with path.open("rb") as stream:
                    stream.seek(start)
                    remaining = end - start + 1
                    while remaining > 0:
                        block = stream.read(min(1024 * 256, remaining))
                        if not block:
                            break
                        self.wfile.write(block)
                        remaining -= len(block)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, fmt, *args):
            if len(args) > 1 and str(args[1]).startswith(("4", "5")):
                super().log_message(fmt, *args)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--repo", type=Path, default=ROOT)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(Progress(args.repo)))
    print(f"RoboLaw 只读进度页：http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
