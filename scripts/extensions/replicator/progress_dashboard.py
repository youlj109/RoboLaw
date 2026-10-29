#!/usr/bin/env python3
"""Loopback dashboard for the actual A-group reports, processes and user notes.

Uses only the Python standard library; no Kit, CUDA, database or external service.
The dashboard summarizes saved evidence. It never creates acceptance evidence.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import tempfile
import time
from urllib.parse import quote, unquote, urlsplit

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator.scene_revision_review import validate_scene_revision
from scripts.extensions.replicator.scene_revision_review_round8 import validate_scene_revision as validate_scene_revision8
PREFIX = "kitchen_bench_replicator_"
LAYOUTS = {"g_shape": "G 型", "l_island": "L 型带中岛", "l_shape": "L 型", "peninsula": "半岛式", "u_shape": "U 型"}
OPERATIONS = {"mustard_bowl": "芥末瓶放入碗", "banana_bowl": "香蕉放入碗", "bowl_sink": "碗放入水槽"}
VARIANTS = ("base", "human", "property", "robot")
SCENE_VARIANTS = (
    ("human_default_mpfb", "人体", "human"),
    ("property_tall_glass", "高水杯", "property"),
    ("property_wine_glass", "红酒杯", "property"),
    ("property_glass_bottle", "玻璃瓶", "property"),
    ("property_ceramic_mug", "陶瓷杯", "property"),
    ("property_vase", "花瓶", "property"),
    ("property_tablet_stand", "平板支架", "property"),
    ("robot_steel_clamp", "钢夹", "robot"),
    ("robot_projecting_crossbar", "突出横杆", "robot"),
    ("robot_narrow_opening", "狭窄开口", "robot"),
    ("robot_sharp_equipment_housing", "尖角设备外壳", "robot"),
)
SCENE_STATUSES = ("pending", "generated", "validated", "loading", "ready", "failed")
SCENE_LOAD_CHECKS = ("simulation_loaded", "render_nonempty", "source_assets_unchanged",
                     "robot_articulation_present", "task_objects_rigid_present", "hazard_collision_enabled")
SAFETY_STATUSES = ("pending", "waiting_baseline", "screening", "geometry_passed", "risk_verified",
                   "executing", "accepted", "needs_repair")
SAFETY_CHECKS = ("geometry_passed", "risk_verified", "native_stability_verified",
                 "safe_execution_verified", "video_saved")
ANNOTATION_STATUSES = ("unreviewed", "adjust", "keep")
ANNOTATION_MAX_NOTE = 20000
ANNOTATION_MAX_BODY = 128 * 1024
VISUAL_ONLY_REPORTS = "reports/replicator_visual_audit_20261025/"
VISUAL_ONLY_CONFIGS = "configs/extensions/replicator/visual_only_review_v1/"
TRAJECTORY_REPORTS = "reports/replicator_trajectory_generation_20261026/"
TRAJECTORY_RETRY_REPORTS = "reports/replicator_trajectory_retry_20261026/"
TRAJECTORY_RETRY2_REPORTS = "reports/replicator_trajectory_retry2_20261026/"
TRAJECTORY_ROOTS = (TRAJECTORY_RETRY2_REPORTS, TRAJECTORY_RETRY_REPORTS, TRAJECTORY_REPORTS)
TRAJECTORY_DIRECTORY = re.compile(r"^replicator_trajectory_(generation|retry([0-9]*))_([0-9]{8})$")
TRAJECTORY_STATUSES = ("pending", "running", "generated", "failed", "skipped")
PHYSICAL_DEMOS = 'exports/replicator_154_physical_demos_20261028/'
PHYSICAL_SOURCE = 'reports/replicator_trajectory_retry10_20261028/'
PHYSICAL_STATUSES = ('pending', 'running', 'success', 'failed', 'error')
CAMERA_RETAKE_DEMOS = 'exports/replicator_u_banana_camera_retakes_20261028/'
CAMERA_RETAKE_PLAN = 'configs/extensions/replicator/physical_camera_retake_plan_20261028.json'
CAMERA_RETAKE_TOTAL = 11
PHYSICAL_RETRY_DEMOS = 'exports/replicator_physical_retry_round1_20261028/'
PHYSICAL_RETRY_PLAN = 'configs/extensions/replicator/physical_retry_round1_20261028.json'
PHYSICAL_RETRY_DIAGNOSTICS = 'reports/replicator_physical_retry_round1_diagnostics_20261028/'
PHYSICAL_RETRY_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round1_diagnostics_detail_20261028/'
PHYSICAL_RETRY_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round1_diagnostics_dark_20261028/'
PHYSICAL_RETRY_TOTAL = 84
PHYSICAL_RETRY_RESULTS = 'configs/extensions/replicator/physical_retry_results_round1_20261028.json'
PHYSICAL_RETRY_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND1_20261028.md'
PHYSICAL_RETRY2_DEMOS = 'exports/replicator_physical_retry_round2_20261028/'
PHYSICAL_RETRY2_PLAN = 'configs/extensions/replicator/physical_retry_round2_20261028.json'
PHYSICAL_RETRY2_PLANNING = 'reports/replicator_physical_retry_round2_planning_20261028/'
PHYSICAL_RETRY2_DIAGNOSTICS = 'reports/replicator_physical_retry_round2_diagnostics_20261028/'
PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round2_diagnostics_detail_20261028/'
PHYSICAL_RETRY2_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round2_diagnostics_dark_20261028/'
PHYSICAL_RETRY2_TOTAL = 48
PHYSICAL_RETRY2_RESULTS = 'configs/extensions/replicator/physical_retry_results_round2_20261028.json'
PHYSICAL_RETRY2_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND2_20261028.md'

PHYSICAL_RETRY3_DEMOS = 'exports/replicator_physical_retry_round3_20261029/'
PHYSICAL_RETRY3_PLAN = 'configs/extensions/replicator/physical_retry_round3_20261029.json'
PHYSICAL_RETRY3_PLANNING = 'reports/replicator_physical_retry_round3_planning_20261029/'
PHYSICAL_RETRY3_DIAGNOSTICS = 'reports/replicator_physical_retry_round3_diagnostics_20261029/'
PHYSICAL_RETRY3_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round3_diagnostics_detail_20261029/'
PHYSICAL_RETRY3_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round3_diagnostics_dark_20261029/'
PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS = 'reports/replicator_physical_retry_round3_diagnostics_overview_20261029/'
PHYSICAL_RETRY3_TOP_DIAGNOSTICS = 'reports/replicator_physical_retry_round3_diagnostics_top_20261029/'
PHYSICAL_RETRY3_TOTAL = 29
PHYSICAL_RETRY3_RESULTS = 'configs/extensions/replicator/physical_retry_results_round3_20261029.json'
PHYSICAL_RETRY3_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND3_20261029.md'


PHYSICAL_RETRY4_DEMOS = 'exports/replicator_physical_retry_round4_20261029/'
PHYSICAL_RETRY4_PLAN = 'configs/extensions/replicator/physical_retry_round4_20261029.json'
PHYSICAL_RETRY4_PLANNING = 'reports/replicator_physical_retry_round4_planning_20261029/'
PHYSICAL_RETRY4_DIAGNOSTICS = 'reports/replicator_physical_retry_round4_diagnostics_20261029/'
PHYSICAL_RETRY4_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round4_diagnostics_detail_20261029/'
PHYSICAL_RETRY4_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round4_diagnostics_dark_20261029/'
PHYSICAL_RETRY4_OVERVIEW_DIAGNOSTICS = 'reports/replicator_physical_retry_round4_diagnostics_overview_20261029/'
PHYSICAL_RETRY4_TOP_DIAGNOSTICS = 'reports/replicator_physical_retry_round4_diagnostics_top_20261029/'
PHYSICAL_RETRY4_TOTAL = 6
PHYSICAL_RETRY4_RESULTS = 'configs/extensions/replicator/physical_retry_results_round4_20261029.json'
PHYSICAL_RETRY4_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND4_20261029.md'


PHYSICAL_RETRY5_DEMOS = 'exports/replicator_physical_retry_round5_20261029/'
PHYSICAL_RETRY5_PLAN = 'configs/extensions/replicator/physical_retry_round5_20261029.json'
PHYSICAL_RETRY5_PLANNING = 'reports/replicator_physical_retry_round5_planning_20261029/'
PHYSICAL_RETRY5_DIAGNOSTICS = 'reports/replicator_physical_retry_round5_diagnostics_20261029/'
PHYSICAL_RETRY5_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round5_diagnostics_detail_20261029/'
PHYSICAL_RETRY5_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round5_diagnostics_dark_20261029/'
PHYSICAL_RETRY5_OVERVIEW_DIAGNOSTICS = 'reports/replicator_physical_retry_round5_diagnostics_overview_20261029/'
PHYSICAL_RETRY5_TOP_DIAGNOSTICS = 'reports/replicator_physical_retry_round5_diagnostics_top_20261029/'
PHYSICAL_RETRY5_TOTAL = 5
PHYSICAL_RETRY5_RESULTS = 'configs/extensions/replicator/physical_retry_results_round5_20261029.json'
PHYSICAL_RETRY5_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND5_20261029.md'


PHYSICAL_RETRY6_DEMOS = 'exports/replicator_physical_retry_round6_20261029/'
PHYSICAL_RETRY6_PLAN = 'configs/extensions/replicator/physical_retry_round6_20261029.json'
PHYSICAL_RETRY6_PLANNING = 'reports/replicator_physical_retry_round6_planning_20261029/'
PHYSICAL_RETRY6_DIAGNOSTICS = 'reports/replicator_physical_retry_round6_diagnostics_20261029/'
PHYSICAL_RETRY6_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round6_diagnostics_detail_20261029/'
PHYSICAL_RETRY6_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round6_diagnostics_dark_20261029/'
PHYSICAL_RETRY6_OVERVIEW_DIAGNOSTICS = 'reports/replicator_physical_retry_round6_diagnostics_overview_20261029/'
PHYSICAL_RETRY6_TOP_DIAGNOSTICS = 'reports/replicator_physical_retry_round6_diagnostics_top_20261029/'
PHYSICAL_RETRY6_TOTAL = 3
PHYSICAL_RETRY6_RESULTS = 'configs/extensions/replicator/physical_retry_results_round6_20261029.json'
PHYSICAL_RETRY6_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND6_20261029.md'


PHYSICAL_RETRY7_DEMOS = 'exports/replicator_physical_retry_round7_20261029/'
PHYSICAL_RETRY7_PLAN = 'configs/extensions/replicator/physical_retry_round7_20261029.json'
PHYSICAL_RETRY7_PLANNING = 'reports/replicator_physical_retry_round7_planning_20261029/'
PHYSICAL_RETRY7_DIAGNOSTICS = 'reports/replicator_physical_retry_round7_diagnostics_20261029/'
PHYSICAL_RETRY7_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round7_diagnostics_detail_20261029/'
PHYSICAL_RETRY7_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round7_diagnostics_dark_20261029/'
PHYSICAL_RETRY7_OVERVIEW_DIAGNOSTICS = 'reports/replicator_physical_retry_round7_diagnostics_overview_20261029/'
PHYSICAL_RETRY7_TOP_DIAGNOSTICS = 'reports/replicator_physical_retry_round7_diagnostics_top_20261029/'
PHYSICAL_RETRY7_TOTAL = 3
PHYSICAL_RETRY7_RESULTS = 'configs/extensions/replicator/physical_retry_results_round7_20261029.json'
PHYSICAL_RETRY7_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND7_20261029.md'
PHYSICAL_RETRY8_DEMOS = 'exports/replicator_physical_retry_round8_20261029/'
PHYSICAL_RETRY8_PLAN = 'configs/extensions/replicator/physical_retry_round8_20261029.json'
PHYSICAL_RETRY8_PLANNING = 'reports/replicator_physical_retry_round8_planning_20261029/'
PHYSICAL_RETRY8_DIAGNOSTICS = 'reports/replicator_physical_retry_round8_diagnostics_20261029/'
PHYSICAL_RETRY8_DETAIL_DIAGNOSTICS = 'reports/replicator_physical_retry_round8_diagnostics_detail_20261029/'
PHYSICAL_RETRY8_DARK_DIAGNOSTICS = 'reports/replicator_physical_retry_round8_diagnostics_dark_20261029/'
PHYSICAL_RETRY8_OVERVIEW_DIAGNOSTICS = 'reports/replicator_physical_retry_round8_diagnostics_overview_20261029/'
PHYSICAL_RETRY8_TOP_DIAGNOSTICS = 'reports/replicator_physical_retry_round8_diagnostics_top_20261029/'
PHYSICAL_RETRY8_TOTAL = 2
PHYSICAL_RETRY8_RESULTS = 'configs/extensions/replicator/physical_retry_results_round8_20261029.json'
PHYSICAL_RETRY8_REPORT = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND8_20261029.md'


def retry_paths(retry_round=1):
    if retry_round == 1:
        return dict(demos=PHYSICAL_RETRY_DEMOS,plan=PHYSICAL_RETRY_PLAN,diagnostics=PHYSICAL_RETRY_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY_DARK_DIAGNOSTICS,
                    total=PHYSICAL_RETRY_TOTAL,results=PHYSICAL_RETRY_RESULTS,report=PHYSICAL_RETRY_REPORT)
    if retry_round == 2:
        return dict(demos=PHYSICAL_RETRY2_DEMOS,plan=PHYSICAL_RETRY2_PLAN,diagnostics=PHYSICAL_RETRY2_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY2_DARK_DIAGNOSTICS,
                    total=PHYSICAL_RETRY2_TOTAL,results=PHYSICAL_RETRY2_RESULTS,report=PHYSICAL_RETRY2_REPORT)
    if retry_round == 3:
        return dict(demos=PHYSICAL_RETRY3_DEMOS,plan=PHYSICAL_RETRY3_PLAN,diagnostics=PHYSICAL_RETRY3_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY3_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY3_DARK_DIAGNOSTICS,
                    overview_diagnostics=PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS,top_diagnostics=PHYSICAL_RETRY3_TOP_DIAGNOSTICS,
                    total=PHYSICAL_RETRY3_TOTAL,results=PHYSICAL_RETRY3_RESULTS,report=PHYSICAL_RETRY3_REPORT)
    if retry_round == 4:
        return dict(demos=PHYSICAL_RETRY4_DEMOS,plan=PHYSICAL_RETRY4_PLAN,diagnostics=PHYSICAL_RETRY4_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY4_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY4_DARK_DIAGNOSTICS,
                    overview_diagnostics=PHYSICAL_RETRY4_OVERVIEW_DIAGNOSTICS,top_diagnostics=PHYSICAL_RETRY4_TOP_DIAGNOSTICS,
                    total=PHYSICAL_RETRY4_TOTAL,results=PHYSICAL_RETRY4_RESULTS,report=PHYSICAL_RETRY4_REPORT)
    if retry_round == 5:
        return dict(demos=PHYSICAL_RETRY5_DEMOS,plan=PHYSICAL_RETRY5_PLAN,diagnostics=PHYSICAL_RETRY5_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY5_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY5_DARK_DIAGNOSTICS,
                    overview_diagnostics=PHYSICAL_RETRY5_OVERVIEW_DIAGNOSTICS,top_diagnostics=PHYSICAL_RETRY5_TOP_DIAGNOSTICS,
                    total=PHYSICAL_RETRY5_TOTAL,results=PHYSICAL_RETRY5_RESULTS,report=PHYSICAL_RETRY5_REPORT)
    if retry_round == 6:
        return dict(demos=PHYSICAL_RETRY6_DEMOS,plan=PHYSICAL_RETRY6_PLAN,diagnostics=PHYSICAL_RETRY6_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY6_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY6_DARK_DIAGNOSTICS,
                    overview_diagnostics=PHYSICAL_RETRY6_OVERVIEW_DIAGNOSTICS,top_diagnostics=PHYSICAL_RETRY6_TOP_DIAGNOSTICS,
                    total=PHYSICAL_RETRY6_TOTAL,results=PHYSICAL_RETRY6_RESULTS,report=PHYSICAL_RETRY6_REPORT)
    if retry_round == 7:
        return dict(demos=PHYSICAL_RETRY7_DEMOS,plan=PHYSICAL_RETRY7_PLAN,diagnostics=PHYSICAL_RETRY7_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY7_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY7_DARK_DIAGNOSTICS,
                    overview_diagnostics=PHYSICAL_RETRY7_OVERVIEW_DIAGNOSTICS,top_diagnostics=PHYSICAL_RETRY7_TOP_DIAGNOSTICS,
                    total=PHYSICAL_RETRY7_TOTAL,results=PHYSICAL_RETRY7_RESULTS,report=PHYSICAL_RETRY7_REPORT)
    if retry_round == 8:
        return dict(demos=PHYSICAL_RETRY8_DEMOS,plan=PHYSICAL_RETRY8_PLAN,diagnostics=PHYSICAL_RETRY8_DIAGNOSTICS,
                    detail_diagnostics=PHYSICAL_RETRY8_DETAIL_DIAGNOSTICS,dark_diagnostics=PHYSICAL_RETRY8_DARK_DIAGNOSTICS,
                    overview_diagnostics=PHYSICAL_RETRY8_OVERVIEW_DIAGNOSTICS,top_diagnostics=PHYSICAL_RETRY8_TOP_DIAGNOSTICS,
                    total=PHYSICAL_RETRY8_TOTAL,results=PHYSICAL_RETRY8_RESULTS,report=PHYSICAL_RETRY8_REPORT)
    raise ValueError('Unsupported physical retry round')


def retry2_candidate_path(root, path, scene_id, retry_round=2):
    """Only the current planning tree and retained pre-solver startup recoveries."""
    try:tail=path.relative_to(root/({2:PHYSICAL_RETRY2_PLANNING,3:PHYSICAL_RETRY3_PLANNING,4:PHYSICAL_RETRY4_PLANNING,5:PHYSICAL_RETRY5_PLANNING,6:PHYSICAL_RETRY6_PLANNING,7:PHYSICAL_RETRY7_PLANNING,8:PHYSICAL_RETRY8_PLANNING}[retry_round])).as_posix()
    except ValueError:return False
    return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/'+re.escape(scene_id)+r'/result\.json',tail))


def trajectory_report_roots(root):
    roots = set(TRAJECTORY_ROOTS)
    for path in (root / 'reports').glob('replicator_trajectory_*'):
        if path.is_dir() and TRAJECTORY_DIRECTORY.fullmatch(path.name):
            roots.add(path.relative_to(root).as_posix() + '/')
    def order(path):
        match = TRAJECTORY_DIRECTORY.fullmatch(Path(path.rstrip('/')).name)
        return (match[3], 0 if match[1] == 'generation' else int(match[2] or 1))
    return tuple(sorted(roots, key=order, reverse=True))


class AnnotationError(Exception):
    def __init__(self, status: int, message: str, entry: dict | None = None):
        super().__init__(message)
        self.status, self.message, self.entry = status, message, entry


def derive_suggestion(result: dict) -> str:
    """Stage-aware manual adjustment hints; never a collision-object diagnosis."""
    if result.get("annotation_only") is True or result.get('scene_settings_changed_once') is True:
        return str(result.get("suggestion") or "")
    code, stage = str(result.get("failure_code") or ""), str(result.get("stage") or "")
    initial_stages = {"", "pending", "queued", "planning", "source_composition", "target_preparation",
                      "planner_initialization", "pregrasp", "initial", "initialization", "start"}
    if result.get("status") == "failed" and ("INVALID_START" in code or "START_STATE" in code):
        if stage in initial_stages:
            return ("布局固定：先核对源初态到规划世界的坐标变换、机器人碰撞模型和夹爪包络。"
                    "若确为当前起点的真实碰撞，单改抓取点或抓取姿态无法消除；保留不可规划记录，"
                    "不搬动场景实体、不重置初始关节，也不关闭碰撞。该报错不定位具体碰撞物。")
        label = {"approach": "接近抓取", "lift": "抬升", "transfer": "转运", "lower": "放置",
                 "retreat": "撤离"}.get(stage, stage)
        return (f"布局固定：检查【{label}】阶段起点（上一阶段末端）；结合部分路径，考虑抓取点、抓取姿态、闭爪后的携物包络、"
                "起始抬升方向或路点。该报错不定位具体碰撞物。")
    return str(result.get("suggestion") or "")


def valid_digest(value) -> bool:
    return isinstance(value, str) and bool(re.fullmatch(r"[0-9a-f]{64}", value))


def valid_annotation_views(value) -> bool:
    return (isinstance(value, list) and len(value) <= 32
            and all(isinstance(v, dict) and set(v) == {"view_id", "preview_sha256"}
                    and isinstance(v["view_id"], str)
                    and bool(re.fullmatch(r"[A-Za-z0-9_-]{1,100}", v["view_id"]))
                    and valid_digest(v["preview_sha256"]) for v in value)
            and len({v["view_id"] for v in value}) == len(value))


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def iso(stamp: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if stamp is None else stamp, timezone.utc).isoformat()


def physical_success(data: dict) -> bool:
    """Normalize the two saved collector schemas without treating planning as execution."""
    if (data.get("diagnostic_only") is True or data.get("status") in ("failed", "error") or data.get("error")
            or data.get("physical_failure_reasons") or data.get("clearance_failure")):
        return False
    if "physical_trial_passed" in data or "original_task_success" in data:
        return data.get("physical_trial_passed") is True and data.get("original_task_success") is True
    return all(data.get(key) is True for key in ("success", "arena_success", "physics_grasp", "final_on_destination"))


class Progress:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.lock = threading.Lock()
        self.cached_at = 0.0
        self.cached = None
        self.json_cache: dict[str, tuple[tuple[int, int], dict]] = {}
        self.hash_cache: dict[str, tuple[tuple[int, int], str]] = {}
        self.log_cache: dict[str, tuple[tuple[int, int], set[Path]]] = {}
        self.report_logs: dict[str, list[Path]] | None = None
        self.annotation_lock = threading.Lock()
        self.trajectory_annotation_lock = threading.Lock()
        self.physical_retry_annotation_lock = threading.Lock()

    @property
    def annotation_path(self) -> Path:
        return self.root / "reports/replicator_dashboard/scene_annotations.json"

    @contextmanager
    def locked_annotations(self, exclusive=False):
        """A separate inode protects atomic replacement across threads/processes."""
        with self.annotation_lock:
            self.annotation_path.parent.mkdir(parents=True, exist_ok=True)
            with self.annotation_path.with_suffix(".lock").open("a+b") as lockfile:
                fcntl.flock(lockfile.fileno(), fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
                try:
                    yield
                finally:
                    fcntl.flock(lockfile.fileno(), fcntl.LOCK_UN)

    def read_annotations_unlocked(self) -> dict:
        """Do not use forgiving report parsing: invalid notes must never be lost."""
        try:
            raw = self.annotation_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return {"schema_version": 1, "entries": []}
        except (OSError, UnicodeError) as exc:
            raise AnnotationError(503, "无法读取标注文件；已停止保存以保护现有内容。") from exc
        def require(condition):
            if not condition:
                raise ValueError("Invalid annotation storage")
        try:
            data = json.loads(raw)
            require(isinstance(data, dict) and type(data.get("schema_version")) is int and data["schema_version"] == 1)
            entries = data["entries"]
            require(isinstance(entries, list))
            seen = set()
            for entry in entries:
                require(isinstance(entry, dict))
                require(set(entry) == {"scene_id", "task_id", "variant_id", "status", "note", "revision",
                                      "updated_at", "config_sha256", "views"})
                require(isinstance(entry["scene_id"], str) and re.fullmatch(r"[a-z0-9_]{1,160}", entry["scene_id"]))
                require(entry["scene_id"] not in seen)
                seen.add(entry["scene_id"])
                require(isinstance(entry["task_id"], str) and isinstance(entry["variant_id"], str))
                require(entry["scene_id"] == entry["task_id"].removeprefix(PREFIX) + "__" + entry["variant_id"])
                require(isinstance(entry["status"], str) and entry["status"] in ANNOTATION_STATUSES)
                require(isinstance(entry["note"], str) and len(entry["note"]) <= ANNOTATION_MAX_NOTE)
                require(type(entry["revision"]) is int and entry["revision"] >= 1)
                require(isinstance(entry["updated_at"], str) and datetime.fromisoformat(entry["updated_at"]).tzinfo is not None)
                require(valid_digest(entry["config_sha256"]) and valid_annotation_views(entry["views"]))
            return data
        except (ValueError, KeyError, TypeError) as exc:
            raise AnnotationError(503, "标注文件格式异常；已停止保存以保护现有内容，请先恢复文件。") from exc

    def annotations(self) -> dict:
        try:
            with self.locked_annotations():
                return self.read_annotations_unlocked()
        except OSError as exc:
            raise AnnotationError(503, "无法读取标注文件；请检查本地存储。") from exc

    def save_annotation(self, payload: dict) -> dict:
        fields = {"scene_id", "status", "note", "expected_revision", "config_sha256", "views"}
        if (not isinstance(payload, dict) or not fields - {"views"} <= set(payload)
                or set(payload) - fields):
            raise AnnotationError(400, "标注字段不完整或包含不支持的字段。")
        if (not isinstance(payload["scene_id"], str)
                or not re.fullmatch(r"[a-z0-9_]{1,160}", payload["scene_id"])
                or not isinstance(payload["status"], str) or payload["status"] not in ANNOTATION_STATUSES
                or not isinstance(payload["note"], str) or len(payload["note"]) > ANNOTATION_MAX_NOTE
                or type(payload["expected_revision"]) is not int or payload["expected_revision"] < 0
                or not valid_digest(payload["config_sha256"])
                or not valid_annotation_views(payload.get("views", []))):
            raise AnnotationError(400, "标注内容无效；调整方案最多 20000 字。")
        # Refresh actual bound config/images rather than accepting client paths or cached evidence.
        with self.lock:
            assignment = self.json(self.root / "student_skills/robolaw-replicator/references/assignment.json")
            tasks = {row["id"]: {} for row in assignment.get("tasks", [])
                     if isinstance(row, dict) and isinstance(row.get("id"), str)}
            scene = next((row for row in self.visual_only_review(tasks)["entries"]
                          if row["scene_id"] == payload["scene_id"]), None)
        if scene is None or not scene["config_present"]:
            raise AnnotationError(400, "场景不存在或当前配置无法验证，请刷新场景列表。")
        current_views = {(v["view_id"], v["preview_sha256"]) for v in scene["views"]}
        try:
            with self.locked_annotations(exclusive=True):
                data = self.read_annotations_unlocked()
                current = next((r for r in data["entries"] if r["scene_id"] == payload["scene_id"]), None)
                revision = current["revision"] if current else 0
                if payload["expected_revision"] != revision:
                    raise AnnotationError(409, "此场景的标注已在其他页面更新，请比较最新内容后再保存。", current)
                if payload["config_sha256"] != scene["config_sha256"]:
                    raise AnnotationError(409, "场景配置已更新，请刷新图片后再保存标注。", current)
                if any((v["view_id"], v["preview_sha256"]) not in current_views for v in payload.get("views", [])):
                    raise AnnotationError(409, "场景图片已变化或不属于当前视图，请刷新图片后再保存标注。", current)
                entry = {"scene_id": scene["scene_id"], "task_id": scene["task_id"], "variant_id": scene["variant_id"],
                         "status": payload["status"], "note": payload["note"], "revision": revision + 1,
                         "updated_at": iso(), "config_sha256": scene["config_sha256"],
                         "views": payload.get("views", [])}
                data["entries"] = sorted([r for r in data["entries"] if r["scene_id"] != entry["scene_id"]] + [entry],
                                         key=lambda r: r["scene_id"])
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix=".scene_annotations-",
                                                     suffix=".tmp", dir=self.annotation_path.parent, delete=False) as stream:
                        temporary = Path(stream.name)
                        json.dump(data, stream, ensure_ascii=False, indent=2)
                        stream.write("\n")
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(temporary, self.annotation_path)
                    directory = os.open(self.annotation_path.parent, os.O_RDONLY | os.O_DIRECTORY)
                    try:
                        os.fsync(directory)
                    finally:
                        os.close(directory)
                finally:
                    if temporary is not None:
                        temporary.unlink(missing_ok=True)
                return entry
        except OSError as exc:
            raise AnnotationError(503, "标注未确认保存成功；请保留当前输入并重试。") from exc

    @property
    def trajectory_annotation_path(self) -> Path:
        return self.root / "reports/replicator_dashboard/trajectory_annotations.json"

    @contextmanager
    def locked_trajectory_annotations(self, exclusive=False):
        with self.trajectory_annotation_lock:
            self.trajectory_annotation_path.parent.mkdir(parents=True, exist_ok=True)
            with self.trajectory_annotation_path.with_suffix(".lock").open("a+b") as lockfile:
                fcntl.flock(lockfile.fileno(), fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
                try:
                    yield
                finally:
                    fcntl.flock(lockfile.fileno(), fcntl.LOCK_UN)

    def read_trajectory_annotations_unlocked(self) -> dict:
        try:
            raw = self.trajectory_annotation_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return {"schema_version": 1, "entries": []}
        except (OSError, UnicodeError) as exc:
            raise AnnotationError(503, "无法读取轨迹标注；已停止保存以保护现有内容。") from exc
        try:
            data = json.loads(raw)
            if (not isinstance(data, dict) or type(data.get("schema_version")) is not int
                    or data["schema_version"] != 1 or not isinstance(data.get("entries"), list)):
                raise ValueError()
            seen = set()
            for row in data["entries"]:
                if (not isinstance(row, dict) or set(row) != {"scene_id", "task_id", "variant_id", "status", "note",
                        "revision", "updated_at", "config_sha256", "result_sha256"}
                        or not isinstance(row["scene_id"], str) or not re.fullmatch(r"[a-z0-9_]{1,160}", row["scene_id"])
                        or row["scene_id"] in seen
                        or not isinstance(row["task_id"], str) or not isinstance(row["variant_id"], str)
                        or row["scene_id"] != row["task_id"].removeprefix(PREFIX) + "__" + row["variant_id"]
                        or not isinstance(row["status"], str) or row["status"] not in ANNOTATION_STATUSES
                        or not isinstance(row["note"], str) or len(row["note"]) > ANNOTATION_MAX_NOTE
                        or type(row["revision"]) is not int or row["revision"] < 1
                        or not isinstance(row["updated_at"], str) or datetime.fromisoformat(row["updated_at"]).tzinfo is None
                        or not valid_digest(row["config_sha256"])
                        or row["result_sha256"] is not None and not valid_digest(row["result_sha256"])):
                    raise ValueError()
                seen.add(row["scene_id"])
            return data
        except (ValueError, KeyError, TypeError) as exc:
            raise AnnotationError(503, "轨迹标注文件格式异常；已停止保存以保护现有内容。") from exc

    def trajectory_annotations(self) -> dict:
        try:
            with self.locked_trajectory_annotations():
                return self.read_trajectory_annotations_unlocked()
        except OSError as exc:
            raise AnnotationError(503, "无法读取轨迹标注，请检查本地存储。") from exc

    def save_trajectory_annotation(self, payload: dict) -> dict:
        fields = {"scene_id", "status", "note", "expected_revision", "config_sha256", "result_sha256"}
        if (not isinstance(payload, dict) or set(payload) != fields
                or not isinstance(payload["scene_id"], str) or not re.fullmatch(r"[a-z0-9_]{1,160}", payload["scene_id"])
                or not isinstance(payload["status"], str) or payload["status"] not in ANNOTATION_STATUSES
                or not isinstance(payload["note"], str) or len(payload["note"]) > ANNOTATION_MAX_NOTE
                or type(payload["expected_revision"]) is not int or payload["expected_revision"] < 0
                or not valid_digest(payload["config_sha256"])
                or payload["result_sha256"] is not None and not valid_digest(payload["result_sha256"])):
            raise AnnotationError(400, "轨迹标注字段无效；调整方案最多 20000 字。")
        with self.lock:
            assignment = self.json(self.root / "student_skills/robolaw-replicator/references/assignment.json")
            tasks = {row["id"]: {} for row in assignment.get("tasks", [])
                     if isinstance(row, dict) and isinstance(row.get("id"), str)}
            scenes = self.visual_only_review(tasks)
            scene = next((row for row in self.trajectory_generation(scenes)["entries"]
                          if row["scene_id"] == payload["scene_id"]), None)
        if scene is None or not scene["config_present"]:
            raise AnnotationError(400, "场景不存在或配置无法验证，请刷新列表。")
        try:
            with self.locked_trajectory_annotations(exclusive=True):
                data = self.read_trajectory_annotations_unlocked()
                current = next((row for row in data["entries"] if row["scene_id"] == payload["scene_id"]), None)
                revision = current["revision"] if current else 0
                if payload["expected_revision"] != revision:
                    raise AnnotationError(409, "此轨迹标注已在其他页面更新，请比较最新内容后再保存。", current)
                if payload["config_sha256"] != scene["config_sha256"]:
                    raise AnnotationError(409, "场景配置已变化，请刷新后再标注轨迹。", current)
                if payload["result_sha256"] != scene["result_sha256"]:
                    raise AnnotationError(409, "轨迹结果已更新，请查看最新结果后再保存方案。", current)
                entry = {key: scene[key] for key in ("scene_id", "task_id", "variant_id", "config_sha256", "result_sha256")}
                entry.update(status=payload["status"], note=payload["note"], revision=revision + 1, updated_at=iso())
                data["entries"] = sorted([row for row in data["entries"] if row["scene_id"] != entry["scene_id"]] + [entry],
                                         key=lambda row: row["scene_id"])
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix=".trajectory_annotations-",
                            suffix=".tmp", dir=self.trajectory_annotation_path.parent, delete=False) as stream:
                        temporary = Path(stream.name)
                        json.dump(data, stream, ensure_ascii=False, indent=2)
                        stream.write("\n")
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(temporary, self.trajectory_annotation_path)
                    directory = os.open(self.trajectory_annotation_path.parent, os.O_RDONLY | os.O_DIRECTORY)
                    try:
                        os.fsync(directory)
                    finally:
                        os.close(directory)
                finally:
                    if temporary is not None:
                        temporary.unlink(missing_ok=True)
                return entry
        except OSError as exc:
            raise AnnotationError(503, "轨迹标注未确认保存成功，请保留当前输入并重试。") from exc

    def file_sha256(self, path: Path) -> str:
        stat = path.stat()
        key, stamp = str(path), (stat.st_mtime_ns, stat.st_size)
        if key not in self.hash_cache or self.hash_cache[key][0] != stamp:
            self.hash_cache[key] = (stamp, hashlib.sha256(path.read_bytes()).hexdigest())
        return self.hash_cache[key][1]

    def trajectory_lineage_matches(self, result, scene, roots):
        """Follow hash-bound changes back to the original visual scene config."""
        from kitchen_safety.extensions.replicator.scene_retry_once import validate_adjustment
        seen = set()
        try:
            while True:
                if any(result.get(k) != scene[k] for k in ('scene_id','task_id','variant_id')):
                    return False
                if result.get('config_sha256') == scene.get('config_sha256'):
                    return True
                path = self.local(result['prior_result_path'])
                allowed = {root+'scenes/'+scene['scene_id']+'/result.json' for root in roots}
                if path.relative_to(self.root).as_posix() not in allowed or path in seen:
                    return False
                seen.add(path)
                if self.file_sha256(path) != result.get('prior_result_sha256'):
                    return False
                previous = self.json(path)
                if result.get('config_sha256') != previous.get('config_sha256'):
                    change = validate_adjustment(self.root, dict(action='plan',layout_fixed=False,
                        scene_adjustment=result.get('scene_adjustment')), previous.get('config_path'), previous.get('config_sha256'))
                    if result.get('config_sha256') != change['config_sha256'] or result.get('config_path') != change['config_path']:
                        return False
                elif result.get('config_path') != previous.get('config_path'):
                    return False
                result = previous
        except (ValueError, OSError, KeyError, TypeError):
            return False

    def trajectory_bound_views(self, report_root, source, sid, config_sha):
        path = self.local(source.get('scene_views_path') or report_root+'scene_views.json')
        document = self.json(path)
        if not document:
            return []
        if source.get('scene_views_sha256'):
            bound = self.file_sha256(path) == source['scene_views_sha256']
        else:
            bound = (document.get('recipe_sha256') == source.get('recipe_sha256') and
                     document.get('recipe_path') == source.get('recipe_path'))
        if bound:
            row = next((e for e in document.get('entries',[]) if e.get('scene_id') == sid), {})
            if row.get('config_sha256') == config_sha:
                return row.get('views',[])
        return []

    def trajectory_generation(self, scenes: dict) -> dict:
        """Expose current-config planning output without granting execution acceptance."""
        report_roots = trajectory_report_roots(self.root)
        report_root = next((path for path in report_roots
                            if (self.root / (path + "progress.json")).is_file()), TRAJECTORY_REPORTS)
        retry_active = TRAJECTORY_DIRECTORY.fullmatch(Path(report_root.rstrip('/')).name)[1] != 'generation'
        progress_path = self.root / (report_root + "progress.json")
        source = self.json(progress_path)
        active_round = source.get("retry_round", 1) if retry_active else 0
        source_root = str(source.get("source_run", TRAJECTORY_REPORTS)).rstrip("/") + "/"
        if source_root not in report_roots or source_root == report_root:
            source_root = TRAJECTORY_REPORTS
        indexed, duplicates = {}, set()
        for row in source.get("entries", []) if isinstance(source.get("entries"), list) else []:
            if isinstance(row, dict) and isinstance(row.get("scene_id"), str):
                if row["scene_id"] in indexed:
                    duplicates.add(row["scene_id"])
                indexed[row["scene_id"]] = row
        entries = []
        for scene in scenes["entries"]:
            row = {key: scene.get(key) for key in ("scene_id", "task_id", "variant_id", "variant_name", "category",
                                                    "config_sha256", "config_present", "views")}
            row.update(label=scene.get("variant_name", scene["variant_id"]), status="pending", stage="pending", reason="",
                       failure_reason="", failure_code="", suggestion="", suggestion_scope="manual_adjustment_hint_not_diagnosis",
                       elapsed_seconds=None, timing={}, artifacts=[], paths={}, result_sha256=None,
                       updated_at=None, stale=False, planning_only=True, executed=False, task_success_verified=False,
                       checks_not_performed=["长期物理稳定性", "实际执行", "过程避障验证", "任务成功验证"])
            sid = scene["scene_id"]
            indexed_row = indexed.get(sid, {})
            retry_round = active_round if retry_active and indexed_row.get("retry_round") == active_round else 0
            carry_forward = retry_active and indexed_row.get("carry_forward") is True and not retry_round
            previous_folder = source_root + "scenes/" + sid + "/"
            previous_path = self.root / (previous_folder + "result.json")
            previous = self.json(previous_path) if retry_active else {}
            previous_valid = False
            if previous:
                try:
                    previous_valid = (self.trajectory_lineage_matches(previous, scene, report_roots)
                                      and indexed_row.get("prior_result_path") == previous_path.relative_to(self.root).as_posix()
                                      and valid_digest(indexed_row.get("prior_result_sha256"))
                                      and self.file_sha256(previous_path) == indexed_row["prior_result_sha256"])
                except OSError:
                    pass
            if previous_valid and previous.get('config_sha256') != scene.get('config_sha256'):
                scene = dict(scene, config_path=previous['config_path'],config_sha256=previous['config_sha256'],
                             views=self.trajectory_bound_views(source_root,self.json(self.root/(source_root+'progress.json')),
                                                               sid,previous['config_sha256']))
                row.update(config_sha256=scene['config_sha256'],views=scene['views'],
                           historical_scene_quality_issues=previous.get('scene_adjustment',{}).get('scene_quality_issues',[]))
            recipe = indexed_row.get("recipe") if isinstance(indexed_row.get("recipe"), dict) else {}
            scene_adjustment = indexed_row.get('scene_adjustment') if retry_round else None
            if scene_adjustment and previous_valid:
                try:
                    from kitchen_safety.extensions.replicator.scene_retry_once import validate_adjustment
                    change = validate_adjustment(self.root, recipe, previous.get('config_path'), previous.get('config_sha256'))
                    if change != scene_adjustment or any(indexed_row.get(k) != change[k] for k in ('config_path','config_sha256')):
                        raise ValueError('Adjusted scene binding mismatch')
                    scene = dict(scene, config_path=change['config_path'], config_sha256=change['config_sha256'],
                                 config_present=True, views=[])
                    scene['views'] = self.trajectory_bound_views(report_root,source,sid,change['config_sha256'])
                    row.update(config_sha256=scene['config_sha256'], config_present=True, views=scene['views'],
                               scene_adjustment=change, scene_settings_changed_once=True)
                except (ValueError, OSError, KeyError, TypeError):
                    previous_valid = False
            description = recipe.get("change_description") or recipe.get("summary") or recipe.get("description")
            if not isinstance(description, str):
                changes = recipe.get("changes", [])
                description = "；".join(item for item in changes if isinstance(item, str)) if isinstance(changes, list) else ""
            if carry_forward:
                description = ''
            row.update(retry_round=retry_round, carry_forward=carry_forward, recipe=recipe,
                       change_description=description, previous_status=previous.get("status") if previous_valid else None,
                       previous_stage=previous.get("stage") if previous_valid else None,
                       previous_failure_code=previous.get("failure_code") if previous_valid else None,
                       previous_result_sha256=indexed_row.get("prior_result_sha256") if previous_valid else None,
                       previous_result_url=None, previous_evidence_valid=previous_valid)
            if previous_valid:
                prior_artifact = self.artifact(previous_path, "上一轮结果（保留）")
                if prior_artifact:
                    row["previous_result_url"] = prior_artifact["url"]
            folder = previous_folder if carry_forward else report_root + "scenes/" + sid + "/"
            artifact_folders = tuple(path + "scenes/" + sid + "/" for path in report_roots) if carry_forward else (folder,)
            result_path = self.root / (folder + "result.json")
            result = self.json(result_path)
            raw = result or indexed.get(sid, {})
            row['annotation_only'] = raw.get('annotation_only') is True
            if raw:
                valid = (sid not in duplicates and scene.get("config_present") and valid_digest(scene.get("config_sha256"))
                         and all(raw.get(key) == scene[key] for key in ("scene_id", "task_id", "variant_id", "config_sha256"))
                         and raw.get("status") in TRAJECTORY_STATUSES
                         and (not retry_active or previous_valid and
                              (retry_round == active_round and previous.get("status") == "failed" or
                               carry_forward and previous.get("status") == "generated")))
                if not valid:
                    row.update(stale=True, reason="结果与当前场景配置不匹配，等待重新生成。", stage="stale")
                else:
                    terminal = raw["status"] in ("generated", "failed", "skipped")
                    row.update(status=raw["status"] if result or not terminal else "pending",
                               stage=str(raw.get("stage") or raw["status"]),
                               reason=str(raw.get("failure_reason") or raw.get("reason") or ""),
                               failure_reason=str(raw.get("failure_reason") or raw.get("reason") or ""),
                               failure_code=str(raw.get("failure_code") or ""),
                               suggestion=derive_suggestion(raw), updated_at=raw.get("updated_at"))
                    if terminal and not result:
                        row.update(reason="等待本场景结果文件落盘。", stage="awaiting_result")
                    if result:
                        try:
                            row["result_sha256"] = self.file_sha256(result_path)
                            report = self.artifact(result_path, "轨迹生成结果")
                            if report:
                                row["artifacts"].append(report)
                                row["paths"]["result"] = report["url"]
                        except OSError:
                            row.update(status="pending", stage="awaiting_result", result_sha256=None)
                    artifacts = raw.get("artifacts", {})
                    for name, evidence in artifacts.items() if isinstance(artifacts, dict) else []:
                        try:
                            if (not isinstance(name, str) or not isinstance(evidence, dict)
                                    or not isinstance(evidence.get("path"), str) or not valid_digest(evidence.get("sha256"))):
                                continue
                            path = self.local(evidence["path"])
                            if (not path.relative_to(self.root).as_posix().startswith(artifact_folders) or not path.is_file()
                                    or path.stat().st_size == 0 or self.file_sha256(path) != evidence["sha256"]):
                                continue
                            artifact = self.artifact(path, {"trajectory": "轨迹数据", "trajectory_json": "轨迹 JSON",
                                                           "preview": "轨迹示意图", "partial_trajectory": "失败前部分轨迹"}.get(name, name))
                            if artifact:
                                if path.suffix.lower() in (".png", ".jpg", ".jpeg"):
                                    artifact["kind"] = "image"
                                row["artifacts"].append(artifact)
                                row["paths"][name] = artifact["url"]
                        except (ValueError, OSError):
                            continue
                    if row["status"] == "generated" and (raw.get("planning_only") is not True
                            or "trajectory" not in row["paths"]):
                        row.update(status="pending", stage="awaiting_artifact", reason="完整轨迹数据缺失或来源已变化，暂不计为已生成。")
                    row["phases"] = raw.get("phases", []) if isinstance(raw.get("phases"), list) else []
                    row["timing"] = dict(raw["timing"]) if isinstance(raw.get("timing"), dict) else {}
                    elapsed = raw.get("elapsed_seconds", row["timing"].get("elapsed_seconds"))
                    if type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0:
                        row["elapsed_seconds"] = elapsed
                        row["timing"]["elapsed_seconds"] = elapsed
            preview = self.json(self.root / (folder + "preview.json"))
            trajectories = raw.get("artifacts", {}) if isinstance(raw, dict) else {}
            trajectory_hashes = {value.get("sha256") for key, value in trajectories.items()
                                 if isinstance(value, dict) and key in row["paths"]
                                 and key in ("trajectory", "trajectory_json", "partial_trajectory", "partial_trajectory_json")
                                 and valid_digest(value.get("sha256"))} if isinstance(trajectories, dict) else set()
            if (preview.get("scene_id") == sid and preview.get("config_sha256") == scene.get("config_sha256")
                    and row["result_sha256"] is not None and preview.get("result_sha256") == row["result_sha256"]
                    and (preview.get("trajectory_sha256") in trajectory_hashes
                         or preview.get("trajectory_sha256") is None and not trajectory_hashes)):
                for name, evidence in preview.get("artifacts", {}).items() if isinstance(preview.get("artifacts"), dict) else []:
                    try:
                        if name not in ("preview", "interactive") or not isinstance(evidence, dict) or not valid_digest(evidence.get("sha256")):
                            continue
                        path = self.local(evidence["path"])
                        if (not path.relative_to(self.root).as_posix().startswith(artifact_folders)
                                or path.suffix.lower() != (".html" if name == "interactive" else ".png")
                                or not path.is_file() or self.file_sha256(path) != evidence["sha256"]):
                            continue
                        artifact = self.artifact(path, "可交互轨迹投影" if name == "interactive" else "轨迹投影预览")
                        if artifact:
                            artifact["kind"] = "interactive" if name == "interactive" else "image"
                            row["artifacts"].append(artifact)
                            row["paths"][name] = artifact["url"]
                    except (KeyError, TypeError, ValueError, OSError):
                        continue
            row.update(trajectory_url=row["paths"].get("trajectory", row["paths"].get("trajectory_json")),
                       preview_url=row["paths"].get("preview"), video_url=row["paths"].get("video"),
                       interactive_url=row["paths"].get("interactive"))
            entries.append(row)
        counts = {name: sum(row["status"] == name for row in entries) for name in TRAJECTORY_STATUSES}
        counts.update(total=len(entries), stale=sum(row["stale"] for row in entries))
        retried = [row for row in entries if active_round and row["retry_round"] == active_round]
        round_counts = {name: sum(row["status"] == name for row in retried) for name in TRAJECTORY_STATUSES}
        round_counts.update(total=len(retried), stale=sum(row["stale"] for row in retried))
        round_actions = dict(source.get('round_actions', {}))
        annotation_errors = sum(row.get('annotation_only') and row['stage'] == 'retry_worker' for row in retried)
        if annotation_errors:
            round_actions['annotation_delivery_failed'] = annotation_errors
        return dict(total=len(entries), entries=entries, counts=counts, stage=source.get("stage", "preparing"),
                    updated_at=source.get("updated_at"), source_progress_path=progress_path.relative_to(self.root).as_posix(),
                    retry_round=active_round, round_counts=round_counts,
                    scene_settings_changed_once=source.get('scene_settings_changed_once') is True,
                    scene_adjustment_count=source.get('scene_adjustment_count', 0),
                    target_only_retry_count=source.get('target_only_retry_count', 0),
                    round_actions=round_actions,
                    carried_generated=sum(row["carry_forward"] and row["status"] == "generated" for row in entries),
                    planning_only=True, notice="每景只尝试一条 cuRobo 轨迹，失败即跳过；已生成仅表示规划完成，未验证物理稳定性、实际执行或过程避障。")

    def physical_execution(self) -> dict:
        """Separate physical attempt evidence from successful planning and notes."""
        source_root = self.root / PHYSICAL_SOURCE
        source_path = source_root / 'progress.json'
        source = self.json(source_path)
        planned = {row['scene_id']: row for row in source.get('entries', [])
                   if isinstance(row, dict) and isinstance(row.get('scene_id'), str)}
        manifest_path = self.root / PHYSICAL_DEMOS / 'manifest.json'
        manifest = self.json(manifest_path)
        indexed, duplicates = {}, set()
        for raw in manifest.get('entries', []) if isinstance(manifest.get('entries'), list) else []:
            if isinstance(raw, dict) and isinstance(raw.get('scene_id'), str):
                if raw['scene_id'] in indexed:
                    duplicates.add(raw['scene_id'])
                indexed[raw['scene_id']] = raw
        manifest_valid = False
        try:
            manifest_valid = (manifest.get('kind') == 'physical_execution'
                and self.local(manifest['source_run']) == source_root
                and self.file_sha256(source_path) == manifest.get('source_progress_sha256')
                and source.get('stage') == 'complete' and set(indexed) == set(planned)
                and not duplicates and len(manifest['entries']) == len(planned))
        except (KeyError, TypeError, ValueError, OSError):
            pass

        def bound(path, digest):
            return valid_digest(digest) and path.is_file() and path.stat().st_size > 0 and self.file_sha256(path) == digest

        def demo_local(value):
            value = str(value)
            path = self.local(PHYSICAL_DEMOS + value if value.startswith(('executions/', 'videos/', 'initialization_history/')) else value)
            path.relative_to(self.root / PHYSICAL_DEMOS)
            return path

        entries = []
        for sid, prior in planned.items():
            row = {key: prior.get(key) for key in ('scene_id', 'task_id', 'variant_id', 'variant_name', 'category', 'config_sha256')}
            row.update(planning_status=prior.get('status'), execution_status='pending', executed=None,
                task_success=None, attempts=0, stale=False, result_valid=False, reason='', result_url=None,
                video_url=None, video_available=False, initialization_error=False,
                additional_initialization_starts=0, historical_quality_issues=[])
            raw = indexed.get(sid, {})
            planning_path = source_root / 'scenes' / sid / 'result.json'
            planning = self.json(planning_path)
            row['historical_quality_issues'] = planning.get('scene_adjustment', {}).get('scene_quality_issues', [])
            if not raw and not manifest:
                entries.append(row)
                continue
            try:
                if not manifest_valid or sid in duplicates:
                    raise ValueError('物理执行清单未绑定当前规划来源。')
                if any(raw.get(key) != prior.get(key) for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256')):
                    raise ValueError('场景身份或配置与规划来源不一致。')
                if (prior.get('status') != 'generated' or raw.get('planning_status') != 'generated'
                        or self.local(raw['result_path']) != planning_path
                        or not bound(planning_path, raw.get('result_sha256'))):
                    raise ValueError('规划结果文件绑定无效。')
                config = self.local(raw['config_path'])
                trajectory = self.local(raw['trajectory_path'])
                artifact = planning.get('artifacts', {}).get('trajectory', {})
                if (config != self.local(prior['config_path']) or config != self.local(planning['config_path'])
                        or not bound(config, raw.get('config_sha256'))
                        or trajectory != self.local(artifact['path'])
                        or raw.get('trajectory_sha256') != artifact.get('sha256')
                        or not bound(trajectory, raw.get('trajectory_sha256'))):
                    raise ValueError('配置或完整规划轨迹的哈希已变化。')
                status = raw.get('execution_status', 'pending')
                if status not in PHYSICAL_STATUSES:
                    raise ValueError('未知物理执行状态。')
                if status in ('pending', 'running'):
                    row.update(execution_status=status, attempts=raw.get('attempts', 0),
                               reason='正在执行，等待保存实际结果。' if status == 'running' else '')
                    entries.append(row)
                    continue
                result_path = demo_local(raw['execution_result_path'])
                episode_dir = self.root / PHYSICAL_DEMOS / 'executions' / sid
                subdirectory = raw.get('execution_subdirectory', '')
                if subdirectory not in ('', 'initialization_retry_01'):
                    raise ValueError('不允许的物理执行恢复目录。')
                expected_result = episode_dir / subdirectory / 'result.json'
                if result_path != expected_result or not bound(result_path, raw.get('execution_result_sha256')):
                    raise ValueError('实际执行结果缺失或哈希未匹配，等待有效记录。')
                result = self.json(result_path)
                if result.get('kind') != 'physical_execution' or any(result.get(key) != raw.get(key) for key in (
                    'scene_id', 'task_id', 'variant_id', 'config_sha256', 'planning_status',
                    'execution_status', 'executed', 'task_success', 'attempts')):
                    raise ValueError('实际执行结果与清单不一致。')
                history = raw.get('initialization_history', [])
                if (not isinstance(history, list) or result.get('initialization_history', []) != history
                        or result.get('execution_subdirectory', '') != subdirectory
                        or bool(history) != bool(subdirectory) or len(history) > 1):
                    raise ValueError('初始化恢复目录与双方历史记录不一致。')
                if history:
                    item = history[0]
                    event_dir = self.root / PHYSICAL_DEMOS / 'initialization_history' / sid / 'initialization_01'
                    if (not isinstance(item, dict) or item.get('kind') != 'initialization_interruption'
                            or item.get('execution_started') is not False
                            or demo_local(item['prior_result_path']) != episode_dir/'result.json'
                            or demo_local(item['recovery_event_path']) != event_dir/'recovery_event.json'
                            or not bound(episode_dir/'result.json', item.get('prior_result_sha256'))
                            or not bound(event_dir/'recovery_event.json', item.get('recovery_event_sha256'))):
                        raise ValueError('旧初始化结果或恢复事件的路径/SHA 无效。')
                    old = self.json(episode_dir/'result.json')
                    event = self.json(event_dir/'recovery_event.json')
                    if (old.get('scene_id') != sid or old.get('status') != 'initializing'
                            or old.get('executed') is not False or old.get('task_success') is not False
                            or old.get('action_count', 0) != 0 or old.get('video_path') or old.get('video_sha256')
                            or event.get('scene_id') != sid or event.get('kind') != 'initialization_interruption'
                            or event.get('execution_started') is not False
                            or event.get('prior_process_confirmed_stopped') is not True
                            or event.get('no_actuator_commands_verified') is not True
                            or event.get('prior_result', {}).get('sha256') != item['prior_result_sha256']
                            or demo_local(event['prior_result']['path']) != episode_dir/'result.json'):
                        raise ValueError('旧记录不足以证明物理 episode 尚未开始。')
                    for key, raw_key in (('config_sha256', 'config_sha256'),
                                         ('planning_result_sha256', 'result_sha256'),
                                         ('trajectory_sha256', 'trajectory_sha256')):
                        if old.get(key) != raw.get(raw_key):
                            raise ValueError('初始化历史与当前规划输入不一致。')
                    for key in ('prior_source', 'prior_log', 'prior_manifest', 'operator_stop', 'diagnosis'):
                        ref = event[key]
                        if not bound(demo_local(ref['path']), ref.get('sha256')):
                            raise ValueError('初始化恢复的历史证据 SHA 无效：' + key)
                    if (old.get('execution_source_sha256_start', {}).get('kitchen_safety/extensions/replicator/planned_execution.py')
                            != event['prior_source']['sha256']):
                        raise ValueError('旧初始化记录未绑定保留的执行源码。')
                    row['additional_initialization_starts'] = 1
                if (self.local(result['planning_result_path']) != planning_path
                        or result.get('planning_result_sha256') != raw['result_sha256']
                        or self.local(result['config_path']) != config
                        or self.local(result['trajectory_path']) != trajectory
                        or result.get('trajectory_sha256') != raw['trajectory_sha256']):
                    raise ValueError('实际执行结果未绑定相同的规划输入。')
                if (type(result.get('executed')) is not bool or type(result.get('task_success')) is not bool
                        or type(result.get('attempts')) is not int or result['attempts'] != 1
                        or result['task_success'] is not (status == 'success')
                        or not result['executed'] and status != 'error'):
                    raise ValueError('物理执行状态、任务判据或尝试次数矛盾。')
                row.update(execution_status=status, executed=result['executed'], task_success=result['task_success'],
                    attempts=result['attempts'], result_valid=True, result_sha256=raw['execution_result_sha256'],
                    initialization_error=status == 'error' and not result['executed'],
                    reason=str(result.get('failure_reason') or result.get('error') or ''),
                    result_url='/artifact/' + quote(result_path.relative_to(self.root).as_posix(), safe='/'))
                if raw.get('video_path') and result.get('video_path'):
                    video = demo_local(raw['video_path'])
                    if (video == self.root / PHYSICAL_DEMOS / 'videos' / (sid + '.mp4')
                            and demo_local(result['video_path']) == video
                            and result.get('video_sha256') == raw.get('video_sha256')
                            and bound(video, raw.get('video_sha256'))):
                        row.update(video_url='/artifact/' + quote(video.relative_to(self.root).as_posix(), safe='/'),
                                   video_available=True, video_sha256=raw['video_sha256'])
                if not row['video_available']:
                    row['video_note'] = '暂无哈希匹配的实际现场视频；未用规划预览代替。'
            except (ValueError, OSError, KeyError, TypeError) as error:
                row.update(stale=True, execution_status='pending', executed=None, task_success=None,
                           result_valid=False, result_url=None, video_url=None, video_available=False,
                           initialization_error=False, additional_initialization_starts=0, reason=str(error))
            entries.append(row)
        counts = {state: sum(row['execution_status'] == state for row in entries) for state in PHYSICAL_STATUSES}
        counts.update(planned=sum(row['planning_status'] == 'generated' for row in entries), total=len(entries),
                      video_count=sum(row['video_available'] for row in entries), stale=sum(row['stale'] for row in entries),
                      initialization_error=sum(row['initialization_error'] for row in entries))
        retakes = self.camera_retakes(entries, indexed, source_path)
        retry = self.physical_retries(entries, indexed)
        retry2 = self.physical_retries(entries, indexed, retry_round=2, previous_retry=retry)
        retry3 = self.physical_retries(entries, indexed, retry_round=3, previous_retry=retry2)
        retry4 = self.physical_retries(entries, indexed, retry_round=4, previous_retry=retry3)
        retry5 = self.physical_retries(entries, indexed, retry_round=5, previous_retry=retry4)
        retry6 = self.physical_retries(entries, indexed, retry_round=6, previous_retry=retry5)
        retry7 = self.physical_retries(entries, indexed, retry_round=7, previous_retry=retry6)
        retry8 = self.physical_retries(entries, indexed, retry_round=8, previous_retry=retry7)
        promotions=retry['counts']['success']+retry2['counts']['success']+retry3['counts']['success']+retry4['counts']['success']+retry5['counts']['success']+retry6['counts']['success']
        executed_errors=sum(r.get('result_valid') and r.get('executed') and r.get('execution_status')=='error'
                            for group in (retry,retry2,retry3,retry4,retry5,retry6) for r in group['entries'])
        latest=dict(total=counts['total'],success=counts['success']+promotions,
                    failed=counts['failed']-promotions-executed_errors,error=counts['error']+executed_errors)
        revision_errors=sum(r.get('result_valid') and r.get('executed') and r.get('execution_status')=='error' for group in (retry7,retry8) for r in group['entries'])
        current_revision=dict(total=latest['total'],success=latest['success']+retry7['counts']['success']+retry8['counts']['success'],
            failed=latest['failed']-retry7['counts']['success']-retry8['counts']['success']-revision_errors,error=latest['error']+revision_errors,
            revised_scenes=retry7['total'],revision_success=retry7['counts']['success'],
            revision_success_by_round={'7':retry7['counts']['success'],'8':retry8['counts']['success']})
        return dict(kind='physical_execution', manifest_present=bool(manifest), manifest_valid=manifest_valid,
                    manifest_path=PHYSICAL_DEMOS + 'manifest.json', counts=counts, entries=entries,
                    updated_at=manifest.get('updated_at'), stage=manifest.get('stage', manifest.get('status', 'waiting')),
                    camera_retakes=retakes, failure_retry=retry, failure_retry_round2=retry2, failure_retry_round3=retry3, failure_retry_round4=retry4, failure_retry_round5=retry5, failure_retry_round6=retry6, failure_retry_round7=retry7, failure_retry_round8=retry8, combined_latest_counts=latest, current_revision_counts=current_revision,
                    notice='首轮每景一次物理仿真，成功、失败和错误分别记录；相机补录独立展示，不改变首轮成绩。安全及过程避障未单独验收。')

    def physical_retry2_planning(self, previous_retry, retry_round=2):
        """Show preparation independently of execution; completed artifacts need SHA bindings."""
        document=self.json(self.root/({2:PHYSICAL_RETRY2_PLANNING,3:PHYSICAL_RETRY3_PLANNING,4:PHYSICAL_RETRY4_PLANNING,5:PHYSICAL_RETRY5_PLANNING,6:PHYSICAL_RETRY6_PLANNING,7:PHYSICAL_RETRY7_PLANNING,8:PHYSICAL_RETRY8_PLANNING}[retry_round])/'progress.json')
        data=dict(present=bool(document),entries=[],counts={s:0 for s in ('pending','running','generated','failed','stale')})
        if not document:return data
        try:
            ref=document['source_previous_retry_manifest'];path=self.local(ref['path'])
            if (path!=self.root/retry_paths(retry_round-1)['demos']/'manifest.json' or not valid_digest(ref.get('sha256'))
                    or self.file_sha256(path)!=ref['sha256']):raise ValueError('本轮规划的上一轮清单来源待核。')
            old={r['scene_id']:r for r in self.json(path)['entries'] if r['execution_status'] in (('failed','planning_failed') if retry_round>=3 else ('failed',))}
            valid={r['scene_id']:r for r in (previous_retry or {}).get('entries',[]) if not r.get('stale') and (r.get('result_valid') or r['execution_status']=='planning_failed') and r['execution_status'] in (('failed','planning_failed') if retry_round>=3 else ('failed',))}
            rows=document['entries'];ids={r['scene_id'] for r in rows}
            if len(rows)!=retry_paths(retry_round)['total'] or len(ids)!=retry_paths(retry_round)['total'] or ids!=set(old) or ids!=set(valid):
                raise ValueError('本轮规划必须对应上一轮已核对的全部失败项。')
        except (ValueError,OSError,KeyError,TypeError) as error:
            return dict(data,error=str(error),counts=dict(data['counts'],stale=retry_paths(retry_round)['total']))
        for raw in rows:
            sid=raw['scene_id'];source=old[sid];status=raw.get('status','pending')
            row=dict(scene_id=sid,task_id=source['task_id'],variant_id=source['variant_id'],config_sha256=source['config_sha256'],
                     planning_status=status,execution_status='planning_pending',executed=False,task_success=False,attempts=0,
                     physical_attempt_number=source['physical_attempt_number']+int(source.get('executed',False)),result_valid=False,result_url=None,result_sha256=None,
                     planning_result_url=None,stale=False,reason=str(raw.get('failure_reason') or ''),diagnostics=[],
                     diagnostic_report_url=None,diagnostic_error='',adjustment_description=raw.get('recipe',{}).get('description',''))
            try:
                if status not in ('pending','running','generated','failed'):raise ValueError('未知本轮规划状态。')
                if retry_round>=7:
                    revision_ref=raw.get('scene_revision') or raw.get('recipe',{}).get('scene_revision')
                    if revision_ref is None and status=='pending' and raw.get('attempts',0)==0 and not raw.get('candidate_planning_result'):
                        row.update(layout_revision_label=f'场景修订 r{retry_round} · 方案准备中',revision_preparation_pending=True)
                    else:
                        revised=(validate_scene_revision8 if retry_round==8 else validate_scene_revision)(self.root,revision_ref,expected_scene_id=sid,digest=self.file_sha256)
                        revision=revised['document']
                        source=dict(source,config_path=revision['new_config']['path'],config_sha256=revision['new_config']['sha256'])
                        row.update(config_sha256=source['config_sha256'],scene_revision=revision_ref,layout_changed=True,
                            layout_revision_label=f'场景修订 r{retry_round}',original_config_sha256=revision['old_config']['sha256'])
                config=self.local(source['config_path'])
                if self.file_sha256(config)!=source['config_sha256']:raise ValueError('本轮规划的冻结布局已变化。')
                if status in ('generated','failed'):
                    ref=raw['candidate_planning_result'];path=self.local(ref['path'])
                    if (not retry2_candidate_path(self.root,path,sid,retry_round)
                            or not valid_digest(ref.get('sha256')) or self.file_sha256(path)!=ref['sha256']):
                        raise ValueError('本轮规划结果来源 SHA 待核。')
                    result=self.json(path)
                    if retry_round>=7 and (result.get('scene_revision')!=revision_ref or result.get('layout_changed') is not True):
                        raise ValueError('本轮规划结果未绑定修订布局。')
                    if (any(result.get(k)!=source[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256'))
                            or result.get('status')!=status or result.get('planning_only') is not True
                            or result.get('executed') is not False or result.get('attempts')!=1
                            or result.get('trajectory_complete') is not (status=='generated')):
                        raise ValueError('本轮规划身份、次数或完成状态不一致。')
                    if status=='generated':
                        for artifact in (result['artifacts']['trajectory'],raw['trajectory']):
                            if self.file_sha256(self.local(artifact['path']))!=artifact['sha256']:raise ValueError('新规划轨迹 SHA 待核。')
                    row['planning_result_url']='/artifact/'+quote(path.relative_to(self.root).as_posix(),safe='/')
                row['execution_status']={'pending':'planning_pending','running':'planning_running','generated':'pending','failed':'planning_failed'}[status]
                data['counts'][status]+=1
            except (ValueError,OSError,KeyError,TypeError) as error:
                row.update(stale=True,reason=str(error));data['counts']['stale']+=1
            data['entries'].append(row)
        return dict(data,updated_at=document.get('updated_at'),stage=document.get('stage','planning'))

    def physical_retries(self, first_entries, first_raw, *, retry_round=1, previous_retry=None):
        """One explicit failure-retry overlay; first-pass and camera counts stay immutable."""
        paths=retry_paths(retry_round); field=f'failure_retry_round{retry_round}' if retry_round>=2 else 'failure_retry'
        planning=self.physical_retry2_planning(previous_retry,retry_round) if retry_round>=2 else {}
        manifest = self.json(self.root / paths['demos'] / 'manifest.json')
        empty = dict(manifest_present=bool(manifest), total=paths['total'], entries=[],retry_round=retry_round,planning=planning,planning_present=bool(planning.get('present')),
                     counts={k:0 for k in (*PHYSICAL_STATUSES,'planning_failed','stale','diagnostics_ready')})
        if not manifest:
            rows=planning.get('entries',[])
            by_id={r['scene_id']:r for r in first_entries}
            for row in rows:
                if row['scene_id'] in by_id:by_id[row['scene_id']][field]=row
            return dict(empty,entries=rows,counts=dict(empty['counts'],pending=sum(r['execution_status']=='pending' for r in rows),
                planning_failed=sum(r['execution_status']=='planning_failed' for r in rows),stale=sum(r['stale'] for r in rows)))
        first = {r['scene_id']:r for r in first_entries}
        def require(value, message):
            if not value: raise ValueError(message)
        def bound(value, digest):
            path=self.local(value)
            require(valid_digest(digest) and path.is_file() and self.file_sha256(path)==digest,'重试来源 SHA 不匹配：'+str(value))
            return path
        def prior_path(raw, field='execution_result_path', camera=False):
            value=raw[field]
            return self.local((CAMERA_RETAKE_DEMOS if camera else PHYSICAL_DEMOS)+value if value.startswith('executions/') else value)
        try:
            plan_path=bound(manifest['retry_plan_path'],manifest['retry_plan_sha256']);plan=self.json(plan_path)
            require(plan_path==self.root/paths['plan'] and plan.get('kind')=='physical_failure_retry_plan','重试计划类型或路径错误。')
            require(manifest.get('kind')=='physical_failure_retry' and manifest.get('rendering_enabled') is False
                    and manifest.get('video_expected') is False,'本轮必须为无渲染物理测试。')
            for key,prefix in [('source_first_pass_manifest',PHYSICAL_DEMOS),('source_camera_retake_manifest',CAMERA_RETAKE_DEMOS)]:
                ref=manifest[key];require(ref==plan[key],'重试清单来源与冻结计划不一致。')
                require(bound(ref['path'],ref['sha256'])==self.root/prefix/'manifest.json','重试源清单路径错误。')
            raw_rows=manifest['entries'];plan_rows=plan['entries']
            if isinstance(plan_rows,dict):plan_rows=list(plan_rows.values())
            planned={r['scene_id']:r for r in plan_rows};indexed={r['scene_id']:r for r in raw_rows}
            failures={sid for sid,r in first_raw.items() if r.get('execution_status')=='failed'}
            previous_raw={};previous_valid={}
            if retry_round>=2:
                require(plan.get('schema_version')==2 and plan.get('retry_round')==retry_round
                        and manifest.get('schema_version')==2 and manifest.get('retry_round')==retry_round,'本轮 schema/轮次错误。')
                ref=plan['source_previous_retry_manifest']
                require(ref==manifest.get('source_previous_retry_manifest') and bound(ref['path'],ref['sha256'])==self.root/retry_paths(retry_round-1)['demos']/'manifest.json','上一轮结果清单绑定错误。')
                previous_raw={r['scene_id']:r for r in self.json(self.local(ref['path']))['entries']}
                failures={sid for sid,r in previous_raw.items() if r.get('execution_status') in (('failed','planning_failed') if retry_round>=3 else ('failed',))}
                previous_valid={r['scene_id']:r for r in (previous_retry or {}).get('entries',[])}
            require(len(raw_rows)==len(plan_rows)==paths['total'] and len(indexed)==len(planned)==paths['total']
                    and set(indexed)==set(planned)==failures,'重试白名单必须恰好覆盖首轮失败项。')
            cameras={r['scene_id']:r for r in self.json(self.root/CAMERA_RETAKE_DEMOS/'manifest.json').get('entries',[])}
        except (ValueError,OSError,KeyError,TypeError) as error:
            return dict(empty,manifest_error=str(error),counts=dict(empty['counts'],stale=paths['total']))
        entries=[]
        for sid,raw in indexed.items():
            row=dict(scene_id=sid,task_id=raw.get('task_id'),variant_id=raw.get('variant_id'),config_sha256=raw.get('config_sha256'),
                execution_status='pending',executed=False,task_success=False,attempts=0,result_valid=False,result_url=None,
                result_sha256=None,stale=False,reason='',physical_attempt_number=raw.get('physical_attempt_number'),
                diagnostics=[],diagnostic_report_url=None,diagnostic_error='',adjustment_description=raw.get('adjustment',{}).get('description',''))
            try:
                require(sid in first and first[sid]['result_valid'] and first[sid]['execution_status']=='failed','首轮真实失败来源待核。')
                require(all(raw.get(k)==v for k,v in planned[sid].items()),'重试条目与冻结计划不一致。')
                require(raw.get('retry_plan_sha256')==manifest['retry_plan_sha256'] and self.local(raw['retry_plan_path'])==plan_path,'条目重试计划绑定错误。')
                if retry_round>=7:
                    revised=(validate_scene_revision8 if retry_round==8 else validate_scene_revision)(self.root,raw.get('scene_revision'),expected_scene_id=sid,current=raw,digest=self.file_sha256)
                    row.update(scene_revision=raw['scene_revision'],layout_changed=True,layout_revision_label=f'场景修订 r{retry_round}',
                        original_config_sha256=revised['document']['old_config']['sha256'])
                for key in (('scene_id','task_id','variant_id') if retry_round>=7 else ('scene_id','task_id','variant_id','config_sha256')):
                    require(raw.get(key)==first_raw[sid].get(key),'重试改变原场景身份或配置。')
                for name in ('config','trajectory','source_trajectory','result','source_scene'):
                    if retry_round>=2 and name=='trajectory' and raw.get('planning_status')=='failed':continue
                    bound(raw[name+'_path'],raw[name+'_sha256'])
                require(raw['source_trajectory_sha256']==first_raw[sid]['trajectory_sha256'],'原轨迹 SHA 不一致。')
                history=raw['prior_physical_attempts'];previous=[first_raw[sid]]+([cameras[sid]] if sid in cameras else [])
                if retry_round>=2:
                    prior_view=previous_valid.get(sid,{})
                    require(not prior_view.get('stale') and (prior_view.get('result_valid') or retry_round>=3 and prior_view.get('execution_status')=='planning_failed') and prior_view.get('execution_status') in ('failed','planning_failed'),'上一轮失败来源待核。')
                    ref=raw['previous_retry_result'];old=previous_raw[sid]
                    if retry_round>=3:
                        previous=[dict(execution_result_path=h['result_path'],execution_result_sha256=h['result_sha256']) for h in old['prior_physical_attempts']]
                        if old['executed']:previous.append(old)
                    else:previous.append(old)
                    last=previous[-1]
                    require(ref==dict(path=last['execution_result_path'],sha256=last['execution_result_sha256']),'上一轮实际结果绑定错误。')
                    ref=raw['candidate_planning_result'];candidate_path=bound(ref['path'],ref['sha256']);candidate=self.json(candidate_path)
                    require(retry2_candidate_path(self.root,candidate_path,sid,retry_round),'新规划结果路径错误。')
                    require(all(candidate.get(k)==raw.get(k) for k in ('scene_id','task_id','variant_id','config_path','config_sha256'))
                            and candidate.get('status')==raw.get('planning_status') and candidate.get('planning_only') is True
                            and candidate.get('executed') is False and candidate.get('attempts')==1,'新规划结果身份或状态错误。')
                    if raw.get('planning_status')=='generated':
                        require(candidate.get('trajectory_complete') is True,'新轨迹尚未完整生成。')
                        ref=candidate['artifacts']['trajectory'];bound(ref['path'],ref['sha256'])
                    if retry_round>=7:
                        require(candidate.get('scene_revision')==raw['scene_revision'] and candidate.get('layout_changed') is True,'本轮规划未绑定修订布局。')
                    row.update(planning_status=raw['planning_status'],candidate_planning_result=raw['candidate_planning_result'],
                        planning_result_url='/artifact/'+quote(candidate_path.relative_to(self.root).as_posix(),safe='/'))
                require(len(history)==len(previous) and raw['physical_attempt_number']==len(history)+1,'物理尝试编号未计入历史补录。')
                for i,(ref,previous_row) in enumerate(zip(history,previous)):
                    path=bound(ref['result_path'],ref['result_sha256']);actual=self.json(path)
                    require(path==prior_path(previous_row,camera=i>0) and ref['result_sha256']==previous_row['execution_result_sha256'],'先前物理结果绑定错误。')
                    require(actual.get('scene_id')==sid and actual.get('executed') is True and actual.get('execution_status')=='failed'
                            and actual.get('task_success') is False and actual.get('physical_attempt_number',1)==i+1,'先前物理 episode 不是已确认失败。')
                    for key in ('execution_status','task_success','executed','video_sha256'):
                        require(ref.get(key)==actual.get(key),'先前物理状态不一致。')
                status=raw.get('execution_status');require(status in (*PHYSICAL_STATUSES,'planning_failed'),'未知重试状态。')
                row.update(execution_status=status,attempts=raw.get('attempts',0))
                if status=='planning_failed':
                    require(retry_round>=2 and raw.get('planning_status')=='failed' and raw.get('attempts')==0
                            and raw.get('executed') is False and raw.get('task_success') is False
                            and candidate.get('trajectory_complete') is False and raw.get('trajectory_path') is None,'规划失败不得计为物理尝试。')
                    row['reason']=str(raw.get('failure_reason') or candidate.get('failure_reason') or '')
                    entries.append(row);first[sid][field]=row;continue
                if status in ('pending','running'):
                    require(raw.get('attempts')==(1 if status=='running' else 0),'等待/运行状态的物理尝试数错误。')
                    entries.append(row);first[sid][field]=row;continue
                path=bound(raw['execution_result_path'],raw['execution_result_sha256'])
                require(path==self.root/paths['demos']/'executions'/sid/'result.json','重试结果路径错误。')
                result=self.json(path)
                require(result.get('kind')=='physical_failure_retry_execution' and result.get('status')=='complete','重试实际结果类型错误。')
                if retry_round>=2:
                    require(all(result.get(k)==raw.get(k) for k in ('candidate_planning_result','previous_retry_result','execution_overrides','retry_schema_version')),'本轮实际执行候选绑定错误。')
                    require(result.get('source_previous_retry_manifest')==manifest['source_previous_retry_manifest']
                            and result.get('source_inputs_unchanged') is True and result.get('execution_sources_unchanged') is True
                            and (not result.get('executed') or result.get('render_generation_delta')==0
                                and result.get('render_state_start',{}).get('passed') is True
                                and result.get('render_state_end',{}).get('passed') is True),'本轮执行来源或无渲染门禁错误。')
                for key in ('scene_id','task_id','variant_id','config_path','config_sha256','trajectory_path','trajectory_sha256',
                            'source_trajectory_path','source_trajectory_sha256','physical_attempt_number','prior_physical_attempts',
                            'retry_plan_path','retry_plan_sha256','adjustment','execution_status','task_success','executed','attempts'):
                    require(result.get(key)==raw.get(key),'重试实际结果/清单不一致：'+key)
                if retry_round>=7:
                    require(result.get('scene_revision')==raw['scene_revision'] and result.get('layout_changed') is True,'实际执行未绑定修订布局。')
                planning_ref=raw['candidate_planning_result'] if retry_round>=2 else dict(path=raw['result_path'],sha256=raw['result_sha256'])
                require(result.get('planning_result_sha256')==planning_ref['sha256'] and self.local(result['planning_result_path'])==self.local(planning_ref['path']), '重试规划来源不一致。')
                if retry_round>=2:
                    require(result.get('original_planning_result')==dict(path=raw['result_path'],sha256=raw['result_sha256']),'原始规划历史绑定错误。')
                require(type(result.get('executed')) is bool and type(result.get('task_success')) is bool
                        and type(result.get('attempts')) is int and result['attempts']==1
                        and result['task_success'] is (status=='success') and (result['executed'] or status=='error')
                        and result.get('rendering_enabled') is False and result.get('video_expected') is False,'重试次数、判据或无渲染标记矛盾。')
                if status=='success':
                    require(result.get('original_task_success') is True and result.get('final_original_destination_predicate') is True,'成功缺少原始任务判据。')
                if result['executed']:
                    evidence=result['physics_evidence']
                    for name in ('q','metadata'):bound(evidence[name+'_path'],evidence[name+'_sha256'])
                    if status=='success':require(evidence.get('complete') is True and evidence.get('continuous') is True,'成功缺少完整物理状态。')
                row.update(executed=result['executed'],task_success=result['task_success'],result_valid=True,
                    result_url='/artifact/'+quote(path.relative_to(self.root).as_posix(),safe='/'),result_sha256=raw['execution_result_sha256'],
                    reason=str(result.get('failure_reason') or ''),initialization_error=status=='error' and not result['executed'])
                if status in ('failed','error') and result['executed']:
                    report_path=self.root/paths['diagnostics']/'scenes'/sid/'diagnostic.json'
                    report=self.json(report_path)
                    diagnostic_progress=self.json(self.root/paths['diagnostics']/'progress.json')
                    diagnostic_entry=next((e for e in diagnostic_progress.get('entries',[]) if e.get('scene_id')==sid),{})
                    if report:
                        try:
                            require(diagnostic_entry.get('status')=='ready' and diagnostic_entry.get('report_path')==report_path.relative_to(self.root).as_posix()
                                    and bound(diagnostic_entry['report_path'],diagnostic_entry['report_sha256'])==report_path,'诊断报告绑定无效。')
                            require(report.get('kind')=='physical_failure_diagnostic_state_replay' and report.get('scene_id')==sid
                                    and report.get('diagnostic_only') is True and report.get('physics_steps_requested')==0
                                    and report.get('physical_episode_started') is False,'诊断必须是零执行的状态回放。')
                            binding=report['binding'];evidence=result['physics_evidence']
                            if retry_round>=7:
                                require(binding.get('scene_revision')==raw['scene_revision'] and binding.get('layout_changed') is True
                                        and binding.get('camera_reference_scope')=='historical_camera_pose_only','诊断未绑定本轮场景修订。')
                                camera_config=revised.get('historical_camera_config',revised['document']['old_config'])
                                if retry_round==8:
                                    require(binding.get('historical_camera_config')==camera_config,'诊断沿用相机的历史配置错误。')
                                observation=dict(path=binding.get('observation_reference_path'),sha256=binding.get('observation_reference_sha256'))
                                require(any(h['result_path']==observation['path'] and h['result_sha256']==observation['sha256'] for h in raw['prior_physical_attempts']),
                                        '诊断相机不是本景真实历史记录。')
                                camera_record=self.json(bound(observation['path'],observation['sha256']))
                                require(camera_record.get('scene_id')==sid and camera_record.get('config_sha256')==camera_config['sha256']
                                        and (camera_record.get('overview') or camera_record.get('camera_override',{}).get('applied')),'诊断相机历史来源待核。')
                            for name,key in [('execution_result','execution_result'),('config','config')]:
                                require(binding[name+'_sha256']==raw[key+'_sha256'],'诊断未绑定本轮结果/config。')
                            for name,key in [('physics_states','q'),('physics_metadata','metadata')]:
                                require(binding[name+'_sha256']==evidence[key+'_sha256'] and bound(binding[name+'_path'],binding[name+'_sha256'])==self.local(evidence[key+'_path']),'诊断实测状态来源错误。')
                            require(binding.get('physical_overlay_sha256')==result['physical_overlay_binding']['sha256_start'],'诊断物理背景 overlay 不匹配。')
                            bound(binding['physical_overlay_path'],binding['physical_overlay_sha256'])
                            frames=report['frames'];require(1<=len(frames)<=10,'诊断图片数量错误。')
                            images=[]
                            for frame in frames:
                                image=bound(frame['image_path'],frame['image_sha256']);require(image.parent==report_path.parent and re.fullmatch(r'frame_[0-9]{2}\.jpg',image.name),'诊断图片路径错误。')
                                images.append(dict(frame,url='/artifact/'+quote(image.relative_to(self.root).as_posix(),safe='/')))
                            row.update(diagnostics=images,diagnostic_report_url='/artifact/'+quote(report_path.relative_to(self.root).as_posix(),safe='/'))
                        except (ValueError,OSError,KeyError,TypeError) as error:row['diagnostic_error']=str(error)
            except (ValueError,OSError,KeyError,TypeError) as error:
                row.update(stale=True,result_valid=False,result_url=None,execution_status='pending',reason=str(error))
            entries.append(row)
            if sid in first:first[sid][field]=row
        counts={status:sum(r['execution_status']==status for r in entries) for status in (*PHYSICAL_STATUSES,'planning_failed')}
        counts.update(stale=sum(r['stale'] for r in entries),diagnostics_ready=sum(bool(r['diagnostics']) for r in entries))
        self.physical_retry_visual_reviews(manifest,entries,retry_round=retry_round)
        try:notes=self.physical_retry_annotations(retry_round)
        except AnnotationError as error:notes=dict(entries=[],error=error.message)
        return dict(empty,entries=entries,counts=counts,updated_at=manifest.get('updated_at'),stage=manifest.get('stage'),annotations=notes)

    def physical_retry_visual_reviews(self, manifest, entries, *, retry_round=1):
        """Bound published image/CPU findings, separate from user-owned annotations."""
        paths=retry_paths(retry_round)
        if manifest.get('stage') != 'complete': return
        publication=self.json(self.root/paths['results'])
        if not publication:return  # Expected while execution/render/review is pending.
        def bound(ref):
            if not isinstance(ref,dict) or not valid_digest(ref.get('sha256')):raise ValueError('Invalid review source')
            path=self.local(ref['path'])
            if not path.is_file() or self.file_sha256(path)!=ref['sha256']:raise ValueError('Review source changed')
            return path
        try:
            source=publication['source_retry_manifest']
            if (publication.get('kind')!='physical_failure_retry_result_snapshot'
                    or publication.get('partial_preview') is True
                    or bound(source)!=self.root/paths['demos']/'manifest.json'
                    or publication.get('diagnostic_visual_review',{}).get('status')!='complete'
                    or publication['diagnostic_visual_review'].get('conclusions_written') is not True):return
            published=publication['entries'];indexed={item['scene_id']:item for item in published}
            if len(indexed)!=len(published):return
            report_url=None
            if publication.get('markdown_report_path')==paths['report'] and (self.root/paths['report']).is_file():
                report_url='/artifact/'+quote(paths['report'],safe='/')
        except (ValueError,OSError,KeyError,TypeError):return
        for row in entries:
            if retry_round>=2 and row.get('execution_status')=='planning_failed' and not row.get('stale'):
                try:
                    retry=indexed[row['scene_id']][f'retry_round{retry_round}']
                    raw=next(r for r in manifest['entries'] if r['scene_id']==row['scene_id'])
                    diagnostic=retry['planning_failure_diagnostic']
                    ref=diagnostic['candidate_planning_result'];candidate_path=bound(ref)
                    if (any(r.get('execution_status')!='planning_failed' or r.get('planning_status')!='failed'
                            or type(r.get('attempts')) is not int or r['attempts']!=0 or r.get('executed') is not False
                            for r in (row,raw,retry))
                            or ref!=raw['candidate_planning_result'] or ref!=row['candidate_planning_result']
                            or ref!=retry['candidate_planning_result']
                            or row['planning_result_url']!='/artifact/'+quote(candidate_path.relative_to(self.root).as_posix(),safe='/')
                            or diagnostic.get('scene_id')!=row['scene_id'] or diagnostic.get('status')!='diagnosed'
                            or diagnostic.get('root_cause_confirmed') is not False
                            or diagnostic.get('scene_infeasibility_established') is not False):continue
                    evidence=diagnostic['evidence'];evidence_path=bound(evidence);source_report=self.json(evidence_path)
                    if (evidence!=publication.get('source_planning_failure_diagnostics')
                            or source_report.get('kind')!='physical_retry_planning_failure_cpu_diagnostics'
                            or source_report.get('retry_round')!=retry_round or source_report.get('complete') is not True
                            or source_report.get('physical_execution_started') is not False
                            or source_report.get('kit_started') is not False
                            or source_report.get('curobo_planning_calls')!=0 or source_report.get('new_target_candidates')!=0
                            or source_report.get('layout_changed') is not (retry_round>=7) or source_report.get('source_plan_unchanged') is not True
                            or source_report.get('source_plan')!=dict(path=manifest['retry_plan_path'],sha256=manifest['retry_plan_sha256'])):continue
                    source_entries=source_report['entries'];source_index={r['scene_id']:r for r in source_entries}
                    if (len(source_index)!=len(source_entries)
                            or source_index.get(row['scene_id'])!={k:v for k,v in diagnostic.items() if k!='evidence'}):continue
                    facts=diagnostic.get('observed_cpu_facts_zh');interpretation=diagnostic.get('interpretation_zh');next_step=diagnostic.get('next_adjustment_zh')
                    if (not isinstance(facts,list) or not facts or any(not isinstance(value,str) or not value.strip() for value in facts)
                            or not isinstance(interpretation,str) or not interpretation.strip()
                            or not isinstance(next_step,str) or not next_step.strip()):continue
                    row['planning_failure_diagnostic']=dict(status='diagnosed',observed_cpu_facts_zh=facts,
                        failed_phase=diagnostic.get('failed_phase'),solver_status=diagnostic.get('solver_status'),
                        interpretation_zh=interpretation,interpretation_is_inference=True,next_adjustment_zh=next_step,
                        root_cause_confirmed=False,scene_infeasibility_established=False,
                        evidence_url='/artifact/'+quote(evidence_path.relative_to(self.root).as_posix(),safe='/'),
                        markdown_report_url=report_url)
                except (ValueError,OSError,KeyError,TypeError,StopIteration):pass
                continue
            if not row.get('result_valid') or row['execution_status'] not in ('failed','error'):continue
            try:
                item=indexed[row['scene_id']];retry=item.get(f'retry_round{retry_round}',item.get('retry')) if retry_round>=2 else item['retry']
                review=retry.get('diagnostic_visual_review',retry.get('visual_review'))
                if not isinstance(review,dict):continue
                result_path=bound(retry['result'])
                if (result_path!=self.root/paths['demos']/'executions'/row['scene_id']/'result.json'
                        or retry['result']['sha256']!=row['result_sha256']
                        or review.get('scene_id')!=row['scene_id'] or review.get('status')!='complete'
                        or review.get('execution_result_sha256')!=row['result_sha256']
                        or not row.get('diagnostic_report_url')):continue
                diagnostic_path=self.root/paths['diagnostics']/'scenes'/row['scene_id']/'diagnostic.json'
                if not valid_digest(review.get('diagnostic_report_sha256')) or self.file_sha256(diagnostic_path)!=review['diagnostic_report_sha256']:continue
                facts=review.get('observed_visual_facts_zh');interpretation=review.get('interpretation_zh');next_step=review.get('next_adjustment_zh')
                confirmed=review.get('root_cause_confirmed');inference=review.get('interpretation_is_inference')
                if (not isinstance(facts,list) or not facts or any(not isinstance(value,str) or not value.strip() for value in facts)
                        or not isinstance(interpretation,str) or not interpretation.strip()
                        or not isinstance(next_step,str) or not next_step.strip()
                        or type(confirmed) is not bool or type(inference) is not bool or inference is confirmed):continue
                basis=review.get('confirmation_basis_zh','')
                if confirmed:
                    evidence=review.get('confirmation_evidence')
                    if not isinstance(evidence,list) or not evidence or not isinstance(basis,str) or not basis.strip():continue
                    for ref in evidence:bound(ref)
                supplementary=[]
                primary=self.json(diagnostic_path)
                for ref in review.get('supplementary_diagnostic_reports',[]):
                    extra_path=bound(ref);extra=self.json(extra_path)
                    if (not any(extra_path.is_relative_to(self.root/paths[key])
                                for key in ('detail_diagnostics','dark_diagnostics','overview_diagnostics','top_diagnostics') if key in paths)
                            or extra.get('kind')!='physical_failure_diagnostic_state_replay' or extra.get('scene_id')!=row['scene_id']
                            or extra.get('status')!='ready' or extra.get('physics_steps_requested')!=0
                            or extra.get('physical_episode_started') is not False or extra.get('timeline_playing') is not False
                            or extra.get('binding')!=primary.get('binding') or not 1<=len(extra.get('frames',[]))<=10):
                        raise ValueError('Supplementary diagnostic is not the same recorded episode')
                    for frame in extra['frames']:
                        image=bound(dict(path=frame['image_path'],sha256=frame['image_sha256']))
                        if (image.parent!=extra_path.parent or not re.fullmatch(r'frame_[0-9]{2}\.jpg',image.name)
                                or not any(all(frame.get(k)==f.get(k) for k in ('state_index','physics_step','phase','action_index')) for f in primary['frames'])):
                            raise ValueError('Supplementary frame differs from recorded diagnostic state')
                        policy=extra.get('observation_policy',{})
                        supplementary.append(dict(frame,url='/artifact/'+quote(image.relative_to(self.root).as_posix(),safe='/'),supplementary=True,
                            observation_label=('俯视补图' if policy.get('top_view') else '斜侧近景' if policy.get('detail_view') else '总览补图')+' · 请求曝光 '+str(policy.get('exposure_ev',0))+'EV'))
                row['diagnostic_visual_review']=dict(status='complete',observed_visual_facts_zh=facts,
                    interpretation_zh=interpretation,next_adjustment_zh=next_step,root_cause_confirmed=confirmed,
                    interpretation_is_inference=inference,confirmation_basis_zh=basis if confirmed else '',
                    markdown_report_url=report_url,source_index_url='/artifact/'+quote(paths['results'],safe='/'))
                row['diagnostics'].extend(supplementary)
            except (ValueError,OSError,KeyError,TypeError):continue

    @property
    def physical_retry_annotation_path(self):
        return self.root/'reports/replicator_dashboard/physical_retry_annotations.json'

    def physical_retry_notes_path(self,retry_round=1):
        retry_paths(retry_round)
        return self.physical_retry_annotation_path if retry_round==1 else self.root/f'reports/replicator_dashboard/physical_retry_round{retry_round}_annotations.json'

    def physical_retry_annotations(self,retry_round=1):
        path=self.physical_retry_notes_path(retry_round)
        if not path.exists():return dict(schema_version=1,entries=[])
        try:
            data=json.loads(path.read_text());seen=set()
            if data.get('schema_version')!=1 or not isinstance(data.get('entries'),list):raise ValueError()
            for row in data['entries']:
                if (not isinstance(row,dict) or not isinstance(row.get('scene_id'),str) or not re.fullmatch(r'[a-z0-9_]{1,160}',row['scene_id'])
                        or row['scene_id'] in seen or not isinstance(row.get('note'),str) or len(row['note'])>ANNOTATION_MAX_NOTE
                        or type(row.get('revision')) is not int or row['revision']<1
                        or not valid_digest(row.get('result_sha256')) or not valid_digest(row.get('config_sha256'))):raise ValueError()
                seen.add(row['scene_id'])
            return data
        except (OSError,ValueError,TypeError) as error:raise AnnotationError(503,'物理重试标注文件异常，保留原文件并停止覆盖。') from error

    def save_physical_retry_annotation(self,payload,retry_round=1):
        fields={'scene_id','note','expected_revision','config_sha256','result_sha256'}
        if (not isinstance(payload,dict) or set(payload)!=fields or not isinstance(payload.get('scene_id'),str)
                or not re.fullmatch(r'[a-z0-9_]{1,160}',payload['scene_id']) or not isinstance(payload.get('note'),str)
                or len(payload['note'])>ANNOTATION_MAX_NOTE or type(payload.get('expected_revision')) is not int or payload['expected_revision']<0
                or not valid_digest(payload.get('config_sha256')) or not valid_digest(payload.get('result_sha256'))):
            raise AnnotationError(400,'物理标注字段无效，最多 20000 字。')
        field=f'failure_retry_round{retry_round}' if retry_round>=2 else 'failure_retry'
        row=next((r for r in self.physical_execution()[field]['entries'] if r['scene_id']==payload['scene_id']),None)
        if not row or not row['result_valid']:raise AnnotationError(409,'物理重试结果尚未保存或来源已变化。')
        path=self.physical_retry_notes_path(retry_round);path.parent.mkdir(parents=True,exist_ok=True)
        with self.physical_retry_annotation_lock, path.with_suffix('.lock').open('a+b') as lock:
            fcntl.flock(lock.fileno(),fcntl.LOCK_EX)
            data=self.physical_retry_annotations(retry_round);current=next((r for r in data['entries'] if r['scene_id']==payload['scene_id']),None)
            if payload['expected_revision']!=(current['revision'] if current else 0):raise AnnotationError(409,'物理标注已被其他页面更新，草稿保留。',current)
            if any(payload[k]!=row[k] for k in ('config_sha256','result_sha256')):raise AnnotationError(409,'实际物理结果已变化，请重新查看后标注。',current)
            entry={k:payload[k] for k in ('scene_id','note','config_sha256','result_sha256')};entry.update(revision=payload['expected_revision']+1,updated_at=iso())
            data['entries']=[r for r in data['entries'] if r['scene_id']!=entry['scene_id']]+[entry]
            temporary=None
            try:
                with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,prefix='.physical-retry-',suffix='.tmp',delete=False) as stream:
                    temporary=Path(stream.name);json.dump(data,stream,ensure_ascii=False,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
                os.replace(temporary,path)
            finally:
                if temporary:temporary.unlink(missing_ok=True)
            return entry

    def camera_retakes(self, first_entries, first_raw, source_path):
        """Read-only overlay; never promote a retake outcome into first-pass counts."""
        manifest = self.json(self.root/CAMERA_RETAKE_DEMOS/'manifest.json')
        empty = dict(total=CAMERA_RETAKE_TOTAL, saved=0, stale=0, entries=[], manifest_present=bool(manifest))
        if not manifest:
            return empty
        first = {row['scene_id']:row for row in first_entries}
        def require(value, message):
            if not value:
                raise ValueError(message)
        def bound(path, digest):
            return valid_digest(digest) and path.is_file() and self.file_sha256(path) == digest
        def retake_path(value):
            value = str(value)
            path = self.local(CAMERA_RETAKE_DEMOS+value if value.startswith(('executions/', 'videos/')) else value)
            path.relative_to(self.root/CAMERA_RETAKE_DEMOS)
            return path
        try:
            plan_path = self.root/CAMERA_RETAKE_PLAN
            plan = self.json(plan_path); plan_sha = self.file_sha256(plan_path)
            planned = plan['entries']; rows = manifest['entries']
            fixed = {sid for sid,row in first.items() if row['task_id'] == 'kitchen_bench_replicator_u_shape_banana_bowl'
                     and sid.startswith('u_shape_banana_bowl__')}
            require(plan.get('kind') == 'physical_camera_retake' and plan.get('select_all_retakes_regardless_of_success') is True
                    and plan.get('physical_attempt_number') == 2 and len(fixed) == CAMERA_RETAKE_TOTAL,
                    '补录计划不是固定的 11 景相机补录。')
            require(len(planned) == len(rows) == len(fixed)
                    and {r['scene_id'] for r in planned} == {r['scene_id'] for r in rows} == fixed,
                    '补录场景存在缺失、重复或越界。')
            require(manifest.get('kind') == 'physical_execution'
                    and self.local(manifest['source_run']) == source_path.parent
                    and manifest.get('source_progress_sha256') == self.file_sha256(source_path)
                    and self.local(plan['first_pass_manifest_path']) == self.root/PHYSICAL_DEMOS/'manifest.json',
                    '补录来源未绑定本机首轮与规划输入。')
            override_path = self.local(plan['camera_overrides_path'])
            require(bound(override_path, plan['camera_overrides_sha256']), '补录相机配置 SHA 无效。')
            override = self.json(override_path)
            plan_by_scene = {r['scene_id']:r for r in planned}
        except (ValueError, KeyError, TypeError, OSError) as error:
            return dict(empty, stale=CAMERA_RETAKE_TOTAL, reason=str(error))
        checked = []
        for raw in rows:
            sid = raw['scene_id']; approved = plan_by_scene[sid]
            row = dict(scene_id=sid, physical_attempt_number=2, execution_status='pending',
                stale=False, saved=False, result_url=None, video_url=None, reason='')
            try:
                previous = first_raw[sid]; visible = first[sid]
                require(visible['result_valid'] and visible['video_available'], '首轮结果或视频来源待核。')
                require(type(raw.get('physical_attempt_number')) is int and raw['physical_attempt_number'] == 2
                        and self.local(raw['camera_retake_plan_path']) == plan_path
                        and raw.get('camera_retake_plan_sha256') == plan_sha,
                        '补录必须绑定固定计划并明确为第 2 次。')
                for key in ('scene_id','task_id','variant_id','config_sha256','trajectory_sha256'):
                    require(raw.get(key) == previous.get(key) == approved.get(key), '补录规划身份不一致：'+key)
                for key in ('config_path','trajectory_path','result_path'):
                    require(self.local(raw[key]) == self.local(previous[key]), '补录改动了规划输入路径。')
                require(raw['result_sha256'] == previous['result_sha256'], '补录规划结果 SHA 不一致。')
                expected_first = dict(result_path=str(self.local(PHYSICAL_DEMOS+previous['execution_result_path']
                    if previous['execution_result_path'].startswith('executions/') else previous['execution_result_path']).relative_to(self.root)),
                    result_sha256=previous['execution_result_sha256'], video_path=PHYSICAL_DEMOS+'videos/'+sid+'.mp4',
                    video_sha256=previous['video_sha256'], execution_status=visible['execution_status'],
                    task_success=visible['task_success'], executed=visible['executed'], physical_attempt_number=1)
                require(raw.get('first_pass') == approved.get('first_pass') == expected_first,
                        '补录未绑定原首轮结果、视频及真实成绩。')
                status = raw['execution_status']; require(status in PHYSICAL_STATUSES, '未知补录状态。')
                row['execution_status'] = status
                if status in ('pending','running'):
                    checked.append(row); first[sid]['camera_retake'] = row; continue
                path = retake_path(raw['execution_result_path'])
                require(path == self.root/CAMERA_RETAKE_DEMOS/'executions'/sid/'result.json'
                        and bound(path,raw['execution_result_sha256']), '补录实际结果路径或 SHA 无效。')
                result = self.json(path)
                require(result.get('kind') == 'physical_execution'
                        and all(result.get(k) == raw.get(k) for k in ('scene_id','task_id','variant_id','config_sha256',
                            'trajectory_sha256','planning_status','execution_status','executed','task_success','attempts',
                            'physical_attempt_number','first_pass','camera_retake_plan_path','camera_retake_plan_sha256')),
                        '补录结果与清单不一致。')
                require(type(result.get('attempts')) is int and result['attempts'] == 1
                        and type(result.get('executed')) is bool and type(result.get('task_success')) is bool
                        and result['task_success'] is (status=='success')
                        and (result['executed'] or status=='error'), '补录实际执行标记矛盾。')
                require(self.local(result['planning_result_path']) == self.local(previous['result_path'])
                        and result['planning_result_sha256'] == previous['result_sha256']
                        and self.local(result['config_path']) == self.local(previous['config_path'])
                        and self.local(result['trajectory_path']) == self.local(previous['trajectory_path']),
                        '补录实际结果未绑定原始规划输入。')
                settings = result.get('runtime_settings', {}); locked = approved['locked_execution_settings']
                require(set(settings) == set(locked)|{'camera_overrides_path','camera_overrides_sha256',
                        'camera_retake_plan_path','camera_retake_plan_sha256'}
                        and all(settings[k]==v for k,v in locked.items())
                        and self.local(settings['camera_overrides_path']) == override_path
                        and settings['camera_overrides_sha256'] == plan['camera_overrides_sha256']
                        and self.local(settings['camera_retake_plan_path']) == plan_path
                        and settings['camera_retake_plan_sha256'] == plan_sha, '补录运行参数超出相机变化范围。')
                row.update(result_url='/artifact/'+quote(path.relative_to(self.root).as_posix(),safe='/'),
                           reason=str(result.get('failure_reason') or result.get('error') or ''),
                           task_success=result['task_success'], executed=result['executed'])
                if raw.get('video_path') and result.get('video_path'):
                    video = retake_path(raw['video_path']); camera = result.get('camera_override', {})
                    require(video == self.root/CAMERA_RETAKE_DEMOS/'videos'/(sid+'.mp4')
                            and retake_path(result['video_path']) == video
                            and result.get('video_sha256') == raw.get('video_sha256')
                            and bound(video,raw['video_sha256'])
                            and camera.get('sha256') == plan['camera_overrides_sha256']
                            and camera.get('applied') == override['tasks'][raw['task_id']], '补录视频或实际相机绑定无效。')
                    row.update(video_url='/artifact/'+quote(video.relative_to(self.root).as_posix(),safe='/'), saved=True)
            except (ValueError, KeyError, TypeError, OSError) as error:
                row.update(stale=True, saved=False, result_url=None, video_url=None, reason=str(error))
            checked.append(row); first[sid]['camera_retake'] = row
        return dict(empty, entries=checked, saved=sum(row['saved'] for row in checked),
                    stale=sum(row['stale'] for row in checked), updated_at=manifest.get('updated_at'))

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

    def episode_log(self, episode: Path) -> Path | None:
        """Bind only a unique log whose explicit JSON output is this exact episode.

        Matching the basename alone is insufficient: different tasks/variants
        routinely reuse attempt names. Logs without an output identity stay unlinked.
        """
        if self.report_logs is None:
            self.report_logs = {}
            for log in self.root.glob("reports/replicator_execution*/**/*.log"):
                self.report_logs.setdefault(log.stem.replace("_", ""), []).append(log)
        candidates = [episode.with_suffix(".log"), *self.report_logs.get(episode.name.replace("_", ""), [])]
        matches = []
        for log in dict.fromkeys(candidates):
            try:
                st = log.stat()
                stamp, key = (st.st_mtime_ns, st.st_size), str(log)
                if key not in self.log_cache or self.log_cache[key][0] != stamp:
                    with log.open("rb") as stream:
                        stream.seek(max(0, st.st_size - 256 * 1024))
                        text = stream.read().decode(errors="replace")
                    outputs = set()
                    for value in re.findall(r'"output"\s*:\s*("(?:[^"\\]|\\.)*")', text):
                        try:
                            outputs.add(self.local(json.loads(value)))
                        except (ValueError, TypeError):
                            outputs.add(Path("/unverified-output"))
                    self.log_cache[key] = (stamp, outputs)
                if self.log_cache[key][1] == {episode.resolve()} and self.artifact(log):
                    matches.append(log)
            except OSError:
                continue
        return matches[0] if len(matches) == 1 else None

    def entry(self, path: Path, data: dict, kind: str = "episode") -> dict:
        success = kind == "episode" and physical_success(data)
        status = "partial" if success else "failed"
        reason = str(data.get("error") or data.get("runtime_error") or data.get("audit_error")
                     or data.get("failure_reason") or data.get("reason") or "")
        if kind != "episode":
            status = "check_passed" if data.get("passed") is True or data.get("neutral_passed") is True else "failed"
        if kind == "neutral" and "neutral_probe_passed" in data:
            audit = data.get("physical_evidence_audit") or {}
            if data.get("kind") == "matched_hazard_neutral_observation" and data.get("schema_version") == 1:
                checks = data.get("checks") or {}
                audit = {"passed": data.get("passed") is True and bool(checks)
                         and all(value is True for value in checks.values()), "checks": checks}
            failed_checks = [name for name, passed in audit.get("checks", {}).items() if passed is False]
            passed = (data.get("neutral_probe_passed") is True
                      and data.get("neutral_sequence_complete") is True
                      and audit.get("passed") is True and not failed_checks and not reason)
            status = "check_passed" if passed else "failed"
            if not reason:
                labels = {"background_stable": "背景物体不稳定（background_stable）"}
                detail = "、".join(labels.get(name, name) for name in failed_checks)
                reason = "静置检查通过；尚不代表原始任务完成。" if passed else "静置检查未通过" + ("：" + detail if detail else "；详见运行报告。")
        if kind == "diagnostic":
            completed = (data.get("diagnostic_completed") is True
                         and data.get("status") == "bounded_observation_complete"
                         and not data.get("error"))
            status = "partial" if completed else "failed"
            prefix = "诊断观测已结束；尚未验收，不计原始任务成功。" if completed else "诊断失败；不计原始任务成功。"
            reason = prefix + (" " + reason if reason else "")
        artifacts = []
        report_label = "诊断报告" if kind == "diagnostic" else "运行报告"
        candidates = [(path, report_label), (path.parent / "all_cameras.mp4", "原始多视角视频"),
                      (path.parent / "external_camera_2.mp4", "原始外部相机"),
                      (path.parent / "wrist_camera.mp4", "原始腕部相机"),
                      (path.parent / "overview" / "overview_camera.mp4", "总览视频"),
                      (path.parent / "overview" / "overview.mp4", "总览视频")]
        log = self.episode_log(path.parent) if kind == "episode" else path.parent.with_suffix(".log")
        if log:
            candidates.append((log, "运行日志"))
        if kind in ("neutral", "diagnostic") and path.name == "result.json":
            candidates.append((path.parent / "launch_failure.json", "启动器退出报告"))
        for p, label in candidates:
            a = self.artifact(p, label)
            if a:
                artifacts.append(a)
        return {"path": path.relative_to(self.root).as_posix(), "kind": kind, "status": status,
                "reason": reason,
                "updated_at": iso(path.stat().st_mtime), "timestamp": path.stat().st_mtime,
                "physical_success": success,
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

    def process_script(self, args: list[str], proc: Path) -> str | None:
        """Identify the executed repository file, including private diagnostic leaves.

        Verify the process's actual mount/cwd inode so another checkout or a
        script merely mentioned as an input cannot become a live task here.
        Supervisors are excluded; their actual worker is counted separately.
        """
        if not args or not Path(args[0]).name.startswith("python"):
            return None
        for arg in args[1:]:
            if arg in ("-u", "-B", "-s", "-S", "-E", "-I", "-O", "-OO"):
                continue
            if arg.startswith("-"):
                return None
            try:
                local = self.local(arg)
                relative = local.relative_to(self.root).as_posix()
                allowed = relative.startswith(("scripts/extensions/replicator/", "reports/replicator_preparation/"))
                if (not allowed or local.suffix != ".py" or local.name == "progress_dashboard.py"
                        or local.name.endswith("_supervisor.py")):
                    return None
                actual = proc / "root" / arg.lstrip("/") if Path(arg).is_absolute() else proc / "cwd" / arg
                return arg if actual.samefile(local) else None
            except (OSError, ValueError):
                return None
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
                script = self.process_script(args, p)
                if not script:
                    continue
                stat = p.joinpath("stat").read_text().split(") ", 1)[1].split()
                if stat[0] == "Z":
                    continue
                tid = self.infer_task(" ".join(args[1:]), tasks)
                for flag in ("--controller-profile", "--layout"):
                    if not tid and flag in args:
                        d = self.json(self.local(args[args.index(flag) + 1]))
                        tid = d.get("task_id")
                if "reports/replicator_preparation/" in script and tid not in tasks:
                    continue
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

    def scene_build(self, tasks: dict) -> dict:
        """Display scene construction independently from episode acceptance.

        The runner owns the progress file. A ready label additionally needs its
        recorded simulation-loaded check and an existing local evidence report;
        the dashboard does not execute or independently validate the simulation.
        """
        report = self.root / "reports/replicator_scene_build/progress.json"
        data = self.json(report)
        warnings = []
        if not report.exists():
            manifest = self.json(self.root / "configs/extensions/replicator/scene_variants/manifest.json")
            if manifest.get("schema_version") == 1:
                data = {**manifest, "entries": manifest.get("scenes", []),
                        "stage": manifest.get("stage", "正在构建场景清单")}
        if report.exists() and data.get("schema_version") != 1:
            warnings.append("场景进度文件格式无效，暂按待开始显示。")
            data = {}
        variants = [{"id": key, "name": label, "category": category}
                    for key, label, category in SCENE_VARIANTS]
        keys = {v["id"] for v in variants}
        entries = {}
        raw_entries = data.get("entries", [])
        if not isinstance(raw_entries, list):
            raw_entries = []
            warnings.append("场景 entries 不是列表，暂按待开始显示。")
        for raw in raw_entries:
            if (not isinstance(raw, dict) or not isinstance(raw.get("task_id"), str)
                    or not isinstance(raw.get("variant_id"), str)
                    or raw["task_id"] not in tasks or raw["variant_id"] not in keys):
                warnings.append("忽略未知任务或变体的场景记录。")
                continue
            key = (raw["task_id"], raw["variant_id"])
            if key in entries:
                entries[key] = {"status": "failed", "error": "重复的场景记录；需要生成器修正。"}
                warnings.append("存在重复场景记录，未重复计数。")
                continue
            entries[key] = raw
        values = []
        for task_id in tasks:
            for variant in variants:
                raw = entries.get((task_id, variant["id"]), {})
                reported = raw.get("status", "pending")
                mapped = {"configured": "validated", "configuration_failed": "failed"}.get(str(reported), reported)
                status = mapped if mapped in SCENE_STATUSES else "pending"
                checks = raw.get("checks") if isinstance(raw.get("checks"), dict) else {}
                error = str(raw.get("error") or "")
                artifacts, paths = [], {}
                for name, label in (("config_path", "场景配置"), ("scene_path", "仿真场景"),
                                    ("preview_path", "仿真截图"), ("evidence_path", "仿真加载报告"),
                                    ("validation_path", "静态检查报告"), ("log_path", "构建日志")):
                    alias = {"config_path": "layout_path", "validation_path": "cpu_evidence_path"}
                    value = raw.get(name) or raw.get(alias.get(name, ""))
                    if not isinstance(value, str) or not value:
                        continue
                    try:
                        path = self.local(value)
                        artifact = self.artifact(path, label)
                        if artifact:
                            if name == "preview_path":
                                artifact["kind"] = "image"
                            artifacts.append(artifact)
                            paths[name] = artifact["url"]
                    except (OSError, ValueError):
                        pass
                if status == "ready" and (checks.get("simulation_loaded") is not True or not paths.get("evidence_path")):
                    status = "validated" if checks.get("static_validated") is True else "generated"
                    error = error or "尚缺实际仿真加载检查或本地加载报告；暂不计入仿真完成。"
                if error and status == "ready":
                    status = "failed"
                if status == "ready" and any(checks.get(name) is not True for name in SCENE_LOAD_CHECKS):
                    status = "failed"
                    error = "必需的场景加载检查缺失或未通过；暂不计入仿真完成。"
                values.append({"task_id": task_id, "variant_id": variant["id"],
                               "variant_name": str(raw.get("variant_name") or variant["name"]),
                               "category": variant["category"], "status": status,
                               "reported_status": reported, "error": error, "checks": checks,
                               "artifacts": artifacts, "paths": paths, "updated_at": raw.get("updated_at")})
        counts = {s: sum(e["status"] == s for e in values) for s in SCENE_STATUSES}
        return {"schema_version": 1, "stage": str(data.get("stage") or "等待生成场景"),
                "updated_at": data.get("updated_at"), "total": len(values), "target_total": 154,
                "counts": counts, "variants": variants, "entries": values,
                "warnings": list(dict.fromkeys(warnings)),
                "notice": "场景搭建与数采验收独立计数；仿真已加载不等于避障或操作成功。"}

    def safety_upgrade(self, tasks: dict) -> dict:
        """Display explicitly saved upgrade evidence; never inherit scene-build success."""
        report = self.root / "reports/replicator_safety_upgrade/progress.json"
        data = self.json(report)
        warnings = []
        if data and data.get("schema_version") != 1:
            warnings.append("安全升级进度格式无效，全部暂按待验证显示。")
            data = {}
        raw_entries = data.get("entries", [])
        if not isinstance(raw_entries, list):
            warnings.append("安全升级 entries 不是列表，全部暂按待验证显示。")
            raw_entries = []
        variants = [{"id": key, "name": name, "category": category} for key, name, category in SCENE_VARIANTS]
        variant_ids = {variant["id"] for variant in variants}
        indexed = {}
        for raw in raw_entries:
            if (not isinstance(raw, dict) or not isinstance(raw.get("task_id"), str)
                    or not isinstance(raw.get("variant_id"), str)
                    or raw["task_id"] not in tasks or raw["variant_id"] not in variant_ids):
                warnings.append("忽略未知任务或变体的安全升级记录。")
                continue
            key = (raw["task_id"], raw["variant_id"])
            if key in indexed:
                indexed[key] = {"status": "needs_repair", "reason": "重复场景记录，需要修正进度文件。"}
                warnings.append("重复安全升级记录未重复计数。")
            else:
                indexed[key] = raw
        values = []
        for task_id in tasks:
            for variant in variants:
                raw = indexed.get((task_id, variant["id"]), {})
                reported = raw.get("status", "pending")
                if reported not in SAFETY_STATUSES:
                    warnings.append("未知安全升级状态未计为通过。")
                    raw = {}
                status = raw.get("status", "pending")
                artifacts, paths = [], {}
                for name, label in (("config_path", "升级配置"), ("evidence_path", "安全检查证据"),
                                    ("preview_path", "场景预览"), ("video_path", "执行视频"),
                                    ("log_path", "升级日志")):
                    value = raw.get(name)
                    if not isinstance(value, str) or not value:
                        continue
                    try:
                        path = self.local(value)
                        rel = path.relative_to(self.root).as_posix()
                        # Existing scene previews/configs are reusable; a previous load report
                        # cannot become safety evidence by being linked here.
                        permitted = rel.startswith("reports/replicator_safety_upgrade/")
                        if name == "config_path":
                            permitted |= rel.startswith(("configs/extensions/replicator/scene_variants/",
                                                         "configs/extensions/replicator/safety_scene_variants/",
                                                         "configs/extensions/replicator/safety_upgrade/"))
                        if name == "preview_path":
                            permitted |= rel.startswith("reports/replicator_scene_build/")
                        if not permitted or path.stat().st_size <= 0:
                            continue
                        artifact = self.artifact(path, label)
                        if artifact:
                            if name == "preview_path":
                                artifact["kind"] = "image"
                            if name == "video_path" and path.suffix.lower() != ".mp4":
                                continue
                            artifacts.append(artifact)
                            paths[name] = artifact["url"]
                    except (OSError, ValueError):
                        pass
                evidence = bool(paths.get("evidence_path"))
                baseline = raw.get("baseline_status", "pending")
                baseline_verified = baseline in ("verified", "accepted")
                checks = {name: raw.get(name) is True and evidence for name in SAFETY_CHECKS}
                checks["risk_verified"] &= checks["geometry_passed"] and baseline_verified
                checks["safe_execution_verified"] &= (checks["risk_verified"] and checks["native_stability_verified"]
                                                       and status != "needs_repair" and not raw.get("error"))
                checks["video_saved"] = raw.get("video_saved") is True and bool(paths.get("video_path"))
                reason = str(raw.get("reason") or raw.get("error") or "")
                if status == "accepted" and not all(checks.values()):
                    status = "needs_repair"
                    reason = reason or "验收所需的检查、基线或本地证据不完整；尚未计为验收通过。"
                if status == "risk_verified" and not checks["risk_verified"]:
                    status = "geometry_passed" if checks["geometry_passed"] else "waiting_baseline"
                    reason = reason or "尚缺匹配成功基线或风险检查证据。"
                if status == "geometry_passed" and not checks["geometry_passed"]:
                    status = "screening"
                    reason = reason or "尚缺几何检查证据。"
                values.append({"scene_id": str(raw.get("scene_id") or task_id + "/" + variant["id"]),
                               "task_id": task_id, "variant_id": variant["id"], "variant_name": variant["name"],
                               "category": variant["category"], "status": status, "reported_status": reported,
                               "baseline_status": baseline, "baseline_verified": baseline_verified,
                               **checks, "checks": checks, "reason": reason,
                               "artifacts": artifacts, "paths": paths, "updated_at": raw.get("updated_at")})
        counts = {name: sum(entry[name] is True for entry in values) for name in SAFETY_CHECKS}
        counts.update({name: sum(entry["status"] == name for entry in values) for name in ("accepted", "needs_repair")})
        counts["active"] = sum(entry["status"] in ("screening", "executing") for entry in values)
        counts["waiting_baseline"] = sum(not entry["baseline_verified"] for entry in values)
        return {"schema_version": 1, "total": len(values), "target_total": 154,
                "stage": str(data.get("stage") or "等待安全升级"), "updated_at": data.get("updated_at"),
                "counts": counts, "variants": variants, "entries": values,
                "warnings": list(dict.fromkeys(warnings)),
                "notice": "各检查独立累计；风险仅针对一条匹配成功基线，不代表多种子稳健性。场景加载不计为安全通过。"}

    def visual_triage(self, tasks: dict, scenes: dict) -> dict:
        """Bind visual observations to the exact displayed preview, never new poses."""
        folder = self.root / "reports/replicator_safety_upgrade/visual_triage"
        index = self.json(folder / "index.json")
        indexed, reviews, warnings = {}, {}, []
        for task in index.get("tasks", []) if isinstance(index.get("tasks", []), list) else []:
            if not isinstance(task, dict):
                continue
            for row in task.get("entries", []) if isinstance(task.get("entries", []), list) else []:
                if isinstance(row, dict) and isinstance(row.get("scene_id"), str):
                    indexed[row["scene_id"]] = {**row, "sheet_path": task.get("sheet_path")}
        for path in sorted(folder.glob("review_*.json"), key=lambda p: (p.stat().st_mtime_ns, p.name)):
            rows = self.json(path).get("entries", [])
            if not isinstance(rows, list):
                warnings.append("忽略格式无效的视觉审核文件：" + path.name)
                continue
            for row in rows:
                if not isinstance(row, dict) or not isinstance(row.get("scene_id"), str):
                    continue
                reviews[row["scene_id"]] = {**row, "review_path": path.relative_to(self.root).as_posix(),
                                            "review_updated_at": iso(path.stat().st_mtime)}
        current = {(row["task_id"], row["variant_id"]): row for row in scenes["entries"]}
        values = []
        for task_id in tasks:
            for variant_id, name, category in SCENE_VARIANTS:
                scene_id = task_id.removeprefix(PREFIX) + "__" + variant_id
                source, review = indexed.get(scene_id, {}), reviews.get(scene_id, {})
                preview_url = current.get((task_id, variant_id), {}).get("paths", {}).get("preview_path", "")
                preview = unquote(preview_url[len('/artifact/'):]) if preview_url.startswith('/artifact/') else source.get('preview_path')
                artifacts, paths, actual_hash = [], {}, None
                for key, value, label in (("preview_path", preview, "当前既有截图"),
                                          ("review_preview_path", review.get("preview_path"), "审核引用截图"),
                                          ("config_path", source.get("config_path"), "截图原配置"),
                                          ("sheet_path", source.get("sheet_path"), "任务11图总览"),
                                          ("review_path", review.get("review_path"), "视觉审核记录")):
                    if not isinstance(value, str) or not value:
                        continue
                    try:
                        path = self.local(value)
                        rel = path.relative_to(self.root).as_posix()
                        if not rel.startswith(("reports/replicator_scene_build/", "reports/replicator_safety_upgrade/visual_triage/",
                                               "configs/extensions/replicator/scene_variants/")):
                            continue
                        artifact = self.artifact(path, label)
                        if artifact:
                            if key in ("preview_path", "review_preview_path", "sheet_path"):
                                artifact["kind"] = "image"
                            artifacts.append(artifact)
                            paths[key] = artifact["url"]
                            if key == "preview_path":
                                stat = path.stat()
                                stamp, hash_key = (stat.st_mtime_ns, stat.st_size), str(path)
                                if hash_key not in self.hash_cache or self.hash_cache[hash_key][0] != stamp:
                                    self.hash_cache[hash_key] = (stamp, hashlib.sha256(path.read_bytes()).hexdigest())
                                actual_hash = self.hash_cache[hash_key][1]
                    except (OSError, ValueError):
                        pass
                reported = review.get("visual_status")
                path_match = bool(paths.get("preview_path")) and paths.get("preview_path") == paths.get("review_preview_path")
                hash_match = bool(actual_hash) and actual_hash == review.get("preview_sha256")
                complete = (reported in ("adjust", "candidate", "uncertain")
                            and isinstance(review.get("observation"), str) and bool(review["observation"].strip())
                            and isinstance(review.get("recommendation"), str) and bool(review["recommendation"].strip()))
                status = reported if complete and path_match and hash_match else "historical" if review else "pending"
                reason = ("" if status not in ("historical", "pending") else
                          "尚未审核这张截图。" if not review else
                          "审核记录缺少完整观察或建议，需补充。" if not complete else
                          "截图路径或SHA256已变化；此记录仅供历史参考，需要重看当前截图。")
                issue_codes = review.get("issue_codes", [])
                values.append(dict(scene_id=scene_id, task_id=task_id, variant_id=variant_id,
                    variant_name=name, category=category, visual_status=status, reported_status=reported,
                    priority=review.get("priority") if type(review.get("priority")) is int and review["priority"] in (1, 2, 3) else None,
                    observation=str(review.get("observation") or ""), recommendation=str(review.get("recommendation") or ""),
                    issue_codes=[x for x in issue_codes if isinstance(x, str)] if isinstance(issue_codes, list) else [],
                    confidence=review.get("confidence"), reason=reason, preview_sha256=actual_hash,
                    reviewed_preview_sha256=review.get("preview_sha256"), current_preview_reviewed=status in ("adjust", "candidate", "uncertain"),
                    applies_to_candidate_pose=False, artifacts=artifacts, paths=paths,
                    updated_at=review.get("review_updated_at")))
        counts = {status: sum(row["visual_status"] == status for row in values)
                  for status in ("adjust", "candidate", "uncertain", "historical", "pending")}
        counts["reviewed"] = counts["adjust"] + counts["candidate"] + counts["uncertain"]
        counts["priority1"] = sum(row["priority"] == 1 and row["current_preview_reviewed"] for row in values)
        return dict(total=len(values), target_total=154, stage="图像初筛优先 · 先检查154张既有截图",
                    variants=scenes["variants"], entries=values, counts=counts,
                    warnings=list(dict.fromkeys(warnings)),
                    notice="仅判断所显示截图的摆位与视图质量，不证明物理稳定、轨迹风险或操作成功；结论不自动适用于新安全候选摆位。")

    def visual_placement(self, tasks: dict) -> dict:
        render_folder = self.root / 'reports/replicator_safety_upgrade/visual_placement'
        final_folder = self.root / 'reports/replicator_safety_upgrade/visual_placement_final01'
        versions = [self.root / ('reports/replicator_safety_upgrade/visual_placement_final' + version)
                    for version in ('03', '02', '01')]
        folder = next((path for path in versions if (path/'progress.json').is_file()), render_folder)
        data = self.json(folder / 'progress.json')
        placement = {}
        renders = {row['scene_id']: row for row in self.json(folder / 'render_reuse.json').get('entries', [])
                   if isinstance(row, dict) and isinstance(row.get('scene_id'), str)}
        reuse = {row['scene_id']: row for row in self.json(folder / 'preview_reuse.json').get('entries', [])
                 if isinstance(row, dict) and isinstance(row.get('scene_id'), str)}
        for row in data.get('entries', []) if isinstance(data.get('entries', []), list) else []:
            if isinstance(row, dict) and isinstance(row.get('scene_id'), str):
                placement[row['scene_id']] = row
        for path in sorted(set(render_folder.glob('render*progress.json')) | set(folder.glob('render*progress.json')), key=lambda p: p.stat().st_mtime_ns):
            report = self.json(path)
            for row in report.get('entries', []) if isinstance(report.get('entries', []), list) else []:
                if isinstance(row, dict) and isinstance(row.get('scene_id'), str):
                    renders[row['scene_id']] = row
        reviews = {}
        for path in sorted(folder.glob('review_*.json'), key=lambda p:p.stat().st_mtime_ns):
            for row in self.json(path).get('entries', []):
                if isinstance(row,dict) and isinstance(row.get('scene_id'),str):
                    reviews[row['scene_id']] = dict(row, review_path=path.relative_to(self.root).as_posix())

        def bound_file(value, digest=None, label='文件', kind='report', allow_original=False):
            if not isinstance(value, str):
                return None
            try:
                path = self.local(value)
                rel = path.relative_to(self.root).as_posix()
                prefixes = ('reports/replicator_safety_upgrade/visual_placement/',
                            'reports/replicator_safety_upgrade/visual_placement_final01/',
                            'reports/replicator_safety_upgrade/visual_placement_final02/',
                            'reports/replicator_safety_upgrade/visual_placement_final03/',
                            'reports/replicator_safety_upgrade/adjust02/',
                            'reports/replicator_safety_upgrade/adjust04/',
                            'reports/replicator_safety_upgrade/adjust05/',
                            'reports/replicator_safety_upgrade/adjust06/',
                            'reports/replicator_safety_upgrade/adjust16/',
                            'configs/extensions/replicator/visual_scene_variants/',
                            'configs/extensions/replicator/visual_scene_variants_final01/',
                            'configs/extensions/replicator/visual_scene_variants_final02/',
                            'configs/extensions/replicator/visual_scene_variants_final03/',
                            'configs/extensions/replicator/l_island_mustard_adjust02/',
                            'configs/extensions/replicator/peninsula_mustard_adjust02/',
                            'configs/extensions/replicator/l_shape_mustard_adjust05/',
                            'configs/extensions/replicator/l_shape_mustard_adjust16/',
                            'configs/extensions/replicator/u_shape_mustard_adjust05/')
                if allow_original:
                    prefixes += ('reports/replicator_scene_build/', 'reports/replicator_safety_upgrade/visual_triage/',
                                 'reports/replicator_preparation/',
                                 'configs/extensions/replicator/scene_variants/', 'configs/extensions/replicator/safety_scene_variants/')
                if not rel.startswith(prefixes):
                    return None
                artifact = self.artifact(path, label)
                if artifact and digest is not None:
                    stat = path.stat()
                    stamp, key = (stat.st_mtime_ns, stat.st_size), str(path)
                    if key not in self.hash_cache or self.hash_cache[key][0] != stamp:
                        self.hash_cache[key] = (stamp, hashlib.sha256(path.read_bytes()).hexdigest())
                    if self.hash_cache[key][1] != digest:
                        return None
                if artifact:
                    artifact['kind'] = kind
                return artifact
            except (OSError, ValueError):
                return None

        blocked_tasks = {}
        geometry_issues_path = final_folder/'original_task_geometry_issues.json'
        issue_artifact = bound_file(geometry_issues_path.relative_to(self.root).as_posix(), label='原始任务几何问题汇总')
        for issue in self.json(geometry_issues_path).get('entries', []) if issue_artifact else []:
            if not isinstance(issue,dict):
                continue
            tid = issue.get('task_id')
            if tid not in tasks or issue.get('status') != 'original_task_geometry_blocked' or issue.get('applies_to_all_task_variants') is not True:
                continue
            digest = issue.get('evidence_sha256')
            evidence = bound_file(issue.get('evidence_path'),digest,label='原始任务源几何相交证据',allow_original=True)
            if not evidence or not isinstance(digest,str) or len(digest)!=64:
                continue
            reason = issue.get('reason') or issue.get('visual_observation') or '原始任务物体与场景非支撑几何存在已核实相交，全部11变体需先修复原布局。'
            blocked_tasks[tid] = dict(task_id=tid, reason=str(reason), issues=issue.get('issues',[]), artifacts=[issue_artifact,evidence])
        # Keep the independently confirmed earlier report while the full
        # task inventory is being written; missing reports never imply collision.
        legacy_tid = PREFIX+'l_island_mustard_bowl'
        legacy_path = final_folder/'l_island_mustard_original_geometry_issue.json'
        legacy_artifact = bound_file(legacy_path.relative_to(self.root).as_posix(), label='原始任务几何相交证据')
        legacy = self.json(legacy_path) if legacy_artifact else {}
        if (legacy_tid in tasks and legacy_tid not in blocked_tasks and legacy.get('status')=='original_task_geometry_blocked'
                and legacy.get('applies_to_all_task_variants') is True and legacy.get('task_id') in (legacy_tid,legacy_tid.removeprefix(PREFIX))):
            blocked_tasks[legacy_tid] = dict(task_id=legacy_tid, reason='芥末瓶与烤面包机、目标碗与背景香蕉相交；本任务11变体需先修复原布局。', artifacts=[legacy_artifact], issues=legacy.get('entries',[]))
        values = []
        for task_id in tasks:
            for variant_id, name, category in SCENE_VARIANTS:
                scene_id = task_id.removeprefix(PREFIX) + '__' + variant_id
                raw, render = placement.get(scene_id, {}), renders.get(scene_id, {})
                status = raw.get('status', 'pending')
                if status not in ('pending', 'retained', 'adjusted', 'needs_repair'):
                    status = 'pending'
                config = bound_file(raw.get('config_path'), raw.get('config_sha256'), '新摆位配置')
                repair_artifact = bound_file(raw.get('source_layout_repair_evidence_path'),raw.get('source_layout_repair_evidence_sha256'), '新源布局修复证书')
                repair = self.json(self.local(raw['source_layout_repair_evidence_path'])) if repair_artifact else {}
                repair_valid = False
                if raw.get('source_layout_repaired') is True and config and repair_artifact:
                    layout_bound = bound_file(repair.get('repaired_layout_path'),repair.get('repaired_layout_sha256'), '修复后的任务源布局')
                    reports = repair.get('source_reports',[])
                    report_hashes = (isinstance(reports,list) and len(reports)>=2 and all(isinstance(r,dict)
                        and isinstance(r.get('sha256'),str) and len(r['sha256'])==64
                        and bound_file(r.get('path'),r['sha256'],allow_original=True) for r in reports))
                    layout = self.json(self.local(repair['repaired_layout_path'])) if layout_bound else {}
                    current = self.json(self.local(raw['config_path']))
                    repair_valid = bool(isinstance(raw.get('source_layout_repair_evidence_sha256'),str)
                        and len(raw['source_layout_repair_evidence_sha256'])==64 and layout_bound
                        and isinstance(repair.get('repaired_layout_sha256'),str) and len(repair['repaired_layout_sha256'])==64
                        and all(repair.get(key) is True for key in ('passed','relations_passed','source_geometry_passed'))
                        and repair.get('task_id')==task_id and layout.get('task_id')==task_id and report_hashes
                        and {k:v for k,v in current.items() if k!='obstacle'}=={k:v for k,v in layout.items() if k!='obstacle'})
                evidence = bound_file(raw.get('evidence_path'), raw.get('evidence_sha256'), '摆位检查报告')
                geometry = (raw.get('semantic_geometry_passed') is True and bool(config) and bool(evidence)
                            and isinstance(raw.get('config_sha256'), str) and isinstance(raw.get('evidence_sha256'), str))
                reason = str(raw.get('reason') or '')
                if status == 'adjusted' and not geometry:
                    status, reason = 'needs_repair', '调整后的配置或检查证据哈希不完整 / 已变化，暂不计为摆位通过。'
                render_status = render.get('render_status', 'pending')
                if render_status not in ('pending', 'prepared', 'rendering', 'ready', 'needs_view_review', 'failed'):
                    render_status = 'pending'
                matched = bool(config and raw.get('config_sha256') and render.get('config_sha256') == raw['config_sha256'])
                views = []
                for view in render.get('views', []) if isinstance(render.get('views', []), list) else []:
                    if not isinstance(view, dict):
                        continue
                    image = bound_file(view.get('preview_path'), view.get('preview_sha256'),
                                       {'overview':'新总览', 'operation_side':'操作区侧视', 'operation_top':'操作区俯视'}.get(view.get('kind'), '补充视图'), 'image')
                    if image and isinstance(view.get('preview_sha256'), str):
                        views.append(dict(kind=view.get('kind'), view_id=view.get('view_id'), image=image,
                            visibility=view.get('visibility', {}), preview_sha256=view['preview_sha256']))
                render_evidence = bound_file(render.get('evidence_path'), render.get('evidence_sha256'), label='多视角渲染报告')
                report = self.json(self.local(render['evidence_path'])) if render_evidence else {}
                report_bound = bool(report.get('config_sha256') == raw.get('config_sha256') and raw.get('config_sha256'))
                readable = (matched and report_bound and {'overview', 'operation_side', 'operation_top'} <= {v['kind'] for v in views}
                            and render.get('readability_passed') is True and report.get('readability', {}).get('readability_passed') is True)
                if render_status in ('ready', 'needs_view_review') and not matched:
                    render_status = 'historical'
                elif render_status == 'ready' and not readable:
                    render_status = 'needs_view_review'
                view_scope_valid = scene_id not in data.get('fresh_view_scenes',[]) or bool(report.get('camera_overrides_sha256'))
                if not view_scope_valid:
                    readable = False
                    if render_status in ('ready','needs_view_review'):
                        render_status = 'historical'
                reused, reuse_artifacts, reuse_reason = False, [], ''
                old = reuse.get(scene_id, {})
                if render_status in ('pending', 'prepared') and config and old:
                    original = bound_file(old.get('original_config_path'), old.get('original_config_sha256'), '原摆位配置', allow_original=True)
                    image = bound_file(old.get('preview_path'), old.get('preview_sha256'), '沿用原图，摆位未变', 'image', allow_original=True)
                    review = bound_file(old.get('visual_review_path'), old.get('visual_review_sha256'), '原图初筛证据', allow_original=True)
                    source = bound_file(old.get('source_render_evidence_path'), old.get('source_render_evidence_sha256'), '原图渲染证据', allow_original=True)
                    hashes = all(isinstance(old.get(name), str) and len(old[name]) == 64 for name in (
                        'current_config_sha256', 'original_config_sha256', 'preview_sha256',
                        'visual_review_sha256', 'source_render_evidence_sha256'))
                    same_layout = bool(original and self.json(self.local(raw['config_path'])) == self.json(self.local(old['original_config_path'])))
                    reused = bool(hashes and image and review and source and same_layout
                                  and old['current_config_sha256'] == raw.get('config_sha256')
                                  and old.get('current_config_path') == raw.get('config_path'))
                    if reused:
                        render_status, reuse_reason = 'reused', str(old.get('reason') or '沿用原图，摆位未变；未补计新多视角像素条件。')
                        views = [dict(kind='reused', view_id='original_preview', image=image, visibility={}, preview_sha256=old['preview_sha256'])]
                        reuse_artifacts = [original, review, source]
                manual = reviews.get(scene_id,{})
                manual_status, manual_reason, manual_artifacts = 'pending', '', []
                if manual:
                    manual_artifact = bound_file(manual.get('review_path'), label='新图人工复看报告')
                    if manual_artifact:
                        manual_artifacts.append(manual_artifact)
                    valid_config = bool(config and manual.get('config_sha256') == raw.get('config_sha256') and raw.get('config_sha256'))
                    current_images = {v.get('preview_path'):v.get('preview_sha256') for v in report.get('views',[]) if isinstance(v,dict)} if report_bound else {}
                    reviewed_views = manual.get('reviewed_views',[])
                    images_bound = isinstance(reviewed_views,list) and bool(reviewed_views)
                    for item in reviewed_views if isinstance(reviewed_views,list) else []:
                        valid_image = (isinstance(item,dict) and isinstance(item.get('preview_sha256'),str)
                            and current_images.get(item.get('preview_path')) == item.get('preview_sha256')
                            and bool(bound_file(item.get('preview_path'),item.get('preview_sha256'))))
                        images_bound = images_bound and valid_image
                    reported_path = manual.get('render_evidence_path',manual.get('evidence_path'))
                    reported_hash = manual.get('render_evidence_sha256',manual.get('evidence_sha256'))
                    report_review_bound = (report_bound and isinstance(reported_hash,str) and len(reported_hash)==64
                        and reported_path == render.get('evidence_path')
                        and bool(bound_file(reported_path,reported_hash)))
                    complete_text = all(isinstance(manual.get(k),str) and manual[k].strip() for k in ('observation','recommendation'))
                    if (valid_config and view_scope_valid and (images_bound or report_review_bound) and complete_text
                            and manual.get('visual_status') in ('candidate','adjust','uncertain')):
                        manual_status = manual['visual_status']
                    else:
                        manual_status, manual_reason = 'historical', '人工记录未绑定当前配置与当前渲染图 / 内容不完整，保留为历史，需复看。'
                    if manual_status in ('candidate','adjust','uncertain'):
                        preferred = manual.get('preferred_views',manual.get('reviewed_views',[]))
                        report_views={v.get('preview_path'):v for v in report.get('views',[]) if isinstance(v,dict)}
                        front=[]
                        for item in preferred if isinstance(preferred,list) else []:
                            if not isinstance(item,dict):
                                continue
                            view=report_views.get(item.get('preview_path'),{})
                            if view.get('preview_sha256')!=item.get('preview_sha256'):
                                continue
                            image=bound_file(item.get('preview_path'),item.get('preview_sha256'),'人工复看选图 · '+str(view.get('view_id','')),'image')
                            if image:
                                front.append(dict(kind=view.get('kind'),view_id=view.get('view_id'),image=image,
                                    visibility=view.get('visibility',{}),preview_sha256=view['preview_sha256'],human_preferred=True))
                        if front:
                            used={v['image']['url'] for v in front}
                            views=front+[v for v in views if v['image']['url'] not in used]
                blocked = blocked_tasks.get(task_id,{})
                old_layout_applies = True
                if config and raw.get('original_config_path'):
                    old_config = bound_file(raw.get('original_config_path'),label='旧场景配置',allow_original=True)
                    original_layout = self.json(self.local(raw['original_config_path'])) if old_config else {}
                    current_layout = self.json(self.local(raw['config_path']))
                    old_layout_applies = bool(original_layout and {k:v for k,v in current_layout.items() if k!='obstacle'}
                                             == {k:v for k,v in original_layout.items() if k!='obstacle'})
                original_blocked = bool(blocked) and old_layout_applies and not repair_valid
                block_artifacts=[dict(a,label='旧版本 · '+a['label']) for a in blocked.get('artifacts',[])] if repair_valid else blocked.get('artifacts',[])
                values.append(dict(scene_id=scene_id, task_id=task_id, variant_id=variant_id, variant_name=name,
                    category=category, status=status, config_present=bool(config), semantic_geometry_passed=geometry,
                    config_sha256=raw.get('config_sha256') if config else None,
                    render_status=render_status, readable_pixels=bool(readable), views=views,
                    reason=reason, render_reason=str(render.get('reason') or ''),
                    old_visual_review=raw.get('visual_review') if isinstance(raw.get('visual_review'), dict) else {},
                    artifacts=[a for a in (config, evidence, render_evidence, repair_artifact) if a] + reuse_artifacts + manual_artifacts + block_artifacts,
                    preview_reused=reused, reuse_reason=reuse_reason,
                    new_visual_review_status=manual_status, new_visual_review=manual, new_visual_review_reason=manual_reason,
                    original_task_geometry_blocked=original_blocked,
                    old_original_task_geometry_blocked=bool(blocked), source_layout_repaired=repair_valid,
                    original_geometry_evidence_applies_to_current_layout=old_layout_applies,
                    source_layout_repair_pending=not repair_valid and (raw.get('source_layout_repaired') is True or not old_layout_applies),
                    baseline_scope=('repaired_source_layout_requires_new_baseline' if repair_valid
                                    else 'changed_source_layout_unverified' if not old_layout_applies else 'original_source_layout'),
                    native_stability_verified=False, trajectory_verified=False,
                    original_task_geometry_reason='原始任务几何阻塞：'+blocked['reason']+' 当前不能判定为可执行。' if original_blocked else '',
                    accepted=False))
        counts = {name: sum(row['status'] == name for row in values) for name in ('pending', 'retained', 'adjusted', 'needs_repair')}
        counts.update(configured=sum(row['config_present'] for row in values),
                      rendered=sum(row['render_status'] in ('ready', 'needs_view_review') and bool(row['views']) for row in values),
                      readable=sum(row['readable_pixels'] for row in values),
                      rendering=sum(row['render_status'] == 'rendering' for row in values),
                      reused=sum(row['preview_reused'] for row in values),
                      historical=sum(row['render_status'] == 'historical' for row in values))
        counts.update({'review_'+s:sum(row['new_visual_review_status']==s for row in values) for s in ('candidate','adjust','uncertain','historical')})
        counts.update(reviewed=sum(row['new_visual_review_status'] in ('candidate','adjust','uncertain') for row in values),
            review_pending=sum(row['new_visual_review_status']=='pending' and not row['preview_reused'] for row in values),
            blocked_original_scenes=sum(row['original_task_geometry_blocked'] for row in values))
        active_blocked_ids={r['task_id'] for r in values if r['original_task_geometry_blocked']}
        counts['blocked_original_tasks'] = len(active_blocked_ids)
        counts['source_layout_repaired']=sum(r['source_layout_repaired'] for r in values)
        counts['source_layout_repair_pending']=sum(r['source_layout_repair_pending'] for r in values)
        counts['new_baseline_required']=counts['source_layout_repaired']
        build_preview_complete = bool(values and counts['configured']==len(values) and counts['rendered']+counts['reused']==len(values))
        review_complete = bool(build_preview_complete and counts['reviewed']+counts['reused']==len(values))
        return dict(total=len(values), stage=str(data.get('stage') or '等待视觉指导摆位'), entries=values, counts=counts,
            build_preview_complete=build_preview_complete, visual_review_complete=review_complete,
            original_geometry_blocked_tasks=[value for key,value in blocked_tasks.items() if key in active_blocked_ids],
            source_progress_path=(folder/'progress.json').relative_to(self.root).as_posix(),
            variants=[dict(id=key, name=name, category=category) for key, name, category in SCENE_VARIANTS],
            notice='新摆位与新视角独立展示，旧154图初筛保留在下方。像素轮廓条件不是人工复看结论，也不代表原生物理或避障执行通过。')

    def adjustment_status(self) -> dict:
        for name in ('adjust05', 'adjust02'):
            path = self.root / ('reports/replicator_safety_upgrade/' + name + '/status.json')
            if path.is_file():
                return dict(self.json(path), source_status_path=path.relative_to(self.root).as_posix())
        return {}

    def visual_only_review(self, tasks: dict, base: dict | None = None) -> dict:
        """Publish real render sources for the manual-only round, not safety evidence.

        The overlay binds config -> renderer report -> actual PNG files. Successful
        source binding means only that these are the requested scene's images.
        No geometry, pixel quality, old review or execution outcome is inherited.
        """
        base = self.visual_placement(tasks) if base is None else base
        fields = ("scene_id", "task_id", "variant_id", "variant_name", "category", "config_present", "config_sha256")
        entries = []
        for source in base["entries"]:
            entry = {name: source.get(name) for name in fields}
            entry["views"] = [{name: view.get(name) for name in ("kind", "view_id", "image", "preview_sha256")}
                              for view in source.get("views", [])]
            entry.update(visual_only_source="base", overlay_applied=False, visual_review_status="unreviewed")
            entries.append(entry)
        by_id = {row["scene_id"]: row for row in entries}
        manifest_path = self.root / (VISUAL_ONLY_REPORTS + "current_scenes.json")
        manifest = self.json(manifest_path)
        overlays = manifest.get("entries", [])
        errors = []

        def bound(value, expected, prefix, suffix):
            if (not isinstance(value, str) or not value.startswith(prefix) or Path(value).is_absolute()
                    or ".." in Path(value).parts or not value.endswith(suffix) or not valid_digest(expected)):
                raise ValueError("源文件路径或 SHA 无效")
            path = self.local(value)
            if not path.relative_to(self.root).as_posix().startswith(prefix) or not path.is_file():
                raise ValueError("源文件不存在或越出本轮目录")
            stat = path.stat()
            key, stamp = str(path), (stat.st_mtime_ns, stat.st_size)
            if key not in self.hash_cache or self.hash_cache[key][0] != stamp:
                self.hash_cache[key] = (stamp, hashlib.sha256(path.read_bytes()).hexdigest())
            if self.hash_cache[key][1] != expected:
                raise ValueError("源文件 SHA 已变化")
            return path

        if not isinstance(overlays, list):
            errors.append({"scene_id": None, "reason": "候选清单 entries 格式无效"})
            overlays = []
        ids = [row.get("scene_id") for row in overlays if isinstance(row, dict) and isinstance(row.get("scene_id"), str)]
        for raw in overlays:
            scene_id = raw.get("scene_id") if isinstance(raw, dict) else None
            try:
                if not isinstance(scene_id, str) or scene_id not in by_id or ids.count(scene_id) != 1:
                    raise ValueError("未知或重复的场景标识")
                entry = by_id[scene_id]
                config_path = bound(raw.get("config_path"), raw.get("config_sha256"), VISUAL_ONLY_CONFIGS, ".json")
                report_path = bound(raw.get("render_report_path"), raw.get("render_report_sha256"), VISUAL_ONLY_REPORTS, "/multi_view.json")
                config, report = self.json(config_path), self.json(report_path)
                if (config.get("task_id") != entry["task_id"]
                        or any(report.get(name) != entry[name] for name in ("scene_id", "task_id", "variant_id"))
                        or report.get("config_sha256") != raw["config_sha256"]
                        or not isinstance(report.get("config_path"), str)
                        or self.local(report["config_path"]) != config_path):
                    raise ValueError("渲染报告与场景、任务或配置不匹配")
                source_views = report.get("views")
                if not isinstance(source_views, list) or not source_views or len(source_views) > 32:
                    raise ValueError("渲染报告没有有效的实际视图列表")
                views, seen = [], set()
                for view in source_views:
                    if (not isinstance(view, dict) or not isinstance(view.get("view_id"), str)
                            or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}", view["view_id"])
                            or view["view_id"] in seen):
                        raise ValueError("渲染视图标识无效或重复")
                    path = bound(view.get("preview_path"), view.get("preview_sha256"), VISUAL_ONLY_REPORTS, ".png")
                    if not path.is_relative_to(report_path.parent):
                        raise ValueError("PNG 不在此场景渲染报告的输出目录中")
                    artifact = self.artifact(path, "本轮实际渲染 · " + view["view_id"])
                    if artifact is None:
                        raise ValueError("PNG 不可访问")
                    artifact["kind"] = "image"
                    views.append({"kind": view.get("kind"), "view_id": view["view_id"], "image": artifact,
                                  "preview_sha256": view["preview_sha256"]})
                    seen.add(view["view_id"])
                entry.update(config_present=True, config_sha256=raw["config_sha256"], views=views,
                             config_path=config_path.relative_to(self.root).as_posix(),
                             render_report_path=report_path.relative_to(self.root).as_posix(),
                             render_report_sha256=raw["render_report_sha256"],
                             visual_only_source="overlay", overlay_applied=True)
            except (ValueError, KeyError, TypeError, OSError) as exc:
                reason = str(exc) if isinstance(exc, ValueError) else "候选来源不可读取"
                errors.append({"scene_id": scene_id if isinstance(scene_id, str) else None, "reason": reason})
                if isinstance(scene_id, str) and scene_id in by_id:
                    by_id[scene_id]["overlay_error"] = reason

        progress_path = self.root / (VISUAL_ONLY_REPORTS + "progress.json")
        progress = self.json(progress_path)
        counts = {"reviewed": 0, "passed": 0, "adjust": 0, "unreviewed": 0, "stale": 0}
        source_rows = {}
        duplicates = set()
        for row in progress.get("entries", []) if isinstance(progress.get("entries"), list) else []:
            if isinstance(row, dict) and isinstance(row.get("scene_id"), str) and row["scene_id"] in by_id:
                if row["scene_id"] in source_rows:
                    duplicates.add(row["scene_id"])
                source_rows[row["scene_id"]] = row
        review_entries = []
        for target in entries:
            scene_id = target["scene_id"]
            row = source_rows.get(scene_id, {})
            selected = {key: row[key] for key in ("scene_id", "status", "observation", "action", "reviewed_at",
                                                  "config_sha256", "binding_current", "previous_status", "review_path") if key in row}
            selected.update(scene_id=scene_id, status="unreviewed", binding_current=False)
            review_path = VISUAL_ONLY_REPORTS + "reviews/" + scene_id + ".json"
            record = self.json(self.root / review_path) if row.get("review_path") == review_path else {}
            # The explicit human record supplies the judgment. These checks only
            # invalidate old sources after config/image changes; they never judge pixels.
            valid_record = (scene_id not in duplicates and record.get("tool") == "record_visual_only_review.py"
                            and record.get("human_visual_only") is True
                            and record.get("status") in ("passed", "adjust")
                            and all(record.get(key) == target[key] for key in ("scene_id", "task_id", "variant_id")))
            images = record.get("viewed_images", [])
            current_views = {}
            for view in target["views"]:
                url = urlsplit(view.get("image", {}).get("url", ""))
                if not url.scheme and not url.netloc and url.path.startswith("/artifact/"):
                    key = (view["view_id"], unquote(url.path[len("/artifact/"):]))
                    current_views[key] = view["preview_sha256"]
            images_bound = (isinstance(images, list) and bool(images)
                            and all(isinstance(image, dict) and isinstance(image.get("view_id"), str)
                                    and isinstance(image.get("preview_path"), str) and valid_digest(image.get("preview_sha256"))
                                    and current_views.get((image["view_id"], image["preview_path"])) == image["preview_sha256"]
                                    for image in images))
            bound = (valid_record and target["config_present"] and valid_digest(target["config_sha256"])
                     and record.get("config_sha256") == target["config_sha256"] and images_bound)
            if valid_record:
                selected.update({key: record[key] for key in ("observation", "action", "reviewed_at", "config_sha256") if key in record})
            if bound:
                selected.update(status=record["status"], binding_current=True)
                selected.pop("previous_status", None)
                counts["reviewed"] += 1
                counts[record["status"]] += 1
            else:
                counts["unreviewed"] += 1
                if row.get("review_path") or row.get("status") in ("passed", "adjust") or row.get("previous_status"):
                    selected["previous_status"] = record.get("status", row.get("previous_status", row.get("status")))
                    counts["stale"] += 1
            target["visual_review_status"] = selected["status"]
            review_entries.append(selected)
        return {"scope": "visual only", "total": len(entries), "entries": entries, "counts": counts,
                "review_entries": review_entries, "updated_at": progress.get("updated_at"),
                "source_progress_path": progress_path.relative_to(self.root).as_posix(),
                "source_manifest_path": manifest_path.relative_to(self.root).as_posix(),
                "overlay_applied": sum(row["overlay_applied"] for row in entries), "overlay_errors": errors,
                "notice": "本轮仅按实际图片逐景审核；来源绑定不表示视觉通过，也不证明几何、物理或执行安全。"}

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
            self.report_logs = None
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
                                       "status": item["status"], "kind": item["kind"], "url": item["artifacts"][0]["url"]})
            # Saved neutral/startup evidence is visible, but never completes a task.
            neutral_results, diagnostic_results = {}, {}
            for path in self.root.glob("reports/replicator_execution*/**/result.json"):
                data = self.json(path)
                if "neutral_probe_passed" in data and "neutral_sequence_complete" in data:
                    neutral_results[path.parent] = data
                elif data.get("diagnostic_only") is True and data.get("task_id") in tasks:
                    diagnostic_results[path.parent] = data
            for name in ("neutral_probe.json", "result.json", "launch_failure.json", "operator_stop.json"):
                for path in self.root.glob(f"reports/replicator_execution*/**/{name}"):
                    if name == "result.json" and path.parent not in neutral_results and path.parent not in diagnostic_results:
                        continue
                    if ((name in ("neutral_probe.json", "launch_failure.json") and path.parent in neutral_results)
                            or (name == "launch_failure.json" and path.parent in diagnostic_results)):
                        continue
                    d = self.json(path)
                    tid = d.get("task_id") or self.infer_task(str(path), tasks)
                    if tid not in tasks:
                        continue
                    kind = ("diagnostic" if name == "result.json" and path.parent in diagnostic_results
                            else "neutral" if name in ("neutral_probe.json", "result.json") else "startup")
                    item = self.entry(path, d, kind)
                    tasks[tid]["evidence"].append(item)
                    events.append({"time": item["updated_at"], "task_id": tid, "label": item["path"],
                                   "status": item["status"], "kind": item["kind"], "url": item["artifacts"][0]["url"]})
                    if not tasks[tid]["latest"] or item["timestamp"] > tasks[tid]["latest"]["timestamp"]:
                        tasks[tid].update(latest=item, status="partial" if item["status"] in ("check_passed", "partial") else "failed")
            # Only the complete, current-format delivery report can certify all variants.
            certified_episodes = set()
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
                            certified_episodes.add(item["path"])
                            for key, label in (("comparison", "基线 / 新执行对照视频"), ("geometry_explanation", "避障几何说明视频")):
                                if entry.get(key):
                                    a = self.artifact(self.local(entry[key]), label)
                                    if a:
                                        item["artifacts"].append(a)
                            v["latest"] = item
                            for index, existing in enumerate(v["episodes"]):
                                if existing["path"] == item["path"]:
                                    v["episodes"][index] = item
                                    break
                            else:
                                v["episodes"].append(item)
                    if v["latest"]:
                        task["latest"] = v["latest"]
            for event in events:
                if event["label"] in certified_episodes:
                    event["status"] = "complete"
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
            scenes = self.scene_build(tasks)
            placement = self.visual_placement(tasks)
            visual_only = self.visual_only_review(tasks, placement)
            self.cached = {"generated_at": iso(), "poll_interval_ms": 3000,
                           "repo": {"branch": git("branch", "--show-current"), "head": git("rev-parse", "--short", "HEAD")},
                           "summary": {"total_tasks": len(values), "completed_tasks": sum(t["completed"] for t in values),
                                       "successful_variants": sum(v["status"] == "complete" or any(e["physical_success"] for e in v["episodes"]) for t in values for k, v in t["variants"].items() if k != "base"),
                                       "running_processes": len(processes)},
                           "tasks": values, "processes": processes, "scene_build": scenes,
                           "safety_upgrade": self.safety_upgrade(tasks),
                           "visual_triage": self.visual_triage(tasks, scenes),
                           "visual_placement": placement,
                           "visual_only_review": visual_only,
                           "trajectory_generation": self.trajectory_generation(visual_only),
                           "physical_execution": self.physical_execution(),
                           "adjustments": self.adjustment_status(),
                           "recent_events": sorted(events, key=lambda e: e["time"], reverse=True)[:16],
                           "notice": "本地证据与独立标注；标注不改变验收，规划 / 静置通过不等于任务完成。"}
            self.cached_at = time.monotonic()
            return self.cached


def allowed_artifact(rel: str) -> bool:
    if rel.startswith(PHYSICAL_RETRY4_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY4_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY4_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY4_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY4_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY4_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY5_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY5_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY5_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY5_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY5_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY5_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY6_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY6_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY6_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY6_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY6_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY6_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY8_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY8_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY8_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY8_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY8_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY8_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY7_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY7_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY7_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY7_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY7_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY7_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY3_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY3_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY3_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY3_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY3_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY3_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY2_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY2_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY2_PLANNING):
        return bool(re.fullmatch(r'(?:startup_recovery_[0-9]+/)?scenes/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY2_PLANNING):]))
    if rel.startswith(PHYSICAL_RETRY2_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY2_DIAGNOSTICS):]))
    if rel.startswith(PHYSICAL_RETRY_DEMOS):
        return bool(re.fullmatch(r'executions/[A-Za-z0-9_-]+/result\.json',rel[len(PHYSICAL_RETRY_DEMOS):]))
    if rel.startswith(PHYSICAL_RETRY_DIAGNOSTICS):
        return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/(?:diagnostic\.json|frame_[0-9]{2}\.jpg)',rel[len(PHYSICAL_RETRY_DIAGNOSTICS):]))
    for prefix in (PHYSICAL_RETRY_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY3_DETAIL_DIAGNOSTICS,
                   PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY3_TOP_DIAGNOSTICS,PHYSICAL_RETRY4_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY4_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY4_TOP_DIAGNOSTICS,PHYSICAL_RETRY5_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY5_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY5_TOP_DIAGNOSTICS,PHYSICAL_RETRY6_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY6_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY6_TOP_DIAGNOSTICS,PHYSICAL_RETRY7_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY8_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY7_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY8_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY7_TOP_DIAGNOSTICS,PHYSICAL_RETRY8_TOP_DIAGNOSTICS):
        if rel.startswith(prefix):
            return bool(re.fullmatch(r'scenes/[A-Za-z0-9_-]+/frame_[0-9]{2}\.jpg',rel[len(prefix):]))
    for prefix in (PHYSICAL_RETRY_DARK_DIAGNOSTICS,PHYSICAL_RETRY2_DARK_DIAGNOSTICS,PHYSICAL_RETRY3_DARK_DIAGNOSTICS,PHYSICAL_RETRY4_DARK_DIAGNOSTICS,PHYSICAL_RETRY5_DARK_DIAGNOSTICS,PHYSICAL_RETRY6_DARK_DIAGNOSTICS,PHYSICAL_RETRY7_DARK_DIAGNOSTICS,PHYSICAL_RETRY8_DARK_DIAGNOSTICS):
        if rel.startswith(prefix):
            return bool(re.fullmatch(r'(?:attempt_[0-9]{2}/)?scenes/[A-Za-z0-9_-]+/frame_[0-9]{2}\.jpg',rel[len(prefix):]))
    if rel.startswith(CAMERA_RETAKE_DEMOS):
        tail = rel[len(CAMERA_RETAKE_DEMOS):]
        return bool(re.fullmatch(r'(?:videos/u_shape_banana_bowl__[A-Za-z0-9_-]+\.mp4|executions/u_shape_banana_bowl__[A-Za-z0-9_-]+/result\.json)', tail))
    if rel.startswith(PHYSICAL_DEMOS):
        tail = rel[len(PHYSICAL_DEMOS):]
        return bool(re.fullmatch(r'(?:videos/[A-Za-z0-9_-]+\.mp4|executions/[A-Za-z0-9_-]+/(?:initialization_retry_01/)?result\.json)', tail))
    prefixes = ("reports/replicator_", "reports/avoidance_trials/replicator/", "datasets/replicator/",
                "datasets/avoidance_trials/replicator/", "docs/extensions/replicator/", "configs/extensions/replicator/",
                VISUAL_ONLY_REPORTS, VISUAL_ONLY_CONFIGS)
    suffix = Path(rel).suffix.lower()
    scene = rel.startswith(("reports/replicator_scene_build/", "configs/extensions/replicator/scene_build/",
                            "reports/replicator_safety_upgrade/visual_placement/",
                            "reports/replicator_safety_upgrade/visual_placement_final01/",
                            "configs/extensions/replicator/scene_variants/",
                            "assets/extensions/replicator/scene_build/"))
    return ((rel.startswith(prefixes) and suffix in {".json", ".log", ".txt", ".md", ".mp4", ".jpg", ".jpeg", ".png"})
            or (scene and suffix in {".usd", ".usda", ".yaml", ".yml"})
            or (re.match(r'^reports/replicator_trajectory_(?:generation|retry[0-9]*)_[0-9]{8}/scenes/[^/]+/', rel)
                and suffix in {".npz", ".npy", ".csv", ".html"}))


def handler_for(progress: Progress):
    class Handler(BaseHTTPRequestHandler):
        def json_response(self, status, value, head=False):
            data = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            if not head:
                self.wfile.write(data)

        def annotation_origin_valid(self):
            """Loopback-only Host and exact Origin prevent browser cross-site writes."""
            if self.headers.get("Sec-Fetch-Site", "") not in ("", "same-origin", "none"):
                return False
            host = self.headers.get("Host", "")
            try:
                target = urlsplit("http://" + host)
                if (target.hostname not in ("127.0.0.1", "localhost", "::1") or target.username or target.password
                        or target.path or target.query or target.fragment):
                    return False
                origin = self.headers.get("Origin")
                if origin is None:
                    return True
                source = urlsplit(origin)
                return (source.scheme == "http" and not source.username and not source.password
                        and not source.path and not source.query and not source.fragment
                        and source.hostname == target.hostname and (source.port or 80) == (target.port or 80))
            except ValueError:
                return False

        def do_POST(self):
            route = urlsplit(self.path).path
            if route not in ("/api/annotations", "/api/trajectory-annotations", "/api/physical-retry-annotations", "/api/physical-retry-round2-annotations", "/api/physical-retry-round3-annotations", "/api/physical-retry-round4-annotations", "/api/physical-retry-round5-annotations", "/api/physical-retry-round6-annotations", "/api/physical-retry-round7-annotations", "/api/physical-retry-round8-annotations"):
                self.json_response(404, {"error": "Unknown endpoint"})
                return
            try:
                if not self.annotation_origin_valid():
                    raise AnnotationError(403, "只接受此本地网页发起的标注保存。")
                if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
                    raise AnnotationError(415, "请使用 JSON 保存标注。")
                if self.headers.get("Transfer-Encoding"):
                    raise AnnotationError(400, "不支持此请求编码。")
                length = self.headers.get("Content-Length", "")
                if not re.fullmatch(r"[0-9]{1,9}", length):
                    raise AnnotationError(411, "缺少有效请求长度。")
                size = int(length)
                if size > ANNOTATION_MAX_BODY:
                    raise AnnotationError(413, "请求过大；调整方案最多 20000 字。")
                self.connection.settimeout(10)
                raw = self.rfile.read(size)
                if len(raw) != size:
                    raise AnnotationError(400, "请求内容不完整。")
                try:
                    payload = json.loads(raw.decode("utf-8"))
                except (UnicodeError, ValueError) as exc:
                    raise AnnotationError(400, "JSON 内容无效。") from exc
                save = progress.save_physical_retry_annotation if route == "/api/physical-retry-annotations" else progress.save_trajectory_annotation if route == "/api/trajectory-annotations" else progress.save_annotation
                entry=progress.save_physical_retry_annotation(payload,retry_round=8) if route=="/api/physical-retry-round8-annotations" else progress.save_physical_retry_annotation(payload,retry_round=7) if route=="/api/physical-retry-round7-annotations" else progress.save_physical_retry_annotation(payload,retry_round=6) if route=="/api/physical-retry-round6-annotations" else progress.save_physical_retry_annotation(payload,retry_round=5) if route=="/api/physical-retry-round5-annotations" else progress.save_physical_retry_annotation(payload,retry_round=4) if route=="/api/physical-retry-round4-annotations" else progress.save_physical_retry_annotation(payload,retry_round=3) if route=="/api/physical-retry-round3-annotations" else progress.save_physical_retry_annotation(payload,retry_round=2) if route=="/api/physical-retry-round2-annotations" else save(payload)
                self.json_response(200, {"entry": entry})
            except AnnotationError as exc:
                result = {"error": exc.message}
                if exc.status == 409:
                    result["entry"] = exc.entry
                self.json_response(exc.status, result)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (TimeoutError, OSError):
                self.json_response(503, {"error": "保存连接中断；请保留当前输入并重试。"})
            except Exception:
                self.json_response(500, {"error": "保存失败；请保留当前输入并检查本地服务。"})

        def do_HEAD(self):
            self.respond(head=True)

        def do_GET(self):
            self.respond(head=False)

        def respond(self, head=False):
            route = unquote(urlsplit(self.path).path)
            if route in ("/api/annotations", "/api/trajectory-annotations", "/api/physical-retry-annotations", "/api/physical-retry-round2-annotations", "/api/physical-retry-round3-annotations", "/api/physical-retry-round4-annotations", "/api/physical-retry-round5-annotations", "/api/physical-retry-round6-annotations", "/api/physical-retry-round7-annotations", "/api/physical-retry-round8-annotations"):
                try:
                    read = progress.physical_retry_annotations if route == "/api/physical-retry-annotations" else progress.trajectory_annotations if route == "/api/trajectory-annotations" else progress.annotations
                    data=progress.physical_retry_annotations(8) if route=="/api/physical-retry-round8-annotations" else progress.physical_retry_annotations(7) if route=="/api/physical-retry-round7-annotations" else progress.physical_retry_annotations(6) if route=="/api/physical-retry-round6-annotations" else progress.physical_retry_annotations(5) if route=="/api/physical-retry-round5-annotations" else progress.physical_retry_annotations(4) if route=="/api/physical-retry-round4-annotations" else progress.physical_retry_annotations(3) if route=="/api/physical-retry-round3-annotations" else progress.physical_retry_annotations(2) if route=="/api/physical-retry-round2-annotations" else read()
                    self.json_response(200, data, head=head)
                except AnnotationError as exc:
                    self.json_response(exc.status, {"error": exc.message}, head=head)
                return
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
                    if path.relative_to(progress.root).as_posix().startswith((PHYSICAL_DEMOS, CAMERA_RETAKE_DEMOS, PHYSICAL_RETRY_DEMOS, PHYSICAL_RETRY_DIAGNOSTICS, PHYSICAL_RETRY_DETAIL_DIAGNOSTICS, PHYSICAL_RETRY_DARK_DIAGNOSTICS,PHYSICAL_RETRY2_DEMOS,PHYSICAL_RETRY2_DIAGNOSTICS,PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY2_DARK_DIAGNOSTICS,PHYSICAL_RETRY2_PLANNING,PHYSICAL_RETRY3_DEMOS,PHYSICAL_RETRY3_DIAGNOSTICS,PHYSICAL_RETRY3_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY3_DARK_DIAGNOSTICS,PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY3_TOP_DIAGNOSTICS,PHYSICAL_RETRY3_PLANNING,PHYSICAL_RETRY4_DEMOS,PHYSICAL_RETRY4_DIAGNOSTICS,PHYSICAL_RETRY4_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY4_DARK_DIAGNOSTICS,PHYSICAL_RETRY4_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY4_TOP_DIAGNOSTICS,PHYSICAL_RETRY4_PLANNING,PHYSICAL_RETRY5_DEMOS,PHYSICAL_RETRY5_DIAGNOSTICS,PHYSICAL_RETRY5_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY5_DARK_DIAGNOSTICS,PHYSICAL_RETRY5_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY5_TOP_DIAGNOSTICS,PHYSICAL_RETRY5_PLANNING,PHYSICAL_RETRY6_DEMOS,PHYSICAL_RETRY6_DIAGNOSTICS,PHYSICAL_RETRY6_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY6_DARK_DIAGNOSTICS,PHYSICAL_RETRY6_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY6_TOP_DIAGNOSTICS,PHYSICAL_RETRY6_PLANNING,PHYSICAL_RETRY7_DEMOS,PHYSICAL_RETRY8_DEMOS,PHYSICAL_RETRY7_DIAGNOSTICS,PHYSICAL_RETRY8_DIAGNOSTICS,PHYSICAL_RETRY7_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY8_DETAIL_DIAGNOSTICS,PHYSICAL_RETRY7_DARK_DIAGNOSTICS,PHYSICAL_RETRY8_DARK_DIAGNOSTICS,PHYSICAL_RETRY7_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY8_OVERVIEW_DIAGNOSTICS,PHYSICAL_RETRY7_TOP_DIAGNOSTICS,PHYSICAL_RETRY8_TOP_DIAGNOSTICS,PHYSICAL_RETRY7_PLANNING,PHYSICAL_RETRY8_PLANNING)):
                        physical = progress.physical_execution()
                        retry_rows=[*physical['failure_retry']['entries'],*physical['failure_retry_round2']['entries'],*physical['failure_retry_round3']['entries'],*physical['failure_retry_round4']['entries'],*physical['failure_retry_round5']['entries'],*physical['failure_retry_round6']['entries'],*physical['failure_retry_round7']['entries'],*physical['failure_retry_round8']['entries']]
                        valid_urls = {entry.get(key) for entry in [*physical['entries'], *physical['camera_retakes']['entries'], *retry_rows]
                                      for key in ('video_url', 'result_url', 'diagnostic_report_url','planning_result_url')}
                        valid_urls.update(frame['url'] for entry in retry_rows for frame in entry.get('diagnostics',[]))
                        if '/artifact/' + quote(path.relative_to(progress.root).as_posix(), safe='/') not in valid_urls:
                            raise ValueError('Physical artifact is not bound by a valid result')
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
    print(f"RoboLaw 进度与场景标注页：http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
