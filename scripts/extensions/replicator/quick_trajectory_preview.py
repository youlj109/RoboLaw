#!/usr/bin/env python3
"""CPU-only projections of actual planned TCP samples on current scene images."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import time
from pathlib import Path
from urllib.parse import unquote, urlsplit
from urllib.request import urlopen

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[3]
DEFAULT = ROOT / "reports/replicator_trajectory_generation_20261026"
COLORS = ["#22d3ee", "#60a5fa", "#a78bfa", "#fbbf24", "#fb923c", "#34d399", "#f472b6"]
SCOPE = "真实规划 TCP 路径投影；背景为静态场景。线条不做遮挡处理，不表示机械臂实体、物理执行或避障验证。"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local(path):
    path = Path(path)
    if str(path).startswith("/workspace/"):
        path = Path(str(path).replace("/workspace/", "", 1))
    resolved = (ROOT / path).resolve()
    if not resolved.is_relative_to(ROOT):
        raise ValueError("Artifact outside repository")
    return resolved


def write_json(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temp.replace(path)


def camera_for_view(view):
    """Read authored camera opinions, without loading kitchen assets or Kit."""
    from pxr import Sdf
    route = unquote(urlsplit(view["image"]["url"]).path)
    if not route.startswith("/artifact/"):
        raise ValueError("Unsupported scene image URL")
    image_path = local(route.removeprefix("/artifact/"))
    if sha(image_path) != view["preview_sha256"]:
        raise ValueError("Scene image changed")
    report = image_path.parent / "multi_view.json"
    scene_path = None
    if report.is_file():
        for candidate in json.loads(report.read_text()).get("views", []):
            if candidate.get("preview_sha256") == view["preview_sha256"]:
                scene_path = local(candidate["scene_path"])
                break
    if scene_path is None:
        report = image_path.parent / "simulation_load.json"
        data = json.loads(report.read_text())
        # The current preview can be a copied camera retry, not scene.usda.
        scene_path = local(data.get("actual_loaded_candidate", data["scene_path"]))
    layer = Sdf.Layer.FindOrOpen(str(scene_path))
    def attr(name):
        spec = layer.GetAttributeAtPath("/World/overview_camera." + name)
        if spec is None or spec.default is None:
            raise ValueError("Missing authored camera attribute: " + name)
        return spec.default
    matrix = np.asarray(attr("xformOp:transform"), dtype=float)
    return image_path, dict(matrix=matrix.tolist(), focal=float(attr("focalLength")),
        horizontal=float(attr("horizontalAperture")), vertical=float(attr("verticalAperture")),
        source_path=str(scene_path.relative_to(ROOT)), source_sha256=sha(scene_path))


def project(points, camera, width, height):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    matrix = np.asarray(camera["matrix"])
    delta = points - matrix[3, :3]
    depth = -delta @ matrix[2, :3]
    safe = np.maximum(depth, 1e-5)
    return np.column_stack((width / 2 + delta @ matrix[0, :3] / safe * width * camera["focal"] / camera["horizontal"],
        height / 2 - delta @ matrix[1, :3] / safe * height * camera["focal"] / camera["vertical"], depth))


def trajectory_file(result, folder):
    artifacts = result.get("artifacts", {})
    iterable = artifacts.values() if isinstance(artifacts, dict) else artifacts
    for artifact in iterable:
        if isinstance(artifact, dict) and str(artifact.get("path", "")).endswith(".npz"):
            path = local(artifact["path"])
            if not path.is_file():
                path = folder / Path(artifact["path"]).name
            if path.is_file():
                if artifact.get("sha256") and sha(path) != artifact["sha256"]:
                    raise ValueError("Trajectory hash mismatch")
                return path
    for name in ("trajectory.npz", "partial_trajectory.npz"):
        if (folder / name).is_file():
            return folder / name
    return None


def target_points(result, folder):
    extra = folder / "trajectory.json"
    if not extra.is_file():
        extra = folder / "partial_trajectory.json"
    data = json.loads(extra.read_text()) if extra.is_file() else result
    targets = data.get("targets", result.get("targets", []))
    if isinstance(targets, dict):
        targets = [dict(phase=k, pose=v) if isinstance(v, list) else dict(v, phase=k) for k, v in targets.items()]
    output = []
    for target in targets:
        if not isinstance(target, dict):
            continue
        pose = target.get("pose", target.get("pose_world", target.get("target_pose_world", target.get("target"))))
        if pose is None:
            continue
        values = np.asarray(pose, dtype=float)
        xyz = values[:3, 3] if values.shape == (4, 4) else values[:3] if values.ndim == 1 else None
        if xyz is not None and np.isfinite(xyz).all():
            output.append(dict(phase=target.get("phase", target.get("name", "target")), xyz=xyz.tolist()))
    return output


def render(result_path, scene):
    result = json.loads(result_path.read_text())
    if result.get("status") not in ("generated", "failed", "skipped"):
        return False
    if result.get("config_sha256") != scene["config_sha256"]:
        raise ValueError("Result does not match current scene")
    folder = result_path.parent
    result_hash = sha(result_path)
    old = folder / "preview.json"
    if old.is_file():
        previous = json.loads(old.read_text())
        if previous.get("result_sha256") == result_hash and previous.get("config_sha256") == scene["config_sha256"]:
            return False
    path = trajectory_file(result, folder)
    points = np.empty((0, 3)); phases = []; dt = .05
    if path:
        with np.load(path, allow_pickle=False) as values:
            raw = values["tcp_world"]
            points = raw[:, :3, 3] if raw.ndim == 3 else raw[:, :3]
            phases = values["phase"].astype(str).tolist()
            dt = float(values["dt"]) if "dt" in values else dt
        if not np.isfinite(points).all() or len(points) != len(phases):
            raise ValueError("Malformed planned TCP samples")
    # Keep phase boundaries and evenly spaced samples for responsive playback.
    selected = np.unique(np.r_[np.linspace(0, max(0, len(points)-1), min(1000, len(points))).astype(int),
        [i for i in range(1, len(phases)) if phases[i] != phases[i-1]]]).astype(int)
    phase_order = list(dict.fromkeys(phases))
    palette = {p: COLORS[i % len(COLORS)] for i, p in enumerate(phase_order)}
    targets = target_points(result, folder)
    views, errors = [], []
    for view in scene["views"]:
        try:
            image_path, camera = camera_for_view(view)
            with Image.open(image_path) as source:
                width, height = source.size
            projected = project(points[selected], camera, width, height)
            projected_targets = project([t["xyz"] for t in targets], camera, width, height)
            views.append(dict(id=view["view_id"], image=view["image"]["url"], source_path=str(image_path.relative_to(ROOT)),
                source_sha256=view["preview_sha256"], width=width, height=height, camera=camera,
                points=np.round(projected, 3).tolist(), targets=[dict(t, pixel=p.tolist()) for t, p in zip(targets, projected_targets)]))
        except Exception as error:
            errors.append(dict(view_id=view["view_id"], error=str(error)))
    if not views:
        raise ValueError("No camera-bound views: " + str(errors))
    scope = SCOPE
    if result.get('annotation_only'):
        scope = '本轮仅标注，未重新求解；图中路径和请求目标来自上一轮失败记录。' + scope
    data = dict(scene_id=scene["scene_id"], status=result["status"], scope=scope,
        phase=[phases[i] for i in selected], time=[round(float(i)*dt, 4) for i in selected], colors=palette,
        views=views, failed_phase=result.get("failed_phase", result.get("stage", result.get("phase", ""))) if result["status"] != "generated" else "",
        failure_reason=result.get("failure_reason", result.get("reason", "")))
    encoded = json.dumps(data, ensure_ascii=False).replace("<", "\\u003c")
    page = HTML.replace("__DATA__", encoded).replace("__TITLE__", html.escape(scene["scene_id"]))
    html_path = folder / "trajectory_preview.html"
    html_path.write_text(page)
    # A real projection, not a synthetic motion video. Color distinguishes phases.
    view = views[0]
    canvas = Image.open(local(view["source_path"])).convert("RGB")
    draw = ImageDraw.Draw(canvas)
    for i in range(1, len(view["points"])):
        p, q = view["points"][i-1], view["points"][i]
        if p[2] > .01 and q[2] > .01:
            draw.line([tuple(p[:2]), tuple(q[:2])], fill=palette[data["phase"][i]], width=4)
    for target in view["targets"]:
        x, y, z = target["pixel"]
        if z > .01:
            draw.line((x-6, y, x+6, y), fill="#fb7185", width=3)
            draw.line((x, y-6, x, y+6), fill="#fb7185", width=3)
    draw.rectangle((0, 0, canvas.width, 54), fill="#101827")
    label = 'ANNOTATION ONLY; PREVIOUS PATH' if result.get('annotation_only') else 'PLANNED TCP ONLY'
    draw.text((12, 8), f"{label} | {result['status']} | {len(points)} samples", fill="white")
    draw.text((12, 29), "Static background; no occlusion / physics / avoidance validation", fill="#b8cadc")
    png_path = folder / "trajectory_preview.png"
    canvas.save(png_path)
    def artifact(p):
        return dict(path=str(p.relative_to(ROOT)), sha256=sha(p))
    write_json(old, dict(schema_version=1, scene_id=scene["scene_id"], config_sha256=scene["config_sha256"],
        result_sha256=result_hash, trajectory_sha256=sha(path) if path else None, scope=scope,
        artifacts=dict(preview=artifact(png_path), interactive=artifact(html_path)),
        samples=len(points), shown_samples=len(selected), views=views, warnings=errors))
    return True


HTML = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>__TITLE__ · 规划路径</title>
<style>*{box-sizing:border-box}body{margin:0;background:#0d1420;color:#e9f0fa;font:14px system-ui,sans-serif}header,footer{padding:12px 16px}h1{font-size:16px;margin:0 0 6px}p{color:#aebed4;margin:5px 0;line-height:1.5}button,select{background:#21324a;color:white;border:1px solid #465b79;border-radius:6px;padding:7px 10px}#view{width:100%;display:block;background:#080e18}#toolbar{display:flex;gap:10px;align-items:center;flex-wrap:wrap}input{flex:1;min-width:120px}#legend{display:flex;gap:14px;flex-wrap:wrap;margin-top:8px}#frame{min-width:180px}</style>
<header><h1 id="title"></h1><p id="scope"></p><div id="toolbar"><select id="camera" aria-label="视角"></select><button id="play">播放路径</button><input id="seek" type="range" min="0" value="0" aria-label="轨迹时间"><span id="frame"></span></div><div id="legend"></div><p id="failure"></p></header><canvas id="view"></canvas><footer>彩色线：已生成的末端路径；红色十字：请求目标。背景机械臂保持截图姿态，播放圆点代表规划 TCP。视图投影不处理实体遮挡。</footer>
<script>const D=__DATA__,C=document.getElementById('view'),ctx=C.getContext('2d'),S=document.getElementById('seek'),V=document.getElementById('camera'),I=new Image();let v=D.views[0],playing=false,last=0;
document.getElementById('title').textContent=D.scene_id+' · '+(D.status==='generated'?'轨迹已生成':'未生成完整轨迹');document.getElementById('scope').textContent=D.scope;
document.getElementById('failure').textContent=D.failed_phase||D.failure_reason?('失败阶段：'+D.failed_phase+' '+D.failure_reason):'';
D.views.forEach((v,i)=>{let o=document.createElement('option');o.value=i;o.textContent=v.id;V.append(o)});Object.entries(D.colors).forEach(([phase,color])=>{let s=document.createElement('span');s.textContent='● '+phase;s.style.color=color;document.getElementById('legend').append(s)});
S.max=Math.max(0,D.phase.length-1);S.value=S.max;document.getElementById('play').disabled=!D.phase.length;
function draw(){ctx.clearRect(0,0,C.width,C.height);if(I.complete&&I.naturalWidth)ctx.drawImage(I,0,0,C.width,C.height);let end=+S.value;
for(let i=1;i<v.points.length;i++){let a=v.points[i-1],b=v.points[i];if(a[2]<=.01||b[2]<=.01)continue;ctx.globalAlpha=i<=end?1:.23;ctx.strokeStyle=D.colors[D.phase[i]];ctx.lineWidth=4;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke()}ctx.globalAlpha=1;
v.targets.forEach(t=>{let[x,y,z]=t.pixel;if(z<=.01)return;ctx.strokeStyle='#fb7185';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x-7,y);ctx.lineTo(x+7,y);ctx.moveTo(x,y-7);ctx.lineTo(x,y+7);ctx.stroke();ctx.font='14px sans-serif';ctx.fillStyle='#fff';ctx.strokeStyle='#000';ctx.lineWidth=3;ctx.strokeText(t.phase,x+10,y-10);ctx.fillText(t.phase,x+10,y-10)});
if(v.points[end]&&v.points[end][2]>.01){let p=v.points[end];ctx.beginPath();ctx.arc(p[0],p[1],8,0,Math.PI*2);ctx.fillStyle='white';ctx.fill();ctx.strokeStyle='#111827';ctx.lineWidth=3;ctx.stroke()}document.getElementById('frame').textContent=D.phase.length?`${D.time[end].toFixed(2)} s · ${D.phase[end]} · ${end+1}/${D.phase.length}`:'当前没有成功规划的路径段';}
function camera(){v=D.views[+V.value||0];C.width=v.width;C.height=v.height;I.src=v.image;draw()}I.onload=draw;V.onchange=camera;S.oninput=()=>{playing=false;document.getElementById('play').textContent='播放路径';draw()};document.getElementById('play').onclick=()=>{playing=!playing;if(playing&&+S.value>=+S.max)S.value=0;document.getElementById('play').textContent=playing?'暂停':'播放路径'};
function tick(t){if(playing&&t-last>40){S.value=Math.min(+S.max,+S.value+1);last=t;draw();if(+S.value>=+S.max){playing=false;document.getElementById('play').textContent='重新播放'}}requestAnimationFrame(tick)}camera();requestAnimationFrame(tick);
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--scene-views", type=Path, help="Exact config-bound views for an authorized scene-change run")
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.scene_views:
        snapshot = json.loads(args.scene_views.read_text())
        scenes = {s['scene_id']: s for s in snapshot['entries']}
    else:
        snapshot = json.load(urlopen("http://127.0.0.1:8765/api/progress", timeout=30))
        scenes = {s["scene_id"]: s for s in snapshot["visual_only_review"]["entries"]}
    failures = {}
    while True:
        count = 0
        for result in sorted(args.output.glob("scenes/*/result.json")):
            if result.parent.name not in scenes:
                continue
            try:
                count += int(render(result, scenes[result.parent.name]))
                failures.pop(result.parent.name, None)
            except Exception as error:
                if failures.get(result.parent.name) != str(error):
                    print("PREVIEW_ERROR", result.parent.name, str(error), flush=True)
                failures[result.parent.name] = str(error)
        if count:
            print("PREVIEWS_UPDATED", count, flush=True)
        if not args.watch:
            return int(bool(failures))
        progress = args.output / "progress.json"
        if progress.is_file():
            value = json.loads(progress.read_text())
            if value.get("status") in ("completed", "complete") or value.get("stage") in ("completed", "complete"):
                print("PREVIEW_BATCH_FINISHED", len(failures), "errors", flush=True)
                return int(bool(failures))
        time.sleep(2)


if __name__ == "__main__":
    raise SystemExit(main())
