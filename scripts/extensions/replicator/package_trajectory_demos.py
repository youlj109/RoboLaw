#!/usr/bin/env python3
"""Validate and package 154 source-bound physical execution attempt videos.

Manifest contract (renderer-owned, never rewritten): schema_version=1,
kind=physical_execution, source_run, source_progress_sha256, entries. Each entry supplies scene_id,
task_id, variant_id, result_path/result_sha256, config_path/config_sha256,
trajectory_path/trajectory_sha256, video_path/video_sha256, fps, frame_count,
width, height and duration_seconds. Physical fields are execution_result_path,
execution_result_sha256, planning_status, execution_status, task_success,
attempts and executed. video_path is videos/<scene_id>.mp4. Failed/error trials
are retained; an initialization error with executed=false is never a success.

Requires numpy and OpenCV. Every video is decoded fully on CPU. No simulation,
rendering, planning, upload or git operation is performed by this script.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
import tempfile
import zipfile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SOURCE = ROOT / 'reports/replicator_trajectory_retry10_20261028'
DEFAULT_DEMOS = ROOT / 'exports/replicator_154_physical_demos_20261028'
PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')
TOTAL = 154
NOTICE = '每景一次实际物理仿真尝试，失败与初始化错误均保留；安全与过程避障未单独验收。'
KNOWN_WARNINGS = {
    'g_shape_banana_bowl__human_default_mpfb',
    'u_shape_bowl_sink__robot_narrow_opening',
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def repo_path(value):
    result = (ROOT / str(value).removeprefix('/workspace/')).resolve()
    result.relative_to(ROOT)
    return result


def demo_path(value, demos):
    value = str(value)
    result = (demos / value).resolve() if value.startswith(('executions/', 'videos/')) else repo_path(value)
    result.relative_to(demos)
    return result


def bound_file(path, expected, label):
    require(isinstance(expected, str) and len(expected) == 64, label + ': missing SHA256')
    require(path.is_file() and path.stat().st_size > 0, label + ': missing/empty file')
    require(sha(path) == expected, label + ': SHA256 mismatch')


def verify_trajectory(path, result):
    with np.load(path, allow_pickle=False) as archive:
        q, tcp, phases = archive['q'], archive['tcp_world'], archive['phase']
        gripper, dt = archive['gripper'], float(archive['dt'])
        count = len(q)
        require(count > 0 and q.shape == (count, 7), 'Invalid joint trajectory shape')
        require(tcp.shape == (count, 4, 4) and phases.shape == (count,)
                and gripper.shape == (count,), 'Invalid trajectory array dimensions')
        require(np.isfinite(q).all() and np.isfinite(tcp).all() and np.isfinite(gripper).all()
                and math.isfinite(dt) and dt > 0, 'Invalid trajectory numeric values')
        ordered = [str(value) for index, value in enumerate(phases)
                   if index == 0 or value != phases[index - 1]]
        require(tuple(ordered) == PHASES, 'Trajectory must contain all six contiguous phases')
        require(result.get('trajectory_samples') == count, 'Result/NPZ sample count mismatch')
    return dict(samples=count, dt_seconds=dt, phases=list(PHASES))


def decode_video(path, entry):
    """Count real decoded frames; container metadata alone is insufficient."""
    expected_frames = entry['frame_count']
    width, height = entry['width'], entry['height']
    fps, duration = float(entry['fps']), float(entry['duration_seconds'])
    require(type(expected_frames) is int and expected_frames > 1, 'Invalid expected frame count')
    require(type(width) is int and type(height) is int and width > 0 and height > 0,
            'Invalid expected video dimensions')
    require(math.isfinite(fps) and fps > 0 and math.isfinite(duration) and duration > 0,
            'Invalid expected video timing')
    capture = cv2.VideoCapture(str(path))
    require(capture.isOpened(), 'Video cannot be opened: ' + str(path))
    count = 0
    try:
        observed_fps = float(capture.get(cv2.CAP_PROP_FPS))
        reported_frames = float(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        require(math.isfinite(observed_fps) and abs(observed_fps - fps) <= max(.001, fps * .001),
                'Video FPS mismatch: ' + str(path))
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            require(frame.shape[:2] == (height, width), 'Decoded frame dimensions changed: ' + str(path))
            count += 1
    finally:
        capture.release()
    require(count == expected_frames, f'Decoded frame count mismatch: {path}: {count} != {expected_frames}')
    require(math.isfinite(reported_frames) and abs(reported_frames - count) <= .01,
            'Container/decoded frame count mismatch: ' + str(path))
    observed_duration = count / observed_fps
    require(abs(observed_duration - duration) <= max(.01, 1 / observed_fps),
            'Decoded video duration mismatch: ' + str(path))
    return dict(decoded_frames=count, fps=observed_fps, duration_seconds=observed_duration,
                width=width, height=height, fully_decoded=True)


def validate_entry(entry, source_row, source, demos):
    sid = entry['scene_id']
    for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256'):
        require(entry.get(key) == source_row.get(key), sid + ': source identity mismatch: ' + key)
    result_path = source / 'scenes' / sid / 'result.json'
    require(repo_path(entry['result_path']) == result_path, sid + ': result is not the selected source-run result')
    bound_file(result_path, entry['result_sha256'], sid + ' result')
    result = read(result_path)
    require(result.get('status') == 'generated' and result.get('stage') == 'complete'
            and result.get('trajectory_complete') is True and result.get('planning_only') is True,
            sid + ': incomplete or non-planning result')
    require(result.get('executed') is False and result.get('task_success_verified') is False,
            sid + ': unexpected execution certification')
    for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256'):
        require(result.get(key) == entry[key], sid + ': manifest/result identity mismatch: ' + key)
    config = repo_path(entry['config_path'])
    require(config == repo_path(source_row['config_path']) == repo_path(result['config_path']),
            sid + ': source config path mismatch')
    bound_file(config, entry['config_sha256'], sid + ' config')
    phases = result.get('phases', [])
    require(tuple(row.get('phase') for row in phases) == PHASES
            and all(row.get('status') == 'generated' and row.get('solver_success', True) is True for row in phases),
            sid + ': incomplete successful phase evidence')
    artifact = result['artifacts']['trajectory']
    trajectory = repo_path(entry['trajectory_path'])
    require(trajectory == repo_path(artifact['path']) and entry['trajectory_sha256'] == artifact['sha256'],
            sid + ': NPZ does not match selected result')
    bound_file(trajectory, entry['trajectory_sha256'], sid + ' trajectory')
    trajectory_check = verify_trajectory(trajectory, result)
    require(entry.get('planning_status') == 'generated', sid + ': wrong planning status')
    require(entry.get('execution_status') in ('success', 'failed', 'error'), sid + ': wrong execution status')
    require(type(entry.get('task_success')) is bool and type(entry.get('executed')) is bool,
            sid + ': physical outcome flags must be explicit booleans')
    require(type(entry.get('attempts')) is int and entry['attempts'] == 1,
            sid + ': expected exactly one physical attempt')
    require(entry['task_success'] is (entry['execution_status'] == 'success'),
            sid + ': success status and physical task predicate disagree')
    require(entry['executed'] or entry['execution_status'] == 'error',
            sid + ': a trial that did not execute must remain an error')
    physical_path = demo_path(entry['execution_result_path'], demos)
    require(physical_path != result_path, sid + ': planning result cannot substitute for physical result')
    bound_file(physical_path, entry['execution_result_sha256'], sid + ' physical result')
    physical = read(physical_path)
    require(physical.get('kind') == 'physical_execution', sid + ': missing actual execution result kind')
    for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256', 'planning_status',
                'execution_status', 'task_success', 'attempts', 'executed'):
        require(physical.get(key) == entry.get(key), sid + ': physical result mismatch: ' + key)
    require(repo_path(physical['config_path']) == config, sid + ': physical config path mismatch')
    require(physical.get('planning_result_sha256') == entry['result_sha256']
            and repo_path(physical['planning_result_path']) == result_path,
            sid + ': physical result does not bind selected planning result')
    require(physical.get('trajectory_sha256') == entry['trajectory_sha256']
            and repo_path(physical['trajectory_path']) == trajectory,
            sid + ': physical result does not bind selected NPZ')
    require(entry['video_path'] == 'videos/' + sid + '.mp4', sid + ': unexpected video path')
    video = (demos / entry['video_path']).resolve()
    video.relative_to(demos)
    require(demo_path(physical['video_path'], demos) == video
            and physical.get('video_sha256') == entry['video_sha256'],
            sid + ': video is not bound by the actual execution result')
    for key in ('fps', 'frame_count', 'width', 'height', 'duration_seconds'):
        require(physical.get(key) == entry.get(key), sid + ': execution video metadata mismatch: ' + key)
    bound_file(video, entry['video_sha256'], sid + ' video')
    video_check = decode_video(video, entry)
    # Detect a renderer writing/replacing its file while it is being checked.
    bound_file(video, entry['video_sha256'], sid + ' decoded video')
    warnings = result.get('scene_adjustment', {}).get('scene_quality_issues', [])
    require(isinstance(warnings, list) and all(isinstance(value, str) for value in warnings),
            sid + ': invalid quality-warning evidence')
    if sid in KNOWN_WARNINGS:
        require(bool(warnings), sid + ': required historical quality warning is missing')
    return dict(scene_id=sid, task_id=entry['task_id'], variant_id=entry['variant_id'],
                variant_name=source_row.get('variant_name', entry['variant_id']),
                category=source_row.get('category'), status='validated', video_path=entry['video_path'],
                planning_status='generated', execution_status=entry['execution_status'],
                task_success=entry['task_success'], attempts=entry['attempts'], executed=entry['executed'],
                execution_result_path=entry['execution_result_path'],
                execution_result_sha256=entry['execution_result_sha256'],
                execution_result_package_path='results/' + sid + '.json',
                failure_reason=physical.get('failure_reason', physical.get('error', '')),
                video_sha256=entry['video_sha256'], video_bytes=video.stat().st_size,
                result_path=entry['result_path'], result_sha256=entry['result_sha256'],
                config_path=entry['config_path'], config_sha256=entry['config_sha256'],
                trajectory_path=entry['trajectory_path'], trajectory_sha256=entry['trajectory_sha256'],
                trajectory=trajectory_check, video=video_check, historical_quality_issues=warnings)


def index_html(entries):
    labels = {'success': '成功', 'failed': '任务失败', 'error': '错误'}
    data = json.dumps(entries, ensure_ascii=False).replace('<', '\\u003c')
    rows = ''.join('<tr><td><a href="' + html.escape(row['video_path'], quote=True) + '">'
                   + html.escape(row['task_id'].removeprefix('kitchen_bench_replicator_'))
                   + '</a></td><td>' + html.escape(row['variant_name']) + '<br><small>'
                   + html.escape(row['variant_id']) + '</small></td><td>' + labels[row['execution_status']]
                   + (' · 未进入执行' if not row['executed'] else '')
                   + '<br><a href="' + html.escape(row['execution_result_package_path'], quote=True)
                   + '">执行记录</a></td><td>'
                   + ('；'.join(html.escape(value) for value in row['historical_quality_issues']) or '—')
                   + '</td></tr>' for row in entries)
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>RoboLaw · 154 个物理仿真视频</title>
<style>body{font:16px/1.6 system-ui,sans-serif;max-width:1280px;margin:24px auto;padding:0 20px;background:#101821;color:#dde7f2}a{color:#8fc7ff}h1{font-size:26px}input,select,button{font:inherit;padding:8px;background:#1d2b3b;color:inherit;border:1px solid #496078;border-radius:5px}video{width:100%;max-height:68vh;background:#000;margin-top:16px}.notice{padding:12px 16px;background:#30301e;border-left:4px solid #d6b34c}.controls{display:flex;gap:10px;flex-wrap:wrap;margin:16px 0}.controls input{flex:1;min-width:180px}table{width:100%;border-collapse:collapse;font-size:14px}td,th{text-align:left;padding:10px;border-bottom:1px solid #35465b}small{color:#97aabd}#quality:empty{display:none}#quality{color:#f3d581;background:#363020;padding:10px}details{margin-top:20px}</style>
<h1>RoboLaw · 154 个物理仿真视频</h1><p class="notice">''' + NOTICE + ''' 规划成功与物理任务成功分别记录；初始化错误会标明“未进入执行”。下方历史场景质量提示仍然适用。</p>
<p>下载后解压并打开此文件即可浏览，无需网络。<a href="manifest.json">来源清单</a> · <a href="validation.json">完整性校验</a> · <a href="SHA256SUMS.txt">文件哈希</a></p>
<div class="controls"><input id="search" aria-label="搜索任务或变体" placeholder="搜索任务、变体或场景编号"><select id="task" aria-label="按任务筛选"><option value="">所有原始任务</option></select><select id="variant" aria-label="按变体筛选"><option value="">所有变体</option></select><button id="prev">上一个</button><button id="next">下一个</button></div>
<p id="title"></p><p id="quality"></p><video id="player" controls preload="metadata" playsinline></video><p id="position"></p><select id="scenes" style="width:100%" aria-label="选择场景"></select>
<details><summary>全部任务与变体（154 项，含历史质量提示）</summary><table><thead><tr><th>原始任务 / 视频</th><th>变体</th><th>物理执行结果</th><th>历史质量提示</th></tr></thead><tbody>''' + rows + '''</tbody></table></details>
<script>const DATA=''' + data + ''';const $=id=>document.getElementById(id);let list=DATA,at=0;
for(const [key,id]of[['task_id','task'],['variant_id','variant']])for(const value of[...new Set(DATA.map(x=>x[key]))].sort()){const o=document.createElement('option');o.value=value;o.textContent=value.replace('kitchen_bench_replicator_','');$(id).append(o)}
function show(){const row=list[at];$('position').textContent=list.length?`${at+1} / ${list.length} · 共154个`:'没有匹配项';$('prev').disabled=!row||at===0;$('next').disabled=!row||at===list.length-1;$('scenes').value=String(at);$('title').textContent=row?row.scene_id+' · '+({success:'成功',failed:'任务失败',error:'错误'}[row.execution_status])+(row.executed?'':' · 未进入执行'):'';$('quality').textContent=row?.historical_quality_issues.length?'历史质量提示：'+row.historical_quality_issues.join('；'):'';if(row){$('player').src=row.video_path;$('player').poster=''}else{$('player').removeAttribute('src');$('player').load()}}
function filter(){const q=$('search').value.toLowerCase();list=DATA.filter(x=>(!$('task').value||x.task_id===$('task').value)&&(!$('variant').value||x.variant_id===$('variant').value)&&(!q||(x.scene_id+' '+x.variant_name).toLowerCase().includes(q)));at=0;$('scenes').replaceChildren(...list.map((x,i)=>{const o=document.createElement('option');o.value=String(i);o.textContent=x.scene_id+(x.historical_quality_issues.length?' · 质量提示':'');return o}));show()}
$('search').oninput=filter;$('task').onchange=filter;$('variant').onchange=filter;$('scenes').onchange=()=>{at=Number($('scenes').value);show()};$('prev').onclick=()=>{at--;show()};$('next').onclick=()=>{at++;show()};filter();</script></html>'''


def readme(entries, source):
    counts = Counter(row['execution_status'] for row in entries)
    not_executed = sum(not row['executed'] for row in entries)
    warnings = [row for row in entries if row['scene_id'] in KNOWN_WARNINGS]
    warning_text = '\n'.join('- `' + row['scene_id'] + '`：' + '；'.join(row['historical_quality_issues'])
                             for row in warnings)
    return f'''# RoboLaw 154 个物理仿真视频

**{NOTICE}** 实际执行结果按任务判据记录；没有完成独立的长期稳定性、接触安全或运行过程避障验收。

解压后打开 `index.html`，按任务或变体筛选并播放；也可直接打开 `videos/` 下的 MP4。共 14 个原始任务 × 11 种增广变体 = 154 个视频。物理任务成功 {counts['success']}、失败 {counts['failed']}、错误 {counts['error']}；其中 {not_executed} 景标记为未进入物理执行。每景仅尝试一次，失败保留。

规划输入来源：`{source.relative_to(ROOT)}`。每个视频在 `manifest.json` 中绑定规划结果、场景配置、NPZ 轨迹及实际执行结果的 SHA256。实际执行 JSON 位于 `results/`，明确记录 `planning_status`、`execution_status`、`task_success`、`attempts` 和 `executed`。初始化错误若 `executed=false`，不计为完成物理执行，视频仅能来自真实现场画面，不能使用占位图补足。

`validation.json` 记录真实逐帧解码的帧数、尺寸、时长，以及输入六阶段轨迹完整性。原始规划 NPZ 和场景资产未打包。

## 保留的历史质量提示

{warning_text}

规划完成不会消除上述质量问题。其余视频没有列出这两条特定历史提示，并不表示完成了物理安全验收。

## 校验文件

`SHA256SUMS.txt` 覆盖本包中除自身外的所有文件；在解压目录可运行 `sha256sum -c SHA256SUMS.txt`。视频逐帧解码和文件哈希校验只能证明文件完整性及来源绑定，不能独立证明回放画面与物理场景执行一致。
'''


def validate_collection(demos, source, workers=2):
    manifest_path = demos / 'manifest.json'
    manifest_digest = sha(manifest_path)
    manifest = read(manifest_path)
    require(manifest.get('schema_version') == 1 and manifest.get('kind') == 'physical_execution',
            'Expected schema_version=1 physical_execution manifest')
    require(repo_path(manifest['source_run']) == source, 'Manifest source_run mismatch')
    bound_file(source / 'progress.json', manifest['source_progress_sha256'], 'Source progress')
    progress = read(source / 'progress.json')
    source_rows = {row['scene_id']: row for row in progress['entries']}
    require(progress.get('stage') == 'complete' and len(progress['entries']) == len(source_rows) == TOTAL
            and all(row['status'] == 'generated' for row in source_rows.values()),
            'Source run must be exactly 154 complete generated scenes')
    entries = manifest['entries']
    require(isinstance(entries, list) and len(entries) == TOTAL, 'Exactly 154 video entries are required')
    ids = [row['scene_id'] for row in entries]
    require(len(set(ids)) == TOTAL and set(ids) == set(source_rows), 'Duplicate/missing/foreign scene IDs')
    require({path.name for path in (demos / 'videos').glob('*.mp4')} == {sid + '.mp4' for sid in ids},
            'Video folder must contain exactly the selected 154 MP4 names')
    with ThreadPoolExecutor(max_workers=workers) as pool:
        checked = list(pool.map(lambda row: validate_entry(row, source_rows[row['scene_id']], source, demos), entries))
    checked.sort(key=lambda row: row['scene_id'])
    require(KNOWN_WARNINGS <= set(ids), 'Required historical warning scenes missing')
    # Freeze bindings again before packaging; reject a changing renderer/source.
    bound_file(source / 'progress.json', manifest['source_progress_sha256'], 'Final source progress')
    bound_file(manifest_path, manifest_digest, 'Final renderer manifest')
    return manifest_path, manifest, checked


def package(demos, source, output, workers=2, validate_only=False):
    require(not output.exists() or validate_only, 'Refusing to replace existing ZIP: ' + str(output))
    manifest_path, manifest, entries = validate_collection(demos, source, workers)
    report = dict(schema_version=1, status='passed', checked_at=datetime.now(timezone.utc).isoformat(),
        scope=NOTICE + ' Only source hashes, trajectory completeness and file decoding are checked.',
        kind='physical_execution', planning_only=False, total=TOTAL,
        execution_counts=dict(Counter(row['execution_status'] for row in entries)),
        executed_count=sum(row['executed'] for row in entries),
        task_success_count=sum(row['task_success'] for row in entries),
        safety_independently_verified=False,
        source_run=str(source.relative_to(ROOT)), source_progress_sha256=manifest['source_progress_sha256'],
        manifest_sha256=sha(manifest_path), task_counts=dict(sorted(Counter(row['task_id'] for row in entries).items())),
        variant_counts=dict(sorted(Counter(row['variant_id'] for row in entries).items())), entries=entries)
    if validate_only:
        print(json.dumps(dict(status='passed', scenes=TOTAL, videos_fully_decoded=TOTAL,
                              output_files_written=False), ensure_ascii=False))
        return report
    generated = {'index.html': index_html(entries), 'README.md': readme(entries, source),
                 'validation.json': json.dumps(report, ensure_ascii=False, indent=2) + '\n'}
    files = {'manifest.json': manifest_path,
             **{row['video_path']: demos / row['video_path'] for row in entries},
             **{row['execution_result_package_path']: demo_path(row['execution_result_path'], demos) for row in entries}}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.trajectory-package-', dir=output.parent) as temporary:
        temporary = Path(temporary)
        for name, text in generated.items():
            path = temporary / name
            path.write_text(text)
            files[name] = path
        checksums = ''.join(sha(path) + '  ' + name + '\n' for name, path in sorted(files.items()))
        checksum_path = temporary / 'SHA256SUMS.txt'
        checksum_path.write_text(checksums)
        files['SHA256SUMS.txt'] = checksum_path
        zipped = temporary / 'demos.zip'
        expected_hashes = {name: sha(path) for name, path in files.items()}
        with zipfile.ZipFile(zipped, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
            for name, path in sorted(files.items()):
                archive.write(path, arcname=name)
        with zipfile.ZipFile(zipped) as archive:
            require(set(archive.namelist()) == set(files), 'ZIP entry set mismatch')
            require(archive.testzip() is None, 'ZIP CRC validation failed')
            for name, expected in expected_hashes.items():
                with archive.open(name) as stream:
                    digest = hashlib.file_digest(stream, 'sha256').hexdigest()
                require(digest == expected, 'ZIP entry SHA mismatch: ' + name)
        # Verify that source files were not replaced between validation and archive creation.
        require(sha(manifest_path) == report['manifest_sha256'], 'Renderer manifest changed while packaging')
        for row in entries:
            bound_file(demos / row['video_path'], row['video_sha256'], row['scene_id'] + ' packaged video')
            bound_file(demo_path(row['execution_result_path'], demos), row['execution_result_sha256'],
                       row['scene_id'] + ' packaged physical result')
        for name in (*generated, 'SHA256SUMS.txt'):
            shutil.copyfile(temporary / name, demos / name)
        zipped.replace(output)
    print(json.dumps(dict(status='packaged', scenes=TOTAL, zip_path=str(output),
                          zip_bytes=output.stat().st_size, zip_sha256=sha(output)), ensure_ascii=False, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demos', type=Path, default=DEFAULT_DEMOS)
    parser.add_argument('--source-run', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    require(1 <= args.workers <= 4, 'Use 1–4 CPU decode workers')
    demos, source = args.demos.resolve(), args.source_run.resolve()
    source.relative_to(ROOT)
    output = args.output.resolve() if args.output else demos.with_suffix('.zip')
    package(demos, source, output, args.workers, args.validate_only)


if __name__ == '__main__':
    main()
