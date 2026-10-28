#!/usr/bin/env python3
"""Package 143 first-pass + 11 fixed camera retakes without replacing history.

This is a separate publication format. The original once-per-scene packager is
unchanged. All 154 first-pass videos and all 11 retakes must decode and retain
their source/result SHA bindings. Retakes are selected as a whole camera-only
cohort, independently of success. No simulation, rendering or source writes.
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
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator import package_trajectory_demos as original

TOTAL = 154
RETAKE_TOTAL = 11
RETAKE_TASK = 'kitchen_bench_replicator_u_shape_banana_bowl'
RETAKE_SCENE_PREFIX = 'u_shape_banana_bowl__'
DEFAULT_FIRST = original.DEFAULT_DEMOS
DEFAULT_SOURCE = original.DEFAULT_SOURCE
DEFAULT_RETAKES = ROOT/'exports/replicator_u_banana_camera_retakes_20261028'
DEFAULT_PLAN = ROOT/'configs/extensions/replicator/physical_camera_retake_plan_20261028.json'
DEFAULT_BUNDLE = ROOT/'exports/replicator_154_physical_video_bundle_20261028'
LABELS = {'success': '成功', 'failed': '任务失败', 'error': '错误'}
NOTICE = ('154 个主演示视频包含 143 个首轮视频和 11 个相机重录视频；'
          '固定选择全部 11 次重录，无论成败，不按结果择优。另保留 11 个首轮遮挡视频。'
          '首轮成绩与选中演示成绩分别统计；安全及过程避障未独立验收。')

require = original.require
sha = original.sha
read = original.read
bound_file = original.bound_file


def local(value):
    path = (ROOT/str(value).removeprefix('/workspace/')).resolve()
    path.relative_to(ROOT)
    return path


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2)+'\n').encode()


def outcome(row):
    return dict(execution_status=row['execution_status'], task_success=row['task_success'],
                executed=row['executed'], attempts_within_output=row['attempts'])


def source_selection(row, demos, *, attempt_number, result_package_path, video_package_path):
    """Both original location and exact distributed byte identity stay explicit."""
    return dict(**outcome(row), physical_attempt_number=attempt_number,
        execution_result_path=str(original.demo_path(row['execution_result_path'], demos).relative_to(ROOT)),
        execution_result_sha256=row['execution_result_sha256'], result_package_path=result_package_path,
        video_path=str((demos/row['video_path']).relative_to(ROOT)), video_sha256=row['video_sha256'],
        video_package_path=video_package_path, video=row['video'])


def selection_document(first, retakes, first_root, retake_root, provenance):
    retake_map = {row['scene_id']:row for row in retakes}
    entries = []
    for row in first:
        sid = row['scene_id']
        selected = retake_map.get(sid, row)
        replacement = sid in retake_map
        entries.append(dict(scene_id=sid, task_id=row['task_id'], variant_id=row['variant_id'],
            variant_name=row['variant_name'], category=row['category'],
            selection_reason='fixed_camera_retake_cohort' if replacement else 'first_pass',
            historical_quality_issues=row['historical_quality_issues'],
            first_pass=source_selection(row, first_root, attempt_number=1,
                result_package_path=f'first_pass_results/{sid}.json',
                video_package_path=f'first_pass_videos/{sid}.mp4' if replacement else f'videos/{sid}.mp4'),
            selected_demo=source_selection(selected, retake_root if replacement else first_root,
                attempt_number=2 if replacement else 1,
                result_package_path=f'selected_results/{sid}.json', video_package_path=f'videos/{sid}.mp4')))
    return dict(schema_version=1, kind='physical_demo_selection', created_at=datetime.now(timezone.utc).isoformat(),
        scope=NOTICE, selection_policy='All 11 camera-retake scenes selected regardless of outcome; no best-of selection.',
        scenes=TOTAL, main_videos=TOTAL, retained_first_pass_videos=RETAKE_TOTAL,
        total_mp4_files=TOTAL+RETAKE_TOTAL, first_pass_episode_records=TOTAL,
        additional_camera_retake_episode_records=RETAKE_TOTAL,
        first_pass_counts=dict(sorted(Counter(row['execution_status'] for row in first).items())),
        selected_demo_counts=dict(sorted(Counter(row['selected_demo']['execution_status'] for row in entries).items())),
        retake_counts=dict(sorted(Counter(row['execution_status'] for row in retakes).items())),
        first_pass_executed=sum(row['executed'] for row in first),
        camera_retake_executed=sum(row['executed'] for row in retakes),
        safety_independently_verified=False, provenance=provenance, entries=entries)


def count_text(counts):
    return ' / '.join(f'{LABELS[key]} {counts.get(key, 0)}' for key in LABELS)


def index_html(selection):
    data = json.dumps(selection['entries'], ensure_ascii=False).replace('<', '\\u003c')
    rows = []
    for entry in selection['entries']:
        first, chosen = entry['first_pass'], entry['selected_demo']
        rows.append('<tr><td>'+html.escape(entry['scene_id'])+'</td><td>'+LABELS[first['execution_status']]
            +' · <a href="'+html.escape(first['result_package_path'], quote=True)+'">首轮结果</a></td><td>'
            +LABELS[chosen['execution_status']]+' · 第 '+str(chosen['physical_attempt_number'])+' 次'
            +' · <a href="'+html.escape(chosen['result_package_path'], quote=True)+'">所选结果</a></td><td>'
            +('；'.join(html.escape(x) for x in entry['historical_quality_issues']) or '—')+'</td></tr>')
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboLaw 154 景物理演示 · 保留首轮与相机重录</title><style>
body{font:16px/1.6 system-ui,sans-serif;background:#111a25;color:#e0e9f5;max-width:1250px;margin:24px auto;padding:0 20px}a{color:#8ecaff}h1{font-size:26px}.notice{padding:14px;background:#403520;border-left:4px solid #e6bc59}.scores{display:flex;gap:20px;flex-wrap:wrap}.score{padding:12px 18px;background:#203247;border-radius:6px}input,select,button{font:inherit;padding:8px;background:#213248;color:inherit;border:1px solid #62738a;border-radius:4px}.controls{display:flex;gap:10px;flex-wrap:wrap;margin:20px 0}video{width:100%;max-height:65vh;background:#000}table{width:100%;border-collapse:collapse;font-size:14px}th,td{text-align:left;border-bottom:1px solid #3c4b5e;padding:8px}#quality{color:#f0c772}.subtle{color:#a6b9cd}</style>
<h1>154 景真实物理演示：首轮与相机重录分开记录</h1>
<p class="notice">'''+NOTICE+''' 原 U 型香蕉任务的首轮相机受天花板遮挡；这 11 景额外真实执行一次用于录制。主播放器展示所选演示，不能据此替代首轮成绩。</p>
<div class="scores"><div class="score"><strong>首轮 154 景成绩</strong><br>'''+count_text(selection['first_pass_counts'])+'''</div><div class="score"><strong>选中 154 个演示成绩</strong><br>'''+count_text(selection['selected_demo_counts'])+'''</div><div class="score"><strong>新增 11 次相机重录</strong><br>'''+count_text(selection['retake_counts'])+'''</div></div>
<p><a href="demo_selection.json">逐景来源与选择清单</a> · <a href="validation.json">校验记录</a> · <a href="README.md">说明</a> · <a href="SHA256SUMS.txt">文件哈希</a></p>
<div class="controls"><input id="search" placeholder="搜索任务或变体" aria-label="搜索任务或变体"><select id="cohort"><option value="">所有场景</option><option value="retake">11 个相机重录场景</option><option value="original">143 个首轮场景</option></select><button id="prev">上一个</button><button id="next">下一个</button></div>
<select id="scenes" style="width:100%" aria-label="选择场景"></select><h2 id="title"></h2><p id="scores"></p><p id="quality"></p><video id="player" controls preload="metadata" playsinline></video><p id="links"></p><p id="position" class="subtle"></p>
<details><summary>154 景首轮与所选结果对照</summary><table><thead><tr><th>场景</th><th>首轮成绩</th><th>所选演示成绩</th><th>历史质量提示</th></tr></thead><tbody>'''+''.join(rows)+'''</tbody></table></details>
<script>const DATA='''+data+''';const $=id=>document.getElementById(id),labels={success:'成功',failed:'任务失败',error:'错误'};let list=DATA,at=0;
function link(href,text){const a=document.createElement('a');a.href=href;a.textContent=text;return a;}
function show(){const row=list[at];$('prev').disabled=!row||at===0;$('next').disabled=!row||at===list.length-1;$('scenes').value=String(at);$('links').replaceChildren();$('position').textContent=row?`${at+1} / ${list.length}`:'没有匹配项';$('title').textContent=row?.scene_id||'';$('quality').textContent=row?.historical_quality_issues.join('；')||'';if(!row){$('scores').textContent='';$('player').removeAttribute('src');$('player').load();return;}const a=row.first_pass,b=row.selected_demo;$('scores').textContent='首轮：'+labels[a.execution_status]+'；所选演示：'+labels[b.execution_status]+'（实际第 '+b.physical_attempt_number+' 次，'+(b.physical_attempt_number===2?'因相机重录，固定选择':'首轮')+'）';$('player').src=b.video_package_path;$('links').append(link(b.result_package_path,'所选实际结果'),document.createTextNode(' · '),link(a.result_package_path,'首轮实际结果'));if(b.physical_attempt_number===2)$('links').append(document.createTextNode(' · '),link(a.video_package_path,'查看保留的首轮遮挡视频'));}
function filter(){const q=$('search').value.toLowerCase(),c=$('cohort').value;list=DATA.filter(x=>(!q||(x.scene_id+' '+x.variant_name).toLowerCase().includes(q))&&(!c||(c==='retake')===(x.selected_demo.physical_attempt_number===2)));at=0;$('scenes').replaceChildren(...list.map((x,i)=>{const o=document.createElement('option');o.value=String(i);o.textContent=x.scene_id+(x.selected_demo.physical_attempt_number===2?' · 相机重录':'');return o}));show();}
$('search').oninput=filter;$('cohort').onchange=filter;$('scenes').onchange=()=>{at=Number($('scenes').value);show();};$('prev').onclick=()=>{at--;show();};$('next').onclick=()=>{at++;show();};filter();</script></html>'''


def readme(selection):
    warnings = '\n'.join('- `'+row['scene_id']+'`：'+'；'.join(row['historical_quality_issues'])
                         for row in selection['entries'] if row['historical_quality_issues'])
    return f'''# RoboLaw 154 景可查看物理演示

**{NOTICE}**

| 统计口径 | 成绩 |
| --- | --- |
| 首轮 154 景 | {count_text(selection['first_pass_counts'])} |
| 选中 154 个演示 | {count_text(selection['selected_demo_counts'])} |
| 新增 11 次相机重录 | {count_text(selection['retake_counts'])} |

U 型香蕉放碗任务的 11 个首轮视频受天花板遮挡，额外各做一次真实物理执行以录制新相机画面。全部 11 份新结果均被选入演示，无论新结果成功还是失败；没有根据成绩挑选较好的一次。首轮记录永久保留，不能把选中演示的成绩当作首轮成绩。共有 154 个首轮 episode 记录，加 11 个相机重录 episode 记录；初始化恢复启动另有历史记录，不混作物理 episode。

解压后打开 `index.html`。`videos/` 有 154 个主视频（143 首轮 + 11 重录）；`first_pass_videos/` 有 11 个原遮挡视频，合计 **165 个 MP4**。`first_pass_results/` 保留 154 份首轮 JSON，`selected_results/` 保存 154 份所选结果，两者分开。`demo_selection.json` 是分发文件的正式映射，逐景绑定原路径、包内路径、真实结果、实际尝试编号及结果/视频 SHA。

`provenance/` 保存原始首轮 manifest、重录 manifest、相机重录计划的原始字节。这些原 manifest 是溯源快照，其原 `video_path` 没有改写，不是本包主视频的索引；切勿用旧路径的 SHA 来解释已选择的新视频。初始化中断历史和证据额外收录，同一日志按 SHA 去重。它们不能替代任何正式结果或视频。

打包前，原有一次执行打包器完整校验首轮 154 景，再逐景校验 11 个新 episode；所有 165 个视频完整解码并校验帧数、尺寸、时长及 SHA。视频解码和来源校验不等于独立证明任务安全、场景质量或过程避障。新画面是否清晰仍应人工检查。

## 保留的历史场景质量提示

{warnings}

## 文件校验

在解压目录运行 `sha256sum -c SHA256SUMS.txt`；该清单覆盖除自身外的所有文件。`validation.json` 记录逐次实际解码与来源绑定。原始规划 NPZ 和大型仿真资产不在视频包中。
'''


def validate_retake(raw, plan_row, first, first_root, retake_root, source_row, source, plan_path, plan_digest, plan):
    """Reuse all once-run checks, then prove this is the fixed second episode."""
    checked = original.validate_entry(raw, source_row, source, retake_root)
    sid = raw['scene_id']
    actual_path = original.demo_path(raw['execution_result_path'], retake_root)
    actual = read(actual_path)
    old_path = original.demo_path(first['execution_result_path'], first_root)
    old = read(old_path)
    require(actual.get('executed') is True, sid+': camera retake never entered physical execution')
    require(type(raw.get('physical_attempt_number')) is int and raw['physical_attempt_number'] == 2
            and type(actual.get('physical_attempt_number')) is int and actual['physical_attempt_number'] == 2,
            sid+': retake must explicitly record physical attempt 2')
    require(old.get('physical_attempt_number', 1) == 1 and old.get('executed') is True,
            sid+': invalid first physical attempt')
    for record in (raw, actual):
        require(local(record['camera_retake_plan_path']) == plan_path
                and record.get('camera_retake_plan_sha256') == plan_digest,
                sid+': retake plan binding mismatch')
        require(record.get('first_pass') == plan_row['first_pass'], sid+': first-pass snapshot differs from plan')
    expected_first = dict(result_path=str(old_path.relative_to(ROOT)), result_sha256=first['execution_result_sha256'],
        video_path=str((first_root/first['video_path']).relative_to(ROOT)), video_sha256=first['video_sha256'],
        execution_status=first['execution_status'], task_success=first['task_success'], executed=first['executed'],
        physical_attempt_number=1)
    require(plan_row['first_pass'] == expected_first, sid+': retained first-pass result/video/outcome mismatch')
    for key in ('scene_id', 'task_id', 'variant_id', 'config_sha256', 'trajectory_sha256', 'physical_attempt_number'):
        require(raw.get(key) == plan_row.get(key), sid+': whitelist identity mismatch: '+key)
    for key in ('config_path', 'trajectory_path'):
        require(local(raw[key]) == local(plan_row[key]), sid+': whitelist source path mismatch: '+key)
    scene = source/'scenes'/sid/'source_scene.usda'
    require(local(plan_row['source_scene_path']) == scene, sid+': source scene path mismatch')
    bound_file(scene, plan_row['source_scene_sha256'], sid+' source scene')
    require(read(source/'scenes'/sid/'result.json')['composition']['scene_sha256'] == plan_row['source_scene_sha256'],
            sid+': source scene differs from selected plan')
    locked = plan_row.get('locked_execution_settings')
    require(isinstance(locked, dict) and bool(locked), sid+': missing locked physical execution settings')
    require(old.get('runtime_settings') == locked, sid+': first-pass settings differ from frozen retake plan')
    settings = actual.get('runtime_settings', {})
    require(set(settings) == set(locked) | {'camera_overrides_path', 'camera_overrides_sha256',
                                          'camera_retake_plan_path', 'camera_retake_plan_sha256'}
            and all(settings[key] == value for key, value in locked.items()),
            sid+': physical execution settings changed beyond the camera override')
    require(local(settings['camera_overrides_path']) == local(plan['camera_overrides_path'])
            and settings['camera_overrides_sha256'] == plan['camera_overrides_sha256']
            and local(settings['camera_retake_plan_path']) == plan_path
            and settings['camera_retake_plan_sha256'] == plan_digest,
            sid+': camera override differs from plan')
    expected_camera = read(local(plan['camera_overrides_path']))['tasks'][raw['task_id']]
    camera = actual.get('camera_override', {})
    require(local(camera['path']) == local(plan['camera_overrides_path'])
            and camera.get('sha256') == plan['camera_overrides_sha256']
            and camera.get('task_id') == raw['task_id'] and camera.get('applied') == expected_camera,
            sid+': actual camera override is not the frozen camera')
    rendering = actual.get('camera_renderer', {})
    require(rendering.get('requested') == expected_camera['renderer_settings']
            and set(rendering.get('applied', {})) == set(expected_camera['renderer_settings'])
            and all(math.isclose(rendering['applied'][key], value, rel_tol=1e-6, abs_tol=1e-12)
                    for key,value in expected_camera['renderer_settings'].items())
            and rendering.get('restored') is True, sid+': renderer exposure was not applied/restored')
    require(actual_path != old_path and retake_root/raw['video_path'] != first_root/first['video_path'],
            sid+': retake overwrote first-pass artifact location')
    checked.update(physical_attempt_number=2, first_pass=expected_first,
                   camera_retake_plan_path=str(plan_path.relative_to(ROOT)), camera_retake_plan_sha256=plan_digest)
    return checked


def validate_inputs(first_root, retake_root, source, plan_path, workers=2):
    # Do not relax or bypass the original 154-scene once-run validation.
    first_mp, first_manifest, first = original.validate_collection(first_root, source, workers)
    require(first_manifest.get('stage') == 'complete', 'First 154-scene physical manifest is not complete')
    require(len(first) == TOTAL and first_root != retake_root, 'First and retake output sets must remain separate')
    first_digest = sha(first_mp)
    require(read(first_mp) == first_manifest, 'First-pass manifest changed after validation')
    plan_digest = sha(plan_path)
    plan = read(plan_path)
    require(plan.get('schema_version') == 1 and plan.get('kind') == 'physical_camera_retake'
            and plan.get('physical_attempt_number') == 2
            and plan.get('select_all_retakes_regardless_of_success') is True,
            'Plan must authorize the complete camera-only retake cohort, without outcome selection')
    require(local(plan['first_pass_manifest_path']) == first_mp, 'Plan points to a different first-pass manifest')
    fixed = {row['scene_id'] for row in first if row['task_id'] == RETAKE_TASK
             and row['scene_id'].startswith(RETAKE_SCENE_PREFIX)}
    require(len(fixed) == RETAKE_TOTAL, 'Expected exactly the original 11 U-banana scenes')
    rows = plan['entries']
    require(len(rows) == RETAKE_TOTAL and len({row['scene_id'] for row in rows}) == RETAKE_TOTAL
            and {row['scene_id'] for row in rows} == fixed, 'Retake whitelist differs from the full fixed 11-scene cohort')
    plans = {row['scene_id']:row for row in rows}
    override_path = local(plan['camera_overrides_path'])
    bound_file(override_path, plan['camera_overrides_sha256'], 'Camera override')
    overrides = read(override_path)
    require(overrides.get('kind') == 'physical_execution_camera_overrides'
            and RETAKE_TASK in overrides.get('tasks', {}), 'Retake task has no camera override')
    retake_mp = retake_root/'manifest.json'
    retake_digest = sha(retake_mp)
    retake_manifest = read(retake_mp)
    require(retake_manifest.get('kind') == 'physical_execution' and retake_manifest.get('stage') == 'complete'
            and local(retake_manifest['source_run']) == source
            and retake_manifest.get('source_progress_sha256') == first_manifest['source_progress_sha256'],
            'Retakes must be a completed physical run bound to the same planning source')
    candidates = retake_manifest['entries']
    require(len(candidates) == RETAKE_TOTAL and len({row['scene_id'] for row in candidates}) == RETAKE_TOTAL
            and {row['scene_id'] for row in candidates} == fixed, 'Missing, duplicate or foreign retake scenes')
    require({p.name for p in (retake_root/'videos').glob('*.mp4')} == {sid+'.mp4' for sid in fixed},
            'Retake video directory must contain exactly the fixed 11 MP4 names')
    sources = {row['scene_id']:row for row in read(source/'progress.json')['entries']}
    first_map = {row['scene_id']:row for row in first}
    with ThreadPoolExecutor(max_workers=workers) as pool:
        retakes = list(pool.map(lambda raw: validate_retake(raw, plans[raw['scene_id']], first_map[raw['scene_id']],
            first_root, retake_root, sources[raw['scene_id']], source, plan_path, plan_digest, plan), candidates))
    retakes.sort(key=lambda row:row['scene_id'])
    provenance = dict(first_pass_manifest=dict(path=str(first_mp.relative_to(ROOT)), sha256=first_digest,
            package_path='provenance/first_pass/manifest.json'),
        retake_manifest=dict(path=str(retake_mp.relative_to(ROOT)), sha256=retake_digest,
            package_path='provenance/camera_retakes/manifest.json'),
        camera_retake_plan=dict(path=str(plan_path.relative_to(ROOT)), sha256=plan_digest,
            package_path='provenance/camera_retake_plan.json'),
        camera_overrides=dict(path=str(override_path.relative_to(ROOT)), sha256=plan['camera_overrides_sha256'],
            package_path='provenance/camera_overrides.json'),
        planning_source_progress=dict(path=str((source/'progress.json').relative_to(ROOT)),
            sha256=first_manifest['source_progress_sha256']))
    for ref in provenance.values():
        bound_file(local(ref['path']), ref['sha256'], 'Final input binding')
    return first, retakes, provenance


def package(first_root, retake_root, source, plan_path, bundle, output, workers=2, validate_only=False):
    for path in (first_root, retake_root, source, plan_path, bundle, output):
        path.resolve().relative_to(ROOT)
    require(not any(dest.is_relative_to(base) for dest in (bundle, output) for base in (first_root, retake_root, source)),
            'Bundle must not replace source execution directories')
    require(validate_only or (not bundle.exists() and not output.exists()), 'Refusing to overwrite an existing bundle/ZIP')
    first, retakes, provenance = validate_inputs(first_root, retake_root, source, plan_path, workers)
    selection = selection_document(first, retakes, first_root, retake_root, provenance)
    validation = dict(schema_version=1, status='passed', kind='reviewable_physical_demo_bundle',
        videos_fully_decoded=TOTAL+RETAKE_TOTAL, first_pass=first, camera_retakes=retakes,
        first_pass_validation='Original validate_collection, unchanged',
        retake_validation='Original validate_entry per independent episode plus frozen camera/first-pass bindings',
        selection_policy=selection['selection_policy'], first_pass_counts=selection['first_pass_counts'],
        selected_demo_counts=selection['selected_demo_counts'], retake_counts=selection['retake_counts'],
        safety_independently_verified=False, provenance=provenance)
    if validate_only:
        print(json.dumps(dict(status='passed', scenes=TOTAL, videos_fully_decoded=TOTAL+RETAKE_TOTAL,
            first_pass_counts=selection['first_pass_counts'], selected_demo_counts=selection['selected_demo_counts'],
            output_files_written=False), ensure_ascii=False))
        return validation
    files, expected = {}, {}
    def include(name, path, digest):
        require(not name.startswith('/') and '..' not in Path(name).parts, 'Unsafe archive path')
        bound_file(path, digest, 'Package input '+name)
        if name in files:
            require(expected[name] == digest, 'Conflicting duplicate archive entry')
        else:
            files[name], expected[name] = path, digest
    for ref in provenance.values():
        if ref.get('package_path'):
            include(ref['package_path'], local(ref['path']), ref['sha256'])
    for row in selection['entries']:
        for key in ('first_pass', 'selected_demo'):
            item = row[key]
            include(item['video_package_path'], local(item['video_path']), item['video_sha256'])
            include(item['result_package_path'], local(item['execution_result_path']), item['execution_result_sha256'])
    for row in [*first, *retakes]:
        for event in row['initialization_history']:
            for ref in event['evidence_files']:
                include(ref['package_path'], local(ref['path']), ref['sha256'])
    require(sum(name.startswith('videos/') and name.endswith('.mp4') for name in files) == TOTAL
            and sum(name.endswith('.mp4') for name in files) == TOTAL+RETAKE_TOTAL,
            'Bundle must have 154 selected MP4 and all 11 retained first-pass MP4')
    generated = {'demo_selection.json':json_bytes(selection), 'validation.json':json_bytes(validation),
                 'index.html':index_html(selection).encode(), 'README.md':readme(selection).encode()}
    output.parent.mkdir(parents=True, exist_ok=True)
    bundle.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.reviewable-demos-', dir=bundle.parent) as temporary:
        temporary = Path(temporary)
        staged = temporary/'bundle'; staged.mkdir()
        for name, path in files.items():
            dest = staged/name; dest.parent.mkdir(parents=True, exist_ok=True)
            # Copy snapshots so the standalone gallery never aliases mutable source files.
            shutil.copyfile(path, dest)
            require(sha(dest) == expected[name], 'Input changed during snapshot copy: '+name)
        for name, data in generated.items():
            (staged/name).write_bytes(data); expected[name] = hashlib.sha256(data).hexdigest()
        checksums = ''.join(digest+'  '+name+'\n' for name,digest in sorted(expected.items())).encode()
        (staged/'SHA256SUMS.txt').write_bytes(checksums)
        expected['SHA256SUMS.txt'] = hashlib.sha256(checksums).hexdigest()
        zipped = temporary/'bundle.zip'
        with zipfile.ZipFile(zipped, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
            for name in sorted(expected):
                archive.write(staged/name, arcname=name)
        with zipfile.ZipFile(zipped) as archive:
            require(len(archive.namelist()) == len(set(archive.namelist())) == len(expected)
                    and set(archive.namelist()) == set(expected), 'ZIP member set mismatch')
            require(archive.testzip() is None, 'ZIP CRC failed')
            for name,digest in expected.items():
                with archive.open(name) as stream:
                    require(hashlib.file_digest(stream, 'sha256').hexdigest() == digest, 'ZIP SHA mismatch: '+name)
        for name,path in files.items():
            bound_file(path, expected[name], 'Source changed during packaging: '+name)
        for ref in provenance.values():
            bound_file(local(ref['path']), ref['sha256'], 'Final source provenance')
        require(not bundle.exists() and not output.exists(), 'Output appeared during packaging; not overwritten')
        staged.replace(bundle)
        zipped.replace(output)
    print(json.dumps(dict(status='packaged', main_videos=TOTAL, total_mp4_files=TOTAL+RETAKE_TOTAL,
        bundle_path=str(bundle), zip_path=str(output), zip_bytes=output.stat().st_size, zip_sha256=sha(output),
        first_pass_counts=selection['first_pass_counts'], selected_demo_counts=selection['selected_demo_counts']),
        ensure_ascii=False, indent=2))
    return validation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first-pass', type=Path, default=DEFAULT_FIRST)
    parser.add_argument('--retakes', type=Path, default=DEFAULT_RETAKES)
    parser.add_argument('--source-run', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--retake-plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--bundle', type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    require(1 <= args.workers <= 4, 'Use 1–4 CPU decoding workers')
    bundle = args.bundle.resolve()
    package(args.first_pass.resolve(), args.retakes.resolve(), args.source_run.resolve(), args.retake_plan.resolve(),
            bundle, args.output.resolve() if args.output else bundle.with_suffix('.zip'), args.workers, args.validate_only)


if __name__ == '__main__':
    main()
