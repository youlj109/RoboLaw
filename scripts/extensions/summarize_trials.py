"""Index and export only independently validated, physically successful trials."""
import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import shutil
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
GROUPS = {
    'replicator': 'Replicator 芥末瓶入碗',
    'lightwheel_pick_place': 'Lightwheel 青柠入木碗',
    'doors': 'Lightwheel 打开冰箱',
    'controls': 'Lightwheel 左烤面包机按钮',
}
VARIANTS = {'human': '人体', 'property': '财产', 'robot': '障碍物'}
CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera', 'all_cameras')
SCOPE = ('One original task per skill, three fixed augmentation categories each; '
         'does not establish other tasks, randomized coverage, bulk success rate or maximum speed.')


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def passed_report(path):
    value = read(path)
    if isinstance(value, list):
        return bool(value) and all(item.get('passed') is True for item in value)
    return value.get('passed') is True


def relative(path):
    return str(path.relative_to(ROOT))


def collect_attempt(group, file):
    result = read(file)
    directory = file.parent
    metadata = read(directory / 'metadata.json')
    variant = result.get('variant', metadata.get('variant'))
    if variant not in VARIANTS:
        return None
    accepted = result.get('accepted', result.get('success')) is True
    evidence = {}
    sources = {'result.json': file}
    notes = {'replicator': 'docs/extensions/replicator/SKILL_TRIAL.txt',
             'lightwheel_pick_place': 'docs/extensions/lightwheel_pick_place/TRIAL_NOTES.md',
             'doors': 'docs/extensions/doors/physical_trial_notes.md',
             'controls': 'docs/extensions/controls/TRIAL_NOTES.md'}
    sources['trial_notes.txt'] = ROOT / notes[group]
    for name in ('metadata.json', 'cameras.json', 'collision_coverage.json', 'audit.json'):
        if (directory / name).is_file():
            sources[name] = directory / name

    def gate(name, key='passed'):
        sources[name] = directory / name
        evidence[name] = read(directory / name).get(key) is True

    gate('fixed_base_hazard_audit.json')
    if group == 'controls':
        gate('independent_validation.json')
        if (directory / 'self_collision_detailed_audit.json').exists():
            gate('self_collision_detailed_audit.json')
            if (directory / 'self_collision_audit.json').exists():
                sources['self_collision_audit.json'] = directory / 'self_collision_audit.json'
        else:
            gate('self_collision_audit.json')
        name = 'strict_clearance_audit.json'
        sources[name] = directory / name
        evidence[name] = read(directory / name).get('executed_augmented_trajectory', {}).get('passed') is True
    elif group == 'doors':
        gate('validation_full.json', 'accepted')
        gate('self_collision_audit.json')
        gate('hazard_clearance.json')
    elif group == 'lightwheel_pick_place':
        gate('validation.json')
        gate('self_collision_audit.json')
        if (directory / 'source_support_audit.json').exists():
            gate('source_support_audit.json')
        for name in ('full_world_audit.json', 'contacts.json', 'clearances.json'):
            sources[name] = directory / name
            evidence[name + '_exists'] = (directory / name).is_file()
    else:
        gate('source_relation_audit.json')
        gate('self_collision_audit.json')
        if variant == 'human':
            gate('overview_validation.json')
        reports = ROOT / 'reports/skill_trials/replicator' / variant / directory.name
        # Corrected reports preserve any historical runner summary that relied on exit codes alone.
        summary = reports / 'summary_corrected.json'
        if not summary.exists():
            summary = reports / 'summary.json'
        sources['runner_summary.json'] = summary
        runner = read(summary)
        evidence['runner_summary'] = runner.get('passed') is True
        codes = runner.get('exit_codes', {})
        evidence['all_runner_stages'] = all(codes.get(stage) == 0 for stage in ('collection', 'export', 'validation', 'audit'))
        for name in ('validation.json', 'audit.json', 'videos.json'):
            sources['runner_' + name] = reports / name
            evidence['runner_' + name] = passed_report(reports / name)
    videos = {name: relative(directory / (name + '.mp4')) for name in CAMERAS}
    evidence['all_four_videos_exist'] = all((ROOT / path).is_file() and (ROOT / path).stat().st_size > 0 for path in videos.values())
    if group in ('lightwheel_pick_place', 'replicator') and variant == 'human':
        videos['overview_camera'] = relative(directory / 'overview_camera.mp4')
        evidence['human_overview_video_exists'] = (directory / 'overview_camera.mp4').is_file() and (directory / 'overview_camera.mp4').stat().st_size > 0
        for name in ('cameras.json', 'frame_alignment.json'):
            sources['overview_' + name] = directory / 'overview' / name
            evidence['overview_' + name + '_exists'] = (directory / 'overview' / name).is_file()
    data_names = ('trajectory.npz', 'records.json', 'images.h5') if group == 'controls' else ('trajectory.h5', 'images.h5')
    data = {name: relative(directory / name) for name in data_names}
    if group in ('lightwheel_pick_place', 'replicator') and variant == 'human':
        data['overview/images.h5'] = relative(directory / 'overview/images.h5')
    evidence['raw_data_exists'] = all((ROOT / path).is_file() and (ROOT / path).stat().st_size > 0 for path in data.values())
    return {
        'group': group, 'group_label': GROUPS[group], 'variant': variant,
        'variant_label': VARIANTS[variant], 'episode': relative(directory),
        'collector_accepted': accepted, 'evidence': evidence,
        'evidence_files': {name: relative(path) for name, path in sources.items()},
        'passed': accepted and all(evidence.values()), 'videos': videos,
        'raw_data': data, 'result_mtime_ns': file.stat().st_mtime_ns,
        'error': result.get('failure', result.get('error')),
    }


def summarize():
    rows, attempts = [], []
    for group, label in GROUPS.items():
        candidates = {variant: [] for variant in VARIANTS}
        for file in (ROOT / 'datasets/skill_trials' / group).rglob('result.json'):
            row = collect_attempt(group, file)
            if row is None:
                continue
            attempts.append(row)
            if row['passed']:
                candidates[row['variant']].append(row)
        for variant, variant_label in VARIANTS.items():
            rows.append(max(candidates[variant], key=lambda item: item['result_mtime_ns'])
                        if candidates[variant] else dict(group=group, group_label=label,
                            variant=variant, variant_label=variant_label, passed=False))
    report = dict(generated_utc=datetime.now(timezone.utc).isoformat(), required=12,
                  completed=sum(row['passed'] for row in rows), episodes=rows,
                  attempts=attempts, scope=SCOPE)
    output = ROOT / 'reports/skill_trials'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    page = ['<!doctype html><html lang="zh"><meta charset="utf-8"><title>四组 Skill 实测视频</title>',
        '<style>body{font:16px system-ui;max-width:1300px;margin:30px auto;background:#f3f5f8;color:#182334}table{border-collapse:collapse;width:100%;background:white}td,th{padding:15px;border:1px solid #dce1e8}a{color:#155bb8}p{line-height:1.6}</style>',
        f'<h1>四组 Skill 实测：{report["completed"]}/12 已通过</h1>',
        '<p>每组一个原始任务，各采集人体、财产、障碍物三个固定变体。失败尝试未计入成功数。</p>',
        '<table><tr><th>原始任务</th><th>变体</th><th>结果</th><th>视频及证据</th></tr>']
    for row in rows:
        links = '待通过物理执行与数据校验'
        if row['passed']:
            links = ' · '.join(f'<a href="../../{html.escape(path)}">{html.escape(name)}</a>' for name, path in row['videos'].items())
            links += '<br>' + ' · '.join(f'<a href="../../{html.escape(path)}">{html.escape(name)}</a>' for name, path in row['evidence_files'].items())
        page.append(f'<tr><td>{row["group_label"]}</td><td>{row["variant_label"]}</td><td>{"通过" if row["passed"] else "待完成"}</td><td>{links}</td></tr>')
    page.append('</table><p>本轮仅代表这 12 条样本及固定配置，其余原始任务、随机化及 200 条批采未验证。</p></html>')
    (output / 'index.html').write_text('\n'.join(page))
    return report


def export(report):
    if report['completed'] != 12:
        raise SystemExit('Refusing final export: fewer than 12 independently validated episodes')
    exports = ROOT / 'exports'
    exports.mkdir(exist_ok=True)
    destination = exports / 'skill-trial-videos'
    with tempfile.TemporaryDirectory(prefix='.skill-trial-videos-', dir=exports) as temp:
        stage = Path(temp) / destination.name
        stage.mkdir()
        exported = []
        for row in report['episodes']:
            source = ROOT / row['videos']['all_cameras']
            name = row['group'] + '_' + row['variant'] + '.mp4'
            target = stage / name
            shutil.copy2(source, target)
            supplemental = {}
            if 'overview_camera' in row['videos']:
                extra_name = row['group'] + '_' + row['variant'] + '_overview.mp4'
                extra_target = stage / extra_name
                shutil.copy2(ROOT / row['videos']['overview_camera'], extra_target)
                supplemental['overview_camera'] = {'file': extra_name, 'bytes': extra_target.stat().st_size,
                    'sha256': hashlib.sha256(extra_target.read_bytes()).hexdigest()}
            exported.append({**row, 'preview': name, 'bytes': target.stat().st_size,
                             'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                             'supplemental_videos': supplemental})
            evidence = stage / 'evidence' / row['group'] / row['variant']
            evidence.mkdir(parents=True)
            for name, path in row['evidence_files'].items():
                shutil.copy2(ROOT / path, evidence / name)
        manifest = dict(completed=12, generated_utc=report['generated_utc'], episodes=exported,
                        scope=SCOPE, storage='ZIP contains 12 multi-camera previews, any additional overview videos, and JSON acceptance evidence. Original three separate MP4s and raw trajectory/images remain at repository-relative paths recorded for each episode.')
        (stage / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        gallery = ['<!doctype html><html lang="zh"><meta charset="utf-8"><title>RoboLaw 12 条实测视频</title>',
            '<style>body{font:16px system-ui;margin:24px;background:#f4f6f9}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:20px}article{padding:16px;background:white;border-radius:10px}video{width:100%}</style>',
            '<h1>RoboLaw：四个原始任务 × 三类安全变体</h1><p>每条通过真实物理执行、原任务成功判据及数据/视频检查。此包含 12 条多相机预览、两条额外场景总览和 JSON 验收证据；独立三路视频与原始数据保留在仓库，具体路径见 manifest.json。</p><main>']
        for row in exported:
            links = ' · '.join(f'<a href="evidence/{row["group"]}/{row["variant"]}/{html.escape(name)}">{html.escape(name)}</a>' for name in row['evidence_files'])
            extra = ''.join(f'<p>额外场景总览（原三路相机保留）</p><video controls preload="metadata" src="{item["file"]}"></video>' for item in row['supplemental_videos'].values())
            gallery.append(f'<article><h2>{row["group_label"]} · {row["variant_label"]}</h2><video controls preload="metadata" src="{row["preview"]}"></video>{extra}<p>{links}</p></article>')
        gallery.append('</main></html>')
        (stage / 'index.html').write_text('\n'.join(gallery))
        # Replace only this script's generated export directory; stale episodes cannot enter a new ZIP.
        if destination.exists():
            shutil.rmtree(destination)
        shutil.move(str(stage), str(destination))
    archive = exports / 'robolaw-12-trial-videos.zip'
    temporary = archive.with_suffix('.zip.tmp')
    with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as bundle:
        for file in sorted(destination.rglob('*')):
            if file.is_file():
                bundle.write(file, file.relative_to(exports))
    temporary.replace(archive)
    print('EXPORTED', archive, archive.stat().st_size)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export', action='store_true')
    args = parser.parse_args()
    report = summarize()
    print(json.dumps(dict(completed=report['completed'], required=12,
                          passed=[row['group'] + '/' + row['variant'] for row in report['episodes'] if row['passed']]), ensure_ascii=False))
    if args.export:
        export(report)
