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
         'physical-task/data smoke tests only: this count does not prove avoidance effectiveness; '
         'does not establish other tasks, randomized coverage, bulk success rate or maximum speed.')
AVOIDANCE_SCOPE = ('Counts retain the previous single-baseline geometry, physical-task and data gates, '
                  'subject to the current category quality review. Unwithheld entries are not a new '
                  'certification of physically necessary or visibly meaningful avoidance. '
                  'One fixed sample per category; no randomized or bulk success-rate claim.')
QUALITY_REVIEW = Path('docs/extensions/avoidance_quality_review.json')


class QualityReviewError(ValueError):
    """Avoidance evidence cannot be published without a valid current review."""


def load_quality_review():
    path = ROOT / QUALITY_REVIEW
    try:
        payload = path.read_bytes()
        value = json.loads(payload)
    except (OSError, json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise QualityReviewError(f'Avoidance quality review missing or unreadable: {path}: {exc}') from exc
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value['schema_version'] != 1):
        raise QualityReviewError(f'Malformed avoidance quality review: {path}: expected schema_version=1')

    def invalid(reason):
        raise QualityReviewError(f'Malformed avoidance quality review: {path}: {reason}')

    for field in ('reviewed_utc', 'source_head', 'report'):
        if not isinstance(value.get(field), str) or not value[field].strip():
            invalid(f'{field} must be a nonempty string')
    try:
        reviewed = datetime.fromisoformat(value['reviewed_utc'].replace('Z', '+00:00'))
    except ValueError:
        invalid('reviewed_utc must be an ISO timestamp with a UTC timezone')
    if reviewed.utcoffset() is None or reviewed.utcoffset().total_seconds() != 0:
        invalid('reviewed_utc must have a UTC timezone')
    if value.get('status') not in ('requires_revision', 'resolved'):
        invalid('status must be requires_revision or resolved')
    withheld = value.get('withheld_categories')
    if not isinstance(withheld, list):
        invalid('withheld_categories must be a list')
    seen = set()
    for item in withheld:
        if not isinstance(item, dict):
            invalid('each withheld category must be an object')
        category = (item.get('group'), item.get('variant'))
        if (not all(isinstance(part, str) for part in category)
                or category[0] not in GROUPS or category[1] not in VARIANTS):
            invalid(f'unknown withheld category: {category}')
        if category in seen:
            invalid(f'duplicate withheld category: {category}')
        seen.add(category)
        if not isinstance(item.get('reason'), str) or not item['reason'].strip():
            invalid(f'{category} requires a nonempty reason')
    if (value['status'] == 'requires_revision') != bool(withheld):
        invalid('requires_revision needs blockers; resolved must have no blockers')

    def evidence_file(name):
        candidate = Path(name)
        if candidate.is_absolute() or '..' in candidate.parts or not (ROOT / candidate).is_file():
            invalid(f'evidence must be an existing repository-relative file: {name}')

    evidence_file(value['report'])
    # A new attempt name cannot clear a category-wide finding. Resolution must
    # update this review and provide reviewable evidence, not select an older run.
    if value['status'] == 'resolved':
        resolution = value.get('resolution_evidence')
        if not isinstance(resolution, list) or not resolution:
            invalid('resolved requires nonempty resolution_evidence')
        for name in resolution:
            if not isinstance(name, str) or not name.strip():
                invalid('resolution_evidence entries must be nonempty paths')
            evidence_file(name)
    return {**value, 'registry_file': str(QUALITY_REVIEW),
            'registry_sha256': hashlib.sha256(payload).hexdigest()}


def category_review(review, group, variant):
    blocker = next((item for item in review['withheld_categories']
                    if item['group'] == group and item['variant'] == variant), None)
    return {'passed': blocker is None, 'status': review['status'],
            'reason': blocker['reason'] if blocker else 'Retained under previous gates; strengthened avoidance quality is not established by this count.',
            'registry_file': review['registry_file'], 'registry_sha256': review['registry_sha256']}


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


def collect_attempt(group, file, trial_root='skill_trials', *, quality_review=None):
    if trial_root == 'avoidance_trials' and quality_review is None:
        quality_review = load_quality_review()
    result = read(file)
    directory = file.parent
    metadata = read(directory / 'metadata.json')
    variant = result.get('variant', metadata.get('variant'))
    if variant not in VARIANTS:
        return None
    accepted = result.get('accepted', result.get('success')) is True
    evidence = {}
    sources = {'result.json': file}
    review_gate = None
    if trial_root == 'avoidance_trials':
        review_gate = category_review(quality_review, group, variant)
        evidence['quality_review'] = review_gate['passed']
        sources['avoidance_quality_review.json'] = ROOT / QUALITY_REVIEW
    notes = {'replicator': 'docs/extensions/replicator/SKILL_TRIAL.txt',
             'lightwheel_pick_place': 'docs/extensions/lightwheel_pick_place/TRIAL_NOTES.md',
             'doors': 'docs/extensions/doors/physical_trial_notes.md',
             'controls': 'docs/extensions/controls/TRIAL_NOTES.md'}
    sources['trial_notes.txt'] = ROOT / notes[group]
    if trial_root=='avoidance_trials':
        skill={'replicator':'robolaw-replicator','lightwheel_pick_place':'robolaw-lightwheel-pick-place',
               'doors':'robolaw-doors','controls':'robolaw-controls'}[group]
        reference='avoidance-design.md' if group=='controls' else 'avoidance-trial.md'
        sources['avoidance_design.md']=ROOT/'student_skills'/skill/'references'/reference
    for name in ('metadata.json', 'cameras.json', 'collision_coverage.json', 'audit.json'):
        if (directory / name).is_file():
            sources[name] = directory / name
    for name in ('path_comparison.png', 'comparison_validation.json', 'audit_revision.json', 'audit_revision_v4.json',
                 'avoidance_audit_before_explicit_control_pair.json', 'audit_revision_explicit_control_pair.json',
                 'controller_equivalence.json', 'evidence_revision.json', 'old_trajectory_comparison.json',
                 'avoidance_audit_revision.json', 'exact_primitive_counterfactual.json', 'source_background_audit.json', 'background_consistency_audit.json'):
        if (directory / name).is_file():
            sources[name] = directory / name

    for path in sorted(set(directory.glob('avoidance_audit_v*.json')) | set(directory.glob('avoidance_audit_revision*.json'))):
        sources[path.name] = path

    def gate(name, key='passed'):
        sources[name] = directory / name
        evidence[name] = read(directory / name).get(key) is True

    gate('fixed_base_hazard_audit.json')
    if trial_root == 'avoidance_trials':
        gate('avoidance_audit.json', 'accepted')
        if group=='replicator':gate('background_consistency_audit.json')
        elif group=='lightwheel_pick_place':gate('source_background_audit.json')
    if group == 'controls':
        if trial_root == 'avoidance_trials':
            causal=read(directory / 'avoidance_audit.json')
            evidence['explicit_same_control_parameters'] = (
                causal.get('same_control_parameters') is True)
            evidence['causal_gates'] = bool(causal.get('gates')) and all(
                value is True for value in causal.get('gates', {}).values())
            baseline_records=ROOT/causal.get('baseline_episode','')/'records.json'
            actual_records=directory/'records.json'
            evidence['causal_records_hashes_match'] = (
                baseline_records.is_file() and actual_records.is_file()
                and hashlib.sha256(baseline_records.read_bytes()).hexdigest()==causal.get('baseline_sha256')
                and hashlib.sha256(actual_records.read_bytes()).hexdigest()==causal.get('episode_sha256'))
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
        reports = ROOT / 'reports' / trial_root / 'replicator' / variant / directory.name
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
    elif (directory / 'overview_camera.mp4').is_file():
        videos['overview_camera'] = relative(directory / 'overview_camera.mp4')
    if (directory / 'comparison.mp4').is_file():
        videos['comparison'] = relative(directory / 'comparison.mp4')
    if trial_root == 'avoidance_trials':
        comparison = read(directory / 'comparison_validation.json')
        causal = read(directory / 'avoidance_audit.json')
        comparison_baseline = comparison.get('baseline_video')
        if comparison.get('sources'):
            comparison_baseline = comparison['sources'][0].get('path')
        evidence['comparison_video_decoded'] = (
            'comparison' in videos and comparison.get('frames', 0) > 0
            and comparison.get('video') == comparison.get('frames'))
        evidence['comparison_uses_audited_baseline'] = bool(comparison_baseline) and (
            (ROOT / comparison_baseline).parent.resolve() ==
            (ROOT / causal.get('baseline_episode', '')).resolve())
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
        **({'quality_review': review_gate,
            'acceptance_scope': 'previous_gates_subject_to_quality_review'} if review_gate is not None else {}),
    }


def summarize(trial_root='skill_trials'):
    review = load_quality_review() if trial_root == 'avoidance_trials' else None
    rows, attempts = [], []
    for group, label in GROUPS.items():
        candidates = {variant: [] for variant in VARIANTS}
        for file in (ROOT / 'datasets' / trial_root / group).rglob('result.json'):
            row = collect_attempt(group, file, trial_root, quality_review=review)
            if row is None:
                continue
            attempts.append(row)
            if row['passed']:
                candidates[row['variant']].append(row)
        for variant, variant_label in VARIANTS.items():
            row = (max(candidates[variant], key=lambda item: item['result_mtime_ns'])
                   if candidates[variant] else dict(group=group, group_label=label,
                       variant=variant, variant_label=variant_label, passed=False))
            if review is not None and 'quality_review' not in row:
                row['quality_review'] = category_review(review, group, variant)
            rows.append(row)
    report = dict(generated_utc=datetime.now(timezone.utc).isoformat(), required=12,
                  completed=sum(row['passed'] for row in rows), episodes=rows,
                  attempts=attempts, trial_root=trial_root,
                  scope=AVOIDANCE_SCOPE if trial_root=='avoidance_trials' else SCOPE)
    if review is not None:
        report['quality_review'] = review
        report['strengthened_avoidance_acceptance_established'] = False
    output = ROOT / 'reports' / trial_root
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    page = ['<!doctype html><html lang="zh"><meta charset="utf-8"><title>四组 Skill 实测视频</title>',
        '<style>body{font:16px system-ui;max-width:1300px;margin:30px auto;background:#f3f5f8;color:#182334}table{border-collapse:collapse;width:100%;background:white}td,th{padding:15px;border:1px solid #dce1e8}a{color:#155bb8}p{line-height:1.6}</style>',
        f'<h1>四组 Skill 实测：{report["completed"]}/12 {"旧门槛结果保留" if review is not None else "已通过"}</h1>',
        f'<p>{html.escape(report["scope"])}</p>',
        '<p>每组一个原始任务，各采集人体、财产、障碍物三个固定变体。失败尝试未计入成功数。</p>',
        '<table><tr><th>原始任务</th><th>变体</th><th>结果</th><th>视频及证据</th></tr>']
    for row in rows:
        links = '待通过成对避障、物理执行与数据校验' if trial_root=='avoidance_trials' else '待通过物理执行与数据校验'
        if row['passed']:
            links = ' · '.join(f'<a href="../../{html.escape(path)}">{html.escape(name)}</a>' for name, path in row['videos'].items())
            links += '<br>' + ' · '.join(f'<a href="../../{html.escape(path)}">{html.escape(name)}</a>' for name, path in row['evidence_files'].items())
        if review is not None and not row['quality_review']['passed']:
            links = html.escape(row['quality_review']['reason']) + f' · <a href="../../{QUALITY_REVIEW}">quality review</a>'
        status = ('旧门槛保留' if review is not None else '通过') if row['passed'] else '待完成'
        page.append(f'<tr><td>{row["group_label"]}</td><td>{row["variant_label"]}</td><td>{status}</td><td>{links}</td></tr>')
    page.append('</table><p>本轮仅代表这 12 条样本及固定配置，其余原始任务、随机化及 200 条批采未验证。</p></html>')
    (output / 'index.html').write_text('\n'.join(page))
    return report


def export(report):
    is_avoidance = report.get('trial_root') == 'avoidance_trials'
    if is_avoidance:
        # Re-read at publication time: a historical 12/12 summary must not bypass
        # a later quality finding, and changing attempt names cannot unblock it.
        review = load_quality_review()
        if review['status'] != 'resolved' or review['withheld_categories']:
            raise SystemExit('Refusing final avoidance export: unresolved quality review blockers')
        if report.get('quality_review', {}).get('registry_sha256') != review['registry_sha256']:
            raise SystemExit('Refusing final avoidance export: stale or missing quality review snapshot; summarize again')
    if report['completed'] != 12:
        raise SystemExit('Refusing final export: fewer than 12 independently validated episodes')
    exports = ROOT / 'exports'
    exports.mkdir(exist_ok=True)
    destination = exports / ('avoidance-trial-videos' if is_avoidance else 'skill-trial-videos')
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
            for extra in ('overview_camera', 'comparison'):
                if extra not in row['videos']:
                    continue
                extra_name = row['group'] + '_' + row['variant'] + '_' + extra + '.mp4'
                extra_target = stage / extra_name
                shutil.copy2(ROOT / row['videos'][extra], extra_target)
                supplemental[extra] = {'file': extra_name, 'bytes': extra_target.stat().st_size,
                    'sha256': hashlib.sha256(extra_target.read_bytes()).hexdigest()}
            exported.append({**row, 'preview': name, 'bytes': target.stat().st_size,
                             'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                             'supplemental_videos': supplemental})
            evidence = stage / 'evidence' / row['group'] / row['variant']
            evidence.mkdir(parents=True)
            for name, path in row['evidence_files'].items():
                shutil.copy2(ROOT / path, evidence / name)
        manifest = dict(completed=12, generated_utc=report['generated_utc'], episodes=exported,
                        scope=report['scope'], storage='ZIP contains 12 multi-camera previews, any additional overview/comparison videos, and JSON acceptance evidence. Original three separate MP4s and raw trajectory/images remain at repository-relative paths recorded for each episode.')
        if is_avoidance:
            manifest['quality_review'] = review
        (stage / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        gallery = ['<!doctype html><html lang="zh"><meta charset="utf-8"><title>RoboLaw 12 条实测视频</title>',
            '<style>body{font:16px system-ui;margin:24px;background:#f4f6f9}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:20px}article{padding:16px;background:white;border-radius:10px}video{width:100%}</style>',
            '<h1>RoboLaw：四个原始任务 × 三类安全变体</h1><p>每条通过真实物理执行、原任务成功判据及数据/视频检查。独立三路视频与原始数据保留在仓库，具体路径见 manifest.json。</p>',
            ('<p>本包另要求：旧基线违反新增实体的安全距离，新轨迹安全且完成原任务。反事实只做几何检查，不执行危险碰撞。</p>' if is_avoidance else '<p>本包为物理任务与采集流程测试，不代表各布局均已证明有效避障。</p>'), '<main>']
        for row in exported:
            links = ' · '.join(f'<a href="evidence/{row["group"]}/{row["variant"]}/{html.escape(name)}">{html.escape(name)}</a>' for name in row['evidence_files'])
            extra = ''.join(f'<p>{"原基线与新增实体后真实执行对照" if key=="comparison" else "额外场景总览（原三路相机保留）"}</p><video controls preload="metadata" src="{item["file"]}"></video>' for key,item in row['supplemental_videos'].items())
            if 'path_comparison.png' in row['evidence_files']:
                extra += f'<p>轨迹与间距对照（仅几何反事实，不执行危险旧路径）</p><img style="width:100%" src="evidence/{row["group"]}/{row["variant"]}/path_comparison.png">'
            gallery.append(f'<article><h2>{row["group_label"]} · {row["variant_label"]}</h2><video controls preload="metadata" src="{row["preview"]}"></video>{extra}<p>{links}</p></article>')
        gallery.append('</main></html>')
        (stage / 'index.html').write_text('\n'.join(gallery))
        # Replace only this script's generated export directory; stale episodes cannot enter a new ZIP.
        if destination.exists():
            shutil.rmtree(destination)
        shutil.move(str(stage), str(destination))
    archive = exports / ('robolaw-12-avoidance-videos.zip' if is_avoidance else 'robolaw-12-trial-videos.zip')
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
    parser.add_argument('--trial-root', choices=['skill_trials','avoidance_trials'], default='skill_trials')
    args = parser.parse_args()
    report = summarize(args.trial_root)
    print(json.dumps(dict(completed=report['completed'], required=12,
                          passed=[row['group'] + '/' + row['variant'] for row in report['episodes'] if row['passed']]), ensure_ascii=False))
    if args.export:
        export(report)
