"""Read-only summary of the bounded physical grasp search; no simulator is started.

    bash scripts/arena_python.sh scripts/summarize_grasp_search.py

This reports existing result/verification records and reviews accepted videos'
terminal state data. It neither reruns acceptance checks nor changes source
manifests, episode records, or the earlier 11-of-16 library result.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def relative(path):
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def resolve(path):
    path = Path(path)
    if path.is_absolute():
        if path.is_relative_to('/workspace'):
            return ROOT / path.relative_to('/workspace')
        return path
    return ROOT / path


def load(path, errors):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as error:
        errors.append({'path': relative(path), 'error': f'{type(error).__name__}: {error}'})
        return None


def reason_category(error):
    text = error or ''
    if 'orientation constraint' in text:
        return 'bottle_orientation_limit'
    if 'IK_FAIL' in text:
        return 'curobo_ik_fail'
    if '/person/' in text:
        return 'person_gripper_clearance' if 'gripper' in text else 'person_arm_or_hull_clearance'
    if '/kitchen/' in text:
        return 'kitchen_collision_or_clearance'
    if 'local Cartesian' in text:
        return 'local_cartesian_collision_or_joint_constraint'
    if 'tracking error' in text:
        return 'execution_tracking_error'
    if 'slipped' in text:
        return 'carried_bottle_slip'
    return 'other'


def check_status(verification, check, passed_field):
    details = verification.get('checks', {}).get(check, {})
    if details.get('skipped_reason'):
        return 'skipped'
    if verification.get(passed_field) is True:
        return 'passed'
    if details.get('exit_code') is not None:
        return 'failed'
    return 'not_recorded'


def summarize_run(directory, role, embedded, errors):
    result = load(directory / 'result.json', errors)
    saved = load(directory / 'verification.json', errors)
    verification = saved or embedded or {}
    execution = load(directory / 'execution.json', errors) or verification.get('execution', {})
    if result is None and not verification and not directory.exists():
        return None
    failures = (result or {}).get('plan_candidate_failures', [])
    reasons = [dict(phase=item.get('phase'), tilt_deg=item.get('tilt_deg'),
                    yaw_deg=item.get('yaw_deg'), failure_stage=item.get('failure_stage'),
                    reason_category=reason_category(item.get('error')), error=item.get('error'))
               for item in failures]
    physical = (result or {}).get('success')
    if result is None:
        status = 'result_not_written'
    elif physical is False:
        status = 'physical_failed'
    elif verification.get('passed') is True:
        status = 'verified_passed'
    else:
        status = 'physical_success_verification_incomplete_or_failed'
    videos = {name: relative(directory / name) for name in
              ('all_cameras.mp4', 'external_camera.mp4', 'external_camera_2.mp4', 'wrist_camera.mp4')
              if (directory / name).is_file()}
    return {
        'role': role, 'directory': relative(directory), 'status': status,
        'result_present': result is not None, 'verification_present': saved is not None,
        'verification_source': ('episode verification.json' if saved is not None else
                                'manifest embedded record' if embedded else 'not recorded'),
        'result_physical_success': physical,
        'verified_execution_success': verification.get('execution_success'),
        'data_status': check_status(verification, 'validation', 'data_success'),
        'audit_status': check_status(verification, 'audit', 'audit_pass'),
        'verification_passed': verification.get('passed'),
        'process_exit_code': execution.get('exit_code'),
        'seed': (result or {}).get('seed', verification.get('seed')),
        'frames': (result or {}).get('steps', verification.get('frames')),
        'failure_phase': (result or {}).get('failure_phase', verification.get('failure_phase')),
        'error': (result or {}).get('error', verification.get('error')),
        'requested_grasp_recorded_by_collector': (result or {}).get('requested_grasp'),
        'physical_grasp_observation': (result or {}).get('grasp_observation'),
        'selected_plan_orientations': [{key: plan[key] for key in
                                        ('phase', 'method', 'tilt_deg', 'yaw_deg', 'samples') if key in plan}
                                       for plan in (result or {}).get('plans', [])],
        'physics_clearance_checks': (result or {}).get('physics_clearance_checks'),
        'recorded_lift_height_m': (result or {}).get('lift_height'),
        'recorded_minimum_hazard_clearance_m': (result or {}).get('minimum_hazard_clearance_m'),
        'audited_minimum_clearance_m': verification.get('audited_minimum_clearance_m'),
        'rejected_orientation_candidate_count': len(reasons),
        'rejection_counts_by_stage': dict(Counter(item['failure_stage'] or 'unknown' for item in reasons)),
        'rejection_counts_by_phase': dict(Counter(item['phase'] or 'unknown' for item in reasons)),
        'rejection_counts_by_reason': dict(Counter(item['reason_category'] for item in reasons)),
        'rejected_orientation_candidates': reasons, 'video_files': videos,
        'successful_video_entry': videos.get('all_cameras.mp4')
            if role == 'video' and verification.get('passed') is True else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='datasets/kitchen_auto/human_grasp_search')
    parser.add_argument('--output', default='reports/human_grasp_search')
    parser.add_argument('--previous-library', default='datasets/kitchen_auto/library_full_validation')
    args = parser.parse_args()
    source, output, previous = resolve(args.input), resolve(args.output), resolve(args.previous_library)
    errors = []
    manifest = load(source / 'manifest.json', errors)
    if manifest is None:
        raise RuntimeError(f'Cannot read search manifest: {source / "manifest.json"}')
    previous_manifest = load(previous / 'manifest.json', errors) or {}
    asset_entries = {item['id']: item for item in manifest.get('assets', [])}
    # Include episodes already on disk even if a still-running manifest writer
    # has not appended their latest completion record yet.
    for path in source.glob('*/*/probe'):
        asset_entries.setdefault(path.parent.parent.name, {'id': path.parent.parent.name, 'trials': []})
    for path in source.glob('*/*/video'):
        asset_entries.setdefault(path.parent.parent.name, {'id': path.parent.parent.name, 'trials': []})
    assets, all_runs, terminal_episodes, terminal_errors = [], [], [], []
    terminal_module = None
    for identifier, entry in asset_entries.items():
        trials = {item['candidate']: item for item in entry.get('trials', [])}
        for path in (source / identifier).glob('*'):
            if path.is_dir() and any((path / role).exists() for role in ('probe', 'video')):
                trials.setdefault(path.name, {'candidate': path.name})
        candidates = []
        for candidate, trial in trials.items():
            row = {'candidate': candidate, 'manifest_requested_grasp': {
                key: trial.get(key) for key in ('tilt_deg', 'yaw_deg', 'offset_z_m')}, 'runs': {}}
            for role in ('probe', 'video'):
                embedded = trial.get(role) or {}
                directory = (resolve(embedded['directory']) if embedded.get('directory') else
                             source / identifier / candidate / role)
                run = summarize_run(directory, role, embedded, errors)
                if run is not None:
                    row['runs'][role] = run
                    all_runs.append(run)
            candidates.append(row)
        old_attempts = []
        for result_path in sorted((previous / identifier).glob('attempt_*/result.json')):
            old = load(result_path, errors) or {}
            verified = load(result_path.parent / 'verification.json', errors) or {}
            old_attempts.append({'directory': relative(result_path.parent), 'seed': old.get('seed'),
                                 'physical_success': old.get('success'), 'verification_passed': verified.get('passed'),
                                 'failure_phase': old.get('failure_phase'), 'error': old.get('error'),
                                 'frames': old.get('steps')})
        accepted = entry.get('accepted')
        accepted_video = None
        if accepted:
            accepted_directory = resolve(accepted['directory'])
            accepted_video = relative(accepted_directory / 'all_cameras.mp4')
            if terminal_module is None:
                spec = importlib.util.spec_from_file_location(
                    'grasp_terminal_review', ROOT / 'reports/library_terminal_state_review.py')
                terminal_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(terminal_module)
            try:
                terminal_episodes.append(terminal_module.review_episode(accepted_directory))
            except (OSError, ValueError, KeyError, IndexError) as error:
                terminal_errors.append({'obstacle_id': identifier, 'directory': relative(accepted_directory),
                                        'error': f'{type(error).__name__}: {error}'})
        assets.append({'obstacle_id': identifier, 'manifest_accepted': accepted,
                       'accepted_video_entry': accepted_video,
                       'old_library_attempts': old_attempts, 'candidates': candidates})
    completed = [run for run in all_runs if run['result_present']]
    stage_counts, reason_counts, phase_counts = Counter(), Counter(), Counter()
    for run in completed:
        stage_counts.update(run['rejection_counts_by_stage'])
        reason_counts.update(run['rejection_counts_by_reason'])
        if run['result_physical_success'] is False:
            phase_counts[run['failure_phase'] or 'unknown'] += 1
    previous_assets = previous_manifest.get('assets', [])
    timestamp = datetime.now(timezone.utc).isoformat()
    report = {
        'created_at_utc': timestamp, 'source_manifest': relative(source / 'manifest.json'),
        'source_manifest_complete_at_read': manifest.get('complete'),
        'scope': 'Fixed-layout, fixed-seed finite search over physical grasps. Counts describe these '
                 'attempts only, not a success-rate estimate or proof that failed scenes are infeasible. '
                 'A probe is distinct from a verified video replay. Existing verification decisions are reported unchanged.',
        'counts': {'assets_in_snapshot': len(assets), 'grasp_candidates_seen': sum(len(a['candidates']) for a in assets),
                   'runs_with_result': len(completed), 'runs_without_result': len(all_runs) - len(completed),
                   'probe_runs_with_result': sum(run['role'] == 'probe' for run in completed),
                   'video_runs_with_result': sum(run['role'] == 'video' for run in completed),
                   'physical_successful_runs': sum(run['result_physical_success'] is True for run in completed),
                   'physical_failed_runs': sum(run['result_physical_success'] is False for run in completed),
                   'verified_successful_probes': sum(run['role'] == 'probe' and run['verification_passed'] is True for run in completed),
                   'verified_successful_videos': sum(run['role'] == 'video' and run['verification_passed'] is True for run in completed),
                   'assets_accepted_in_manifest': sum(a['manifest_accepted'] is not None for a in assets)},
        'physical_failure_counts_by_phase': dict(phase_counts),
        'rejected_orientation_candidate_counts_by_stage': dict(stage_counts),
        'rejected_orientation_candidate_counts_by_reason': dict(reason_counts),
        'counting_note': 'Each physical run may reject several orientation candidates. Rejection counts '
                         'must not be treated as numbers of independent physical episodes.',
        'previous_library': {'manifest': relative(previous / 'manifest.json'),
                             'asset_count': len(previous_assets),
                             'accepted_asset_count': sum(item.get('passed') is True for item in previous_assets),
                             'note': 'Previous results remain separate and are not overwritten or relabelled by this search.'},
        'assets': assets, 'read_errors': errors,
        'terminal_review_file': relative(output / 'terminal_state_review.json'),
    }
    terminal = {'created_at_utc': timestamp, 'scope': 'Descriptive terminal-state review of videos '
                'already accepted by the search manifest; no new acceptance rule.',
                'episodes': terminal_episodes, 'read_errors': terminal_errors}
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    (output / 'terminal_state_review.json').write_text(json.dumps(terminal, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    c = report['counts']
    lines = ['人体场景抓取姿态有限搜索：实际记录汇总', '',
             f'最近一次所选搜索完成：{manifest.get("complete")}；数据目录：{relative(source)}',
             '固定布局、固定 seed 的有限候选搜索；不是成功率估计，也不是场景不可行证明。',
             'probe 与视频重放分别统计；视频原有物理、数据和几何验证结论保持不变。', '',
             f'已落盘运行 {c["runs_with_result"]}：probe {c["probe_runs_with_result"]}，视频 {c["video_runs_with_result"]}；'
             f'物理成功 {c["physical_successful_runs"]}，物理失败 {c["physical_failed_runs"]}。',
             f'已验证 probe {c["verified_successful_probes"]}；已验证视频 {c["verified_successful_videos"]}；'
             f'manifest 接受资产 {c["assets_accepted_in_manifest"]}；运行中/未落盘 {c["runs_without_result"]}。',
             f'旧资产库结果独立保留：{report["previous_library"]["accepted_asset_count"]}/'
             f'{report["previous_library"]["asset_count"]} 个资产通过。',
             f'物理失败阶段计数：{dict(phase_counts)}',
             f'规划候选拒绝阶段计数：{dict(stage_counts)}',
             f'规划候选拒绝原因计数：{dict(reason_counts)}',
             '同一物理运行可拒绝多个姿态目标；拒绝次数不能当作独立轨迹数。', '']
    for asset in assets:
        lines.append(asset['obstacle_id'])
        for old in asset['old_library_attempts']:
            lines.append(f'  旧对照：通过={old["verification_passed"]}；阶段={old["failure_phase"]}；{old["directory"]}')
        for trial in asset['candidates']:
            lines.append(f'  候选 {trial["candidate"]}')
            for role, run in trial['runs'].items():
                lines.append(f'    {role}：物理={run["result_physical_success"]}；数据={run["data_status"]}；'
                             f'audit={run["audit_status"]}；验证={run["verification_passed"]}；'
                             f'帧数={run["frames"]}；阶段={run["failure_phase"]}')
                requested = run['requested_grasp_recorded_by_collector'] or {}
                lines.append(f'      实际请求：tilt={requested.get("tilt_deg")}°；yaw={requested.get("yaw_deg")}°；'
                             f'高度偏移={requested.get("offset_z_m")} m；显式姿态={requested.get("explicit_orientation")}')
                if run['rejection_counts_by_stage']:
                    lines.append(f'      候选拒绝：{run["rejection_counts_by_stage"]}；原因：{run["rejection_counts_by_reason"]}')
                if run['error']:
                    lines.append(f'      最终错误：{run["error"]}')
                if run['successful_video_entry']:
                    lines.append(f'      已验证视频：{run["successful_video_entry"]}')
        if asset['accepted_video_entry']:
            lines.append(f'  manifest 接受视频：{asset["accepted_video_entry"]}')
        else:
            lines.append('  本次快照尚无 manifest 接受视频。')
        lines.append('')
    lines += [f'末态复核：{len(terminal_episodes)} 条接受视频；读取问题 {len(terminal_errors)}。',
              f'末态报告：{relative(output / "terminal_state_review.json")}',
              f'其他读取问题：{len(errors)}；详细逐候选记录见 summary.json。']
    (output / 'summary.txt').write_text('\n'.join(lines) + '\n')
    print(json.dumps({'output': relative(output), **c}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
