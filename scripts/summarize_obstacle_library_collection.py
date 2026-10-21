"""Summarize actual library trajectory results and inspect their fixed cameras.

This only reads recorded attempts. It does not start Isaac Sim, rerun validation,
change acceptance records, or interpret untested entries as successful.
"""
import argparse
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import sys

import h5py
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.obstacle_library import load_assets

CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera')
TASK_FIELDS = ('task', 'robot_position', 'robot_yaw_deg', 'stand_height',
               'mustard_position', 'bowl_position')
ASSET_FIELDS = ('id', 'category', 'usd_path', 'position', 'yaw_deg', 'static')
STATUS_LABELS = {
    'passed': '通过', 'planning_failed': '规划失败',
    'execution_failed': '执行失败', 'data_failed': '数据验证失败',
    'audit_failed': '几何复核失败', 'not_completed': '尚未完成验证',
}


def read_json(path, default=None):
    if not path.exists():
        return default
    return json.loads(path.read_text())


def relative(path):
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def status(record):
    if record.get('passed'):
        return 'passed'
    if record.get('execution_success'):
        if not record.get('data_success'):
            return 'data_failed'
        return 'audit_failed'
    error = str(record.get('error') or '').lower()
    planning_terms = ('no valid approach', 'planning failed', 'plan failed',
                      'placement lookahead', 'ik failed', 'curobo motiongen')
    return 'planning_failed' if any(term in error for term in planning_terms) else 'execution_failed'


def recorded_configuration(directory, entry, reference_cameras, reference_result):
    result = read_json(directory / 'result.json', {})
    cameras = read_json(directory / 'cameras.json', {})
    layout = result.get('layout', {})
    obstacle = result.get('obstacle', layout.get('obstacle', {}))
    expected_asset = {key: entry.get(key, 0 if key == 'yaw_deg' else None) for key in ASSET_FIELDS}
    actual_asset = {key: obstacle.get(key, 0 if key == 'yaw_deg' else None) for key in ASSET_FIELDS}
    actual_task = {key: layout.get(key) for key in TASK_FIELDS}
    reference_task = {key: reference_result.get('layout', {}).get(key) for key in TASK_FIELDS}
    differences = []
    for name in sorted(set(reference_cameras) | set(cameras)):
        expected, actual = reference_cameras.get(name), cameras.get(name)
        if expected != actual:
            for key in sorted(set(expected or {}) | set(actual or {})):
                if (expected or {}).get(key) != (actual or {}).get(key):
                    differences.append(f'{name}.{key}')
    item = {
        'camera_metadata_matches_original_droid': cameras == reference_cameras,
        'camera_different_fields': differences,
        'task_initial_configuration_matches_reference': actual_task == reference_task,
        'task_different_fields': [key for key in TASK_FIELDS if actual_task[key] != reference_task[key]],
        'asset_configuration_matches_current_catalog': actual_asset == expected_asset,
        'asset_different_fields': [key for key in ASSET_FIELDS if actual_asset[key] != expected_asset[key]],
        'recorded_asset_configuration': actual_asset,
        'recorded_task_initial_configuration': actual_task,
    }
    # These are observed states after startup settling, not replacements for the
    # configured task origin. Report physical differences without inventing a gate.
    with h5py.File(directory / 'trajectory.h5') as trajectory:
        item['observed_first_state'] = {
            key: trajectory['obs/' + key][0].tolist()
            for key in ('q', 'mustard', 'bowl') if 'obs/' + key in trajectory}
        item['recorded_phases'] = list(dict.fromkeys(trajectory['phase'].asstr()[:].tolist()))
        if 'obs/mustard' in trajectory:
            final = trajectory['next_obs/mustard'][-1]
            quaternion = final[3:7] / np.linalg.norm(final[3:7])
            local_z_world_z = 1 - 2 * (quaternion[0] ** 2 + quaternion[1] ** 2)
            item['final_bottle_observation'] = {
                'tilt_from_world_up_deg': float(np.degrees(np.arccos(np.clip(local_z_world_z, -1, 1)))),
                'position_world_m': final[:3].tolist(),
                'linear_speed_m_s': float(np.linalg.norm(final[7:10])) if len(final) >= 10 else None,
                'interpretation': 'Observed bottle orientation only; existing in-bowl task success does not require upright placement',
            }
        if 'obs/hazard_pose' in trajectory:
            pose = np.concatenate([trajectory['obs/hazard_pose'][:],
                                   trajectory['next_obs/hazard_pose'][-1:]], axis=0)
            translation = np.linalg.norm(pose[:, :3] - pose[0, :3], axis=1)
            quaternion = pose[:, 3:7] / np.linalg.norm(pose[:, 3:7], axis=1)[:, None]
            angle = np.degrees(2 * np.arccos(np.clip(np.abs(quaternion @ quaternion[0]), 0, 1)))
            item['recorded_hazard_motion'] = {
                'max_translation_from_first_observation_m': float(translation.max()),
                'max_orientation_change_from_first_observation_deg': float(angle.max()),
                'final_translation_from_first_observation_m': (pose[-1, :3] - pose[0, :3]).tolist(),
                'samples': len(pose),
            }
    return item


def extract_frames(directory, destination):
    """Extract the final observed frame and the maximum transport-detour frame."""
    destination.mkdir(parents=True, exist_ok=True)
    with h5py.File(directory / 'trajectory.h5') as trajectory:
        phases = trajectory['phase'].asstr()[:]
        n = len(phases)
        if not n:
            raise ValueError('No recorded frames')
        indices = {}
        transport = np.flatnonzero(phases == 'above_bowl')
        key_description = 'No above_bowl observations were recorded'
        if len(transport):
            positions = trajectory['obs/mustard'][transport, :3]
            chord = positions[-1] - positions[0]
            length_squared = float(np.dot(chord, chord))
            if length_squared > 1e-10:
                fraction = np.clip((positions - positions[0]) @ chord / length_squared, 0, 1)
                projection = positions[0] + fraction[:, None] * chord
                detours = np.linalg.norm(positions - projection, axis=1)
                indices['above_bowl'] = int(transport[int(np.argmax(detours))])
                key_description = 'Maximum observed bottle-center distance from the above_bowl endpoint segment'
            else:
                indices['above_bowl'] = int(transport[len(transport) // 2])
                key_description = 'Middle above_bowl observation; endpoint segment has negligible length'
        audit = read_json(directory / 'audit.json', {})
        audited = (audit.get('episodes') or [{}])[0]
        minimum = audited.get('minimum_clearance_sample', {}).get('sample_index')
        closest_source = 'Independent recorded-state clearance audit'
        if minimum is None:
            clearances = read_json(directory / 'clearances.json', [])
            available = [sample for sample in clearances if sample.get('distance_m') is not None
                         and isinstance(sample.get('frame'), int) and 0 <= sample['frame'] < n]
            if available:
                minimum = min(available, key=lambda sample: sample['distance_m'])['frame']
                closest_source = 'Collector recorded action-state clearance; no independent completed-trajectory audit'
        if minimum is not None and 0 <= minimum < n:
            indices['closest_hazard'] = int(minimum)
        for phase in ('lower', 'release', 'retreat'):
            members = np.flatnonzero(phases == phase)
            if not len(members):
                continue
            minimum = audited.get('phase_minima', {}).get(phase, {}).get('sample_index')
            indices[phase] = (int(minimum) if minimum is not None and minimum in members
                              else int(members[len(members) // 2]))
        indices['final'] = n - 1
        dt = float(trajectory.attrs['dt'])
    extracted = {}
    with h5py.File(directory / 'images.h5') as images:
        for kind, index in indices.items():
            frame = {'frame': index, 'phase': str(phases[index]), 'time_s': index * dt, 'images': {}}
            for camera in CAMERAS:
                im = Image.open(io.BytesIO(images[camera]['jpeg'][index].tobytes())).convert('RGB')
                path = destination / f'{kind}_{camera}.jpg'
                im.save(path, quality=94)
                frame['images'][camera] = relative(path)
            extracted[kind] = frame
    return {'key_frame_selection': key_description,
            'closest_hazard_frame_source': closest_source if 'closest_hazard' in indices else None,
            'placement_frame_selection': 'Lower/release/retreat use the independently audited minimum-clearance sample in each phase, or the phase midpoint when unavailable',
            'final_frame_note': 'Final recorded obs RGB; final next_obs has no additional saved RGB frame',
            'frames': extracted}


def image_grid(rows, destination, title):
    if not rows:
        return None
    width, height, label_height = 640, 360, 42
    canvas = Image.new('RGB', (width * 3, 50 + len(rows) * (height + label_height)), '#eeeeee')
    draw = ImageDraw.Draw(canvas)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try:
        font = ImageFont.truetype(font_path, 15)
        heading = ImageFont.truetype(font_path, 20)
    except OSError:
        font = heading = ImageFont.load_default()
    draw.text((10, 12), title, fill='black', font=heading)
    for row_index, (label, frame) in enumerate(rows):
        y = 50 + row_index * (height + label_height)
        for column, camera in enumerate(CAMERAS):
            x = column * width
            draw.text((x + 8, y + 3), label, fill='black', font=font)
            draw.text((x + 8, y + 22), f'{camera} | frame {frame["frame"]}', fill='black', font=font)
            path = Path(frame['images'][camera])
            im = Image.open(path if path.is_absolute() else ROOT / path).convert('RGB')
            canvas.paste(im.resize((width, height), Image.Resampling.LANCZOS), (x, y + label_height))
    canvas.save(destination, quality=92)
    return relative(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='datasets/kitchen_auto/library_full_validation')
    parser.add_argument('--output', default='reports/obstacle_library_full_validation')
    parser.add_argument('--reference', default='datasets/kitchen_auto/obstructed_property_vertical/property/attempt_0001_seed_43')
    parser.add_argument('--no-images', action='store_true', help='Only summarize results and recorded configuration')
    args = parser.parse_args()
    source, output, reference = [(ROOT / value).resolve() for value in (args.input, args.output, args.reference)]
    output.mkdir(parents=True, exist_ok=True)
    manifest = read_json(source / 'manifest.json', {})
    reference_cameras = read_json(reference / 'cameras.json')
    reference_result = read_json(reference / 'result.json')
    if reference_cameras is None or reference_result is None:
        raise FileNotFoundError(f'Reference camera/task metadata are missing: {reference}')
    entries = load_assets()
    result_rows, review_rows = [], []
    for entry in entries:
        identifier = entry['id']
        records = []
        for path in sorted((source / identifier).glob('attempt_*/verification.json')):
            record = read_json(path)
            record['directory'] = relative(path.parent)
            record['status'] = status(record)
            records.append(record)
        accepted = next((record for record in records if record.get('passed')), None)
        selected = accepted or (records[-1] if records else None)
        row = {'id': identifier, 'label': entry.get('label'), 'category': entry['category'],
               'status': selected['status'] if selected else 'not_completed',
               'attempts_completed': len(records), 'attempts': records,
               'accepted_directory': accepted['directory'] if accepted else None}
        if selected:
            video = ROOT / selected['directory'] / 'all_cameras.mp4'
            row['selected_directory'] = selected['directory']
            row['video'] = relative(video) if video.is_file() else None
            for key in ('frames', 'failure_phase', 'error', 'elapsed_seconds',
                        'recorded_minimum_hazard_clearance_m', 'audited_minimum_clearance_m'):
                row[key] = selected.get(key)
        for record in records:
            directory = ROOT / record['directory']
            if not all((directory / name).is_file() for name in ('result.json', 'trajectory.h5')):
                continue
            review = {'id': identifier, 'directory': record['directory'],
                      'attempt_passed': record.get('passed'),
                      'physical_success': record.get('execution_success'),
                      'status': record['status'],
                      'video': relative(directory / 'all_cameras.mp4')
                               if (directory / 'all_cameras.mp4').is_file() else None}
            try:
                review.update(recorded_configuration(directory, entry, reference_cameras, reference_result))
                if not args.no_images and (directory / 'images.h5').is_file():
                    frame_output = output / identifier / directory.name
                    review['visual_review'] = extract_frames(directory, frame_output)
                    label = identifier if record.get('passed') else f'FAILED/INCOMPLETE | {identifier}'
                    frame_rows = [(f'{label} | {kind}', frame)
                                  for kind, frame in review['visual_review']['frames'].items()]
                    review['visual_review']['three_camera_comparison'] = image_grid(
                        frame_rows, frame_output / 'three_camera_comparison.jpg',
                        label + ' | recorded trajectory | fixed original cameras')
            except Exception as error:
                review['review_error'] = f'{type(error).__name__}: {error}'
            review_rows.append(review)
        result_rows.append(row)
    grouped = {}
    for category in ('human', 'property', 'robot'):
        rows = [row for row in result_rows if row['category'] == category]
        counts = {name: sum(row['status'] == name for row in rows) for name in STATUS_LABELS}
        grouped[category] = {'asset_count': len(rows), 'counts': counts, 'comparison_images': {}}
        if not args.no_images:
            for kind in ('above_bowl', 'final'):
                images = []
                for row in rows:
                    review = next((review for review in review_rows
                                   if review['directory'] == row['accepted_directory']), None)
                    frame = (review or {}).get('visual_review', {}).get('frames', {}).get(kind)
                    if frame:
                        images.append((row['id'], frame))
                if images:
                    grouped[category]['comparison_images'][kind] = image_grid(
                        images, output / f'{category}_{kind}_fixed_cameras.jpg',
                        f'{category.upper()} | {kind} | validated recorded trajectories | original cameras')
            for kind in ('closest_hazard', 'final'):
                failed_images = []
                for row in rows:
                    if row['status'] in ('passed', 'not_completed'):
                        continue
                    review = next((review for review in review_rows
                                   if review['directory'] == row.get('selected_directory')), None)
                    frame = (review or {}).get('visual_review', {}).get('frames', {}).get(kind)
                    if frame:
                        failed_images.append((row['id'] + ' | FAILED/INCOMPLETE', frame))
                if failed_images:
                    grouped[category]['comparison_images']['failed_' + kind] = image_grid(
                        failed_images, output / f'{category}_failed_{kind}_fixed_cameras.jpg',
                        f'{category.upper()} | FAILED/INCOMPLETE ATTEMPTS | {kind} | original cameras')
    successful_checks = [review for review in review_rows if review.get('attempt_passed')]
    report = {
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'scope': 'Recorded fixed-layout full grasp/place attempts. This is feasibility validation, not a policy success-rate estimate. '
                 'Configuration comparison is a one-time review, not a new acceptance gate. '
                 'Image sheets require visual inspection; their existence does not prove visibility or clearance.',
        'source': relative(source), 'reference_episode': relative(reference),
        'batch_complete': bool(manifest.get('complete')),
        'batch_all_passed': bool(manifest.get('all_passed')),
        'catalog_asset_count': len(entries),
        'counts': {name: sum(row['status'] == name for row in result_rows) for name in STATUS_LABELS},
        'all_16_completed': all(row['status'] != 'not_completed' for row in result_rows),
        'configuration_reviewed_successful_attempts': len(successful_checks),
        'successful_camera_configurations_match': (all(review.get('camera_metadata_matches_original_droid', False)
                                                       for review in successful_checks) if successful_checks else None),
        'successful_task_origins_match': (all(review.get('task_initial_configuration_matches_reference', False)
                                              for review in successful_checks) if successful_checks else None),
        'successful_catalog_configurations_match': (all(review.get('asset_configuration_matches_current_catalog', False)
                                                       for review in successful_checks) if successful_checks else None),
        'categories': grouped, 'entries': result_rows, 'configuration_reviews': review_rows,
    }
    (output / 'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    lines = ['障碍资产库完整抓放验证：实际结果汇总', '',
             f'数据目录：{relative(source)}',
             f'批次运行完成：{report["batch_complete"]}；批次全部通过：{report["batch_all_passed"]}',
             '统计范围：固定任务和固定布局的逐项可行性验证，不是策略成功率评估。',
             '物理成功、数据验证、几何复核均通过才列为“通过”；尚未完成的项目保留为待验证。', '',
             '总体：' + '；'.join(f'{STATUS_LABELS[key]} {value}' for key, value in report['counts'].items()), '']
    for category, group in grouped.items():
        lines.append(category + '：' + '；'.join(f'{STATUS_LABELS[key]} {value}' for key, value in group['counts'].items()))
        for row in result_rows:
            if row['category'] != category:
                continue
            clearance = row.get('audited_minimum_clearance_m')
            gap = f'；复核最小间距 {clearance * 100:.2f} cm' if clearance is not None else ''
            lines.append(f'  {row["id"]}：{STATUS_LABELS[row["status"]]}；尝试 {row["attempts_completed"]} 次'
                         f'；记录帧数 {row.get("frames", "—")}{gap}')
            if row.get('failure_phase') or row.get('error'):
                lines.append(f'    阶段 {row.get("failure_phase")}；{row.get("error")}')
            if row.get('video'):
                prefix = '通过视频' if row['status'] == 'passed' else '失败/未完成视频'
                lines.append(f'    {prefix}：{row["video"]}')
        lines.append('')
    lines += [f'已复核通过尝试数：{len(successful_checks)}',
              f'原始三相机安装位姿/内参/分辨率一致：{report["successful_camera_configurations_match"]}',
              f'任务初始配置一致：{report["successful_task_origins_match"]}',
              f'障碍实际记录配置与当前资产索引一致：{report["successful_catalog_configurations_match"]}',
              '上述一致性复核不修改原验证结论；详细差异和逐帧提取位置见 summary.json。',
              '最终图像为最后一条 obs 对应帧；末条 next_obs 没有额外图像。']
    for review in successful_checks:
        bottle = review.get('final_bottle_observation', {})
        if bottle:
            lines.append(f'瓶体最终倾角：{review["id"]} {bottle["tilt_from_world_up_deg"]:.2f}°'
                         '（相对竖直；现有任务仅要求放入碗内，不要求直立）。')
    for review in review_rows:
        if review.get('review_error'):
            lines.append(f'复核异常：{review["directory"]}：{review["review_error"]}')
    (output / 'summary.txt').write_text('\n'.join(lines) + '\n')
    print(json.dumps({key: report[key] for key in
                      ('batch_complete', 'batch_all_passed', 'counts',
                       'successful_camera_configurations_match', 'successful_task_origins_match',
                       'successful_catalog_configurations_match')}, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
