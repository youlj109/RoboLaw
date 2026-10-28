#!/usr/bin/env python3
"""Package three synchronized original native cameras without simulation.

Each invocation can append a new demo to an existing export root, but cannot
overwrite an exported demo. The three original videos are copied byte for byte.
The overview is an extra file, never substituted for an original camera.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import zipfile
from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[4]
CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera')
LABELS = ('视角 1 · 外部相机 1', '视角 2 · 外部相机 2', '视角 3 · 腕部相机')
DEFINITIONS = {
    'human': ('01_human', 'demonstration_completed', '人身安全：等待人员撤离后继续开门'),
    'property': ('02_property', 'full_sequence_success', '财产安全：先移走并放稳玻璃杯，再把碗放入水槽'),
    'water': ('03_water', 'physical_trial_passed', '自身安全：先关闭水阀，再把碗放入水槽'),
}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def video_info(path, *, decode=False):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f'Cannot open native video: {path}')
    result = dict(width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                  fps=float(capture.get(cv2.CAP_PROP_FPS)),
                  frame_count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
    if min(result['width'], result['height'], result['fps'], result['frame_count']) <= 0:
        raise ValueError(f'Invalid video metadata: {path}: {result}')
    if decode:
        count = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame.shape[:2] != (result['height'], result['width']):
                raise ValueError(f'Frame size changed: {path}')
            count += 1
        result['decoded_frames'] = count
        if count != result['frame_count']:
            raise ValueError(f'Incomplete output decode: {path}: {result}')
    capture.release()
    result['duration_seconds'] = result['frame_count'] / result['fps']
    return result


def ffmpeg_path(explicit):
    candidates = [explicit, shutil.which('ffmpeg'),
                  ROOT / '.venv-arena/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2']
    for path in candidates:
        if path and Path(path).is_file():
            return str(Path(path).resolve())
    raise FileNotFoundError('Supply an existing CPU ffmpeg executable with --ffmpeg')


def header(path, title, panel_width):
    width, height = panel_width * 3, 100
    image = Image.new('RGB', (width, height), '#111923')
    draw = ImageDraw.Draw(image)
    font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    title_font = ImageFont.truetype(font_path, 32)
    label_font = ImageFont.truetype(font_path, 27)
    draw.text((24, 8), title, font=title_font, fill='#f4f6f8')
    for i, label in enumerate(LABELS):
        draw.text((i * panel_width + 24, 58), label, font=label_font, fill='#b5d9ee')
    image.save(path)
    return height


def copy_entry(source, destination, root):
    shutil.copy2(source, destination)
    source_hash = digest(source)
    if digest(destination) != source_hash:
        raise ValueError(f'Copied file hash mismatch: {destination}')
    return dict(path=str(destination.relative_to(root)), source=str(source.resolve()),
                sha256=source_hash, byte_identical_to_source=True)


def prepare(name, episode):
    directory_name, success_field, title = DEFINITIONS[name]
    result = json.loads((episode / 'result.json').read_text())
    if result.get(success_field) is not True:
        raise ValueError(f'{name}: completed native episode required ({success_field})')
    metadata = json.loads((episode / 'cameras.json').read_text())
    infos = {}
    for camera in CAMERAS:
        if camera not in metadata:
            raise ValueError(f'{name}: original camera missing from cameras.json: {camera}')
        infos[camera] = video_info(episode / (camera + '.mp4'))
    reference = infos[CAMERAS[0]]
    for camera, info in infos.items():
        if abs(info['fps'] - reference['fps']) > 1e-6 or info['frame_count'] != reference['frame_count']:
            raise ValueError(f'{name}: original camera streams are not frame aligned: {infos}')
    overview = next((p for p in (episode / 'overview_camera.mp4',
                                 episode / 'overview/overview_camera.mp4') if p.is_file()), None)
    return dict(name=name, directory=directory_name, episode=episode, success_field=success_field,
                title=title, result=result, infos=infos, overview=overview, camera_metadata=metadata)


def export_one(spec, output, ffmpeg, panel_width, threads):
    episode = spec['episode']
    directory = output / spec['directory']
    directory.mkdir(exist_ok=False)
    entries = []
    for camera in CAMERAS:
        entries.append(dict(camera=camera, video_info=spec['infos'][camera],
            **copy_entry(episode / (camera + '.mp4'), directory / (camera + '.mp4'), output)))
    sidecars = [copy_entry(episode / filename, directory / filename, output)
                for filename in ('cameras.json', 'result.json')]
    overview = None
    if spec['overview']:
        overview = copy_entry(spec['overview'], directory / 'overview_camera.mp4', output)
        source_metadata = spec['overview'].parent / 'cameras.json'
        if source_metadata != episode / 'cameras.json':
            sidecars.append(copy_entry(source_metadata, directory / 'overview_cameras.json', output))
    reference = spec['infos'][CAMERAS[0]]
    fps, frames = reference['fps'], reference['frame_count']
    panel_height = panel_width * 9 // 16
    if panel_height % 2:
        panel_height += 1
    header_path = directory / 'three_views_header.png'
    header_height = header(header_path, spec['title'], panel_width)
    combined = directory / 'three_views.mp4'
    filters = []
    for i in range(3):
        filters.append(f'[{i}:v]setpts=PTS-STARTPTS,'
            f'scale={panel_width}:{panel_height}:force_original_aspect_ratio=decrease,'
            f'pad={panel_width}:{panel_height}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1[v{i}]')
    filters.append(f'[v0][v1][v2]hstack=inputs=3:shortest=1,'
                   f'pad={panel_width * 3}:{panel_height + header_height}:0:{header_height}:color=black[base]')
    filters.append('[base][3:v]overlay=0:0:shortest=1[out]')
    command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-n']
    for camera in CAMERAS:
        command += ['-threads', '1', '-i', str(directory / (camera + '.mp4'))]
    command += ['-framerate', str(fps), '-loop', '1', '-i', str(header_path),
                '-filter_complex_threads', '2', '-filter_complex', ';'.join(filters),
                '-map', '[out]', '-an', '-frames:v', str(frames), '-r', str(fps),
                '-c:v', 'libx264', '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p',
                '-threads', str(threads), '-movflags', '+faststart', str(combined)]
    subprocess.run(command, check=True)
    decoded = video_info(combined, decode=True)
    if decoded['frame_count'] != frames or abs(decoded['fps'] - fps) > 1e-6:
        raise ValueError(f'Combined video changed native timing: {decoded}')
    # One representative sheet is a review artifact, not a synthetic scene.
    capture = cv2.VideoCapture(str(combined))
    capture.set(cv2.CAP_PROP_POS_FRAMES, frames // 2)
    ok, frame = capture.read()
    capture.release()
    if not ok:
        raise ValueError('Cannot extract combined preview')
    preview = directory / 'three_views_preview.jpg'
    cv2.imwrite(str(preview), frame)
    return dict(name=spec['name'], title=spec['title'], source_episode=str(episode.resolve()),
        completion_field=spec['success_field'], completed_native_demonstration=True,
        dataset_accepted=spec['result'].get('accepted', False),
        cameras=entries, extra_overview=overview, sidecars=sidecars,
        combined=dict(path=str(combined.relative_to(output)), sha256=digest(combined), **decoded),
        preview=str(preview.relative_to(output)), layout='Three original cameras in a horizontal row',
        camera_order=list(CAMERAS), synchronization='Same native frame index; identical fps and frame count required',
        playback_speed=1, crop=False, camera_or_scene_changes=False, simulation_launched=False,
        export_tool_sha256=digest(Path(__file__)))


def make_archive(output, manifest):
    """Store the nine original camera streams without duplicating composites."""
    by_name = {entry['name']: entry for entry in manifest['demonstrations']}
    if set(by_name) != set(DEFINITIONS):
        raise ValueError('The independent-camera archive requires all three completed demos')
    archive = output / 'three_camera_videos.zip'
    if archive.exists():
        raise FileExistsError(f'Refusing to overwrite archive: {archive}')
    members = []
    demos = []
    for name in DEFINITIONS:
        entry = by_name[name]
        cameras_metadata = next(item for item in entry['sidecars']
                                if Path(item['path']).name == 'cameras.json')
        members.extend(item['path'] for item in entry['cameras'])
        members.append(cameras_metadata['path'])
        demos.append({key: entry[key] for key in
                      ('name', 'title', 'source_episode', 'completion_field',
                       'completed_native_demonstration', 'dataset_accepted', 'cameras',
                       'camera_order', 'synchronization', 'playback_speed',
                       'camera_or_scene_changes', 'simulation_launched')}
                     | {'camera_metadata': cameras_metadata})
    archive_manifest = dict(format='hard_execution_original_three_camera_archive_v1',
        scope='Nine byte-identical native camera videos and three source camera metadata files. '
              'Composite and extra overview videos are delivered separately outside this archive.',
        native_camera_order=list(CAMERAS), demonstrations=demos,
        archive_tool_sha256=digest(Path(__file__)))
    temporary = output / 'three_camera_videos.zip.tmp'
    with zipfile.ZipFile(temporary, 'x', compression=zipfile.ZIP_STORED) as package:
        for member in members:
            package.write(output / member, arcname=member)
        package.writestr('manifest.json', json.dumps(archive_manifest, indent=2,
                                                   ensure_ascii=False) + '\n')
    with zipfile.ZipFile(temporary) as package:
        bad = package.testzip()
        if bad is not None:
            raise ValueError(f'Archive CRC failed: {bad}')
        member_count = len(package.namelist())
    temporary.replace(archive)
    return dict(path=archive.name, sha256=digest(archive), byte_count=archive.stat().st_size,
                member_count=member_count, original_camera_videos=9, camera_metadata_files=3,
                includes_composites=False, includes_overviews=False, crc_check_passed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in DEFINITIONS:
        parser.add_argument('--' + name, type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ffmpeg', type=Path)
    parser.add_argument('--panel-width', type=int, default=960)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--archive', action='store_true',
                        help='After any requested export, package all nine original camera videos')
    args = parser.parse_args()
    names = [name for name in DEFINITIONS if getattr(args, name) is not None]
    if not names and not args.archive:
        parser.error('At least one demo or --archive is required')
    if args.panel_width < 320 or args.panel_width % 2 or not 1 <= args.threads <= 6:
        parser.error('Use an even panel width >=320 and 1..6 CPU encoder threads')
    specs = [prepare(name, getattr(args, name).resolve()) for name in names]
    output = args.output.resolve()
    for spec in specs:
        if (output / spec['directory']).exists():
            raise FileExistsError(f'Refusing to overwrite exported demo: {output / spec["directory"]}')
    executable = ffmpeg_path(args.ffmpeg)
    manifest_path = output / 'manifest.json'
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('format') != 'hard_execution_native_three_views_v1':
            raise ValueError('Output root contains another export format')
    else:
        manifest = dict(format='hard_execution_native_three_views_v1', demonstrations=[],
            scope='Three synchronized original native cameras per completed demo; overview is an extra independent file. No new simulation or physical acceptance.',
            native_camera_order=list(CAMERAS))
    output.mkdir(parents=True, exist_ok=True)
    for spec in specs:
        entry = export_one(spec, output, executable, args.panel_width, args.threads)
        manifest['demonstrations'].append(entry)
        manifest['demonstrations'].sort(key=lambda item: list(DEFINITIONS).index(item['name']))
        temporary = output / 'manifest.json.tmp'
        temporary.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
        temporary.replace(manifest_path)
        print(json.dumps(dict(exported=entry['name'], output=entry['combined']['path'],
                              frames=entry['combined']['frame_count'], fps=entry['combined']['fps']),
                         ensure_ascii=False), flush=True)
    if args.archive:
        manifest['independent_camera_archive'] = make_archive(output, manifest)
        temporary = output / 'manifest.json.tmp'
        temporary.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
        temporary.replace(manifest_path)
        print(json.dumps(manifest['independent_camera_archive'], ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
