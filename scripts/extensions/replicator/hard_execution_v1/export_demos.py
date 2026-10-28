#!/usr/bin/env python3
"""Export completed native demonstrations and their recorded trajectories.

This packaging step does not simulate, retime robot actions, or grant dataset
acceptance. The combined preview preserves each source video's 15 Hz timing.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

import numpy as np


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def export_valve_states(episode, directory):
    """Preserve recorded valve readbacks without duplicating large contact JSON."""
    source = episode / 'water_physics.json'
    observed = json.loads(source.read_text())
    fields = ('physics_step', 'phase', 'action_index', 'angle_deg', 'speed_deg_s',
              'flowing', 'lever_pose', 'base_pose', 'gripper_observed_joint_positions_rad',
              'gripper_processed_target_rad')
    rows = observed['rows']
    target = directory / 'valve_states.npz'
    np.savez_compressed(target, **{key: np.asarray([row[key] for row in rows]) for key in fields})
    metadata = directory / 'valve_states_metadata.json'
    metadata.write_text(json.dumps(dict(source=str(source.resolve()), source_sha256=digest(source),
        samples=len(rows), fields=list(fields), values='Unmodified native readbacks from matching physics substeps',
        success_rule='Actual water off after release and throughout bowl placement; angular speed is diagnostic only'),
        indent=2, ensure_ascii=False) + '\n')
    return [target, metadata]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('human', 'property', 'water'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    definitions = [
        ('01_human', args.human, 'overview_camera.mp4', 'demonstration_completed',
         ['trajectory.h5', 'trajectory.json', 'metadata.json', 'events.json'],
         'Human safety: pause for the person, then resume opening'),
        ('02_property', args.property, 'overview/overview_camera.mp4', 'full_sequence_success',
         ['physics_states.npz', 'physics_metadata.json', 'actions.json'],
         'Property safety: move the glass aside, then place the bowl in the sink'),
        ('03_water', args.water, 'external_camera_2.mp4', 'physical_trial_passed',
         ['physics_states.npz', 'physics_metadata.json', 'actions.json', 'trajectory.npz'],
         'Robot safety: close the valve, then place the bowl in the sink'),
    ]
    results = {}
    for name, episode, video, gate, files, title in definitions:
        results[name] = json.loads((episode / 'result.json').read_text())
        if results[name].get(gate) is not True:
            raise ValueError(f'{name}: completed native execution required ({gate})')
        for source in [video, 'result.json', *files]:
            if not (episode / source).is_file():
                raise FileNotFoundError(episode / source)
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(scope='Single native physical demonstrations; independent dataset acceptance remains separate.',
                    playback_speed=1, native_rendering=True, demonstrations=[])
    normalized = []
    for name, episode, video, gate, files, title in definitions:
        directory = args.output / name
        directory.mkdir()
        entries = []
        for source in [video, 'result.json', *files]:
            destination = directory / ('execution.mp4' if source == video else Path(source).name)
            shutil.copy2(episode / source, destination)
            entries.append(dict(path=str(destination.relative_to(args.output)), sha256=digest(destination)))
        if name == '03_water':
            for destination in export_valve_states(episode, directory):
                entries.append(dict(path=str(destination.relative_to(args.output)), sha256=digest(destination)))
        preview = args.output / (name + '_preview.mp4')
        caption = directory / 'caption.txt'
        caption.write_text(title)
        # All paths are passed as arguments; no shell or command substitution.
        filters = ("scale=1280:720:force_original_aspect_ratio=decrease,"
                   "pad=1280:768:(ow-iw)/2:48+(720-ih)/2:color=black,setsar=1,"
                   f"drawtext=textfile='{caption.resolve()}':fontcolor=white:fontsize=24:x=24:y=12")
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-n', '-i',
                        str(directory / 'execution.mp4'), '-vf', filters, '-an', '-c:v', 'libx264',
                        '-crf', '20', '-pix_fmt', 'yuv420p', '-r', '15', str(preview)], check=True)
        normalized.append(preview)
        task_goals = {
            '01_human': '开冰箱门时停下等待人撤手并离开，然后继续开门。',
            '02_property': '先移走碗承载的玻璃杯并放稳，再将碗放入水槽。',
            '03_water': '把台面上的碗放进正在流水的水槽：先关阀确认停水，再抓碗并放入水槽。',
        }
        manifest['demonstrations'].append(dict(name=name, task_goal=task_goals[name], source_episode=str(episode.resolve()),
            completion_field=gate, demonstration_completed=True,
            dataset_accepted=results[name].get('accepted', False), files=entries))
    listing = args.output / 'concat.txt'
    listing.write_text(''.join(f"file '{path.name}'\n" for path in normalized))
    combined = args.output / 'three_robot_executions.mp4'
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-n', '-f', 'concat', '-safe', '0',
                    '-i', str(listing), '-c', 'copy', '-movflags', '+faststart', str(combined)], check=True)
    manifest['combined_preview'] = dict(path=combined.name, sha256=digest(combined))
    manifest['trajectory_archive'] = dict(path='trajectories.zip',
        contents='Recorded actions, native physical states, per-demo results and this manifest; videos are provided separately.')
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    with zipfile.ZipFile(args.output / 'trajectories.zip', 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        archive.write(args.output / 'manifest.json', 'manifest.json')
        for demo in manifest['demonstrations']:
            for entry in demo['files']:
                source = args.output / entry['path']
                if source.suffix != '.mp4':
                    archive.write(source, entry['path'])
    print(json.dumps(dict(output=str(args.output.resolve()), demonstrations=len(definitions))))


if __name__ == '__main__':
    main()
