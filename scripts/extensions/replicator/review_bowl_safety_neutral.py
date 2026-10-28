"""CPU-only independent review of the explicit hazard-neutral schema.

Recompute every recorded numeric/native-contact gate and source FCL state,
verify archived input identities, then decode all six videos. No visual pass is
generated. Base neutral/task acceptance are deliberately separate interfaces.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def review(directory):
    from kitchen_safety.extensions.replicator.bowl_safety_neutral_evidence import audit_neutral, sha, require
    from kitchen_safety.extensions.replicator.bowl_safety_cpu import build_world
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from kitchen_safety.dataset import decode_video
    directory = Path(directory).resolve()
    report = json.loads((directory/'neutral_probe.json').read_text())
    provenance = json.loads((directory/'task_collection_provenance.json').read_text())
    hazard = json.loads((directory/'hazard_contacts.json').read_text())
    with np.load(directory/'neutral_states.npz', allow_pickle=False) as raw:
        data = {key: raw[key] for key in raw.files}
    inputs = {'neutral_probe.json': sha(directory/'neutral_probe.json'),
        'review_source': sha(__file__)}
    for key, expected in report['input_sha256'].items():
        path = Path(key) if Path(key).is_absolute() else directory/key
        actual = sha(path)
        require(actual == expected, 'Changed recorded neutral input: '+key)
        inputs[key] = actual
    entries = provenance['entries']
    require(provenance['source_layout'] == report['layout'] and provenance['background_physics'] == report['background_physics'],
        'Composed scene/provenance identity differs')
    require(provenance['checked_physics_step'] == int(data['physics_step'][-1]), 'Provenance ends before observation')
    for key, row in entries.items():
        require(row['initial_sha256'] == row['checked_sha256'] == row.get('expected_sha256', row['initial_sha256']) and
            row['unchanged'] is True and row['matches_declared_source'] is True, 'Runtime source changed: '+key)
        if 'source_text' in row:
            require(hashlib.sha256(row['source_text'].encode()).hexdigest() == row['initial_sha256'], 'Archived source text mismatch: '+key)
    # Replayed geometry must be the same implementation as the actual probe.
    sources = ['kitchen_safety/planning.py', *['kitchen_safety/extensions/replicator/'+n+'.py' for n in
        ('bowl_safety_neutral_evidence','bowl_safety_cpu','bowl_collision','task_collision','collision_planner',
         'task_contact_policy','task_motion','bowl_safety_recorder','bowl_safety_policy')],
        'scripts/extensions/replicator/neutral_task_probe.py']
    for key in sources:
        require(sha(ROOT/key) == entries[key]['initial_sha256'], 'Cannot replay with different source: '+key)
    numeric = audit_neutral(data, report, hazard)
    world, _, _ = build_world(report['layout'], report['hazard_policy'])
    world.set_phase('initial_settle')
    arms = [report['joint_names'].index(f'panda_joint{i}') for i in range(1,8)]
    fingers = [report['joint_names'].index(n) for n in world.GRIPPER_JOINT_NAMES]
    source_rows = []
    for i, step in enumerate(data['physics_step']):
        q = data['q'][i]
        bowl = data['task_object_states'][i,0,:7]
        world.sync_background(report['background_paths'], data['background'][i])
        world.set_obstacle_pose('bowl', bowl)
        world.set_obstacle_pose(report['hazard_policy']['hazard_name'], data['hazard_pose_w'][i])
        world.payload_pose = pose_matrix(bowl)
        world.gripper_joint_positions, world.gripper_joint_position = q[fingers].copy(), float(q[fingers][0])
        valid = world.valid(q[arms],mode='execution',gripper_q=q[fingers],object_pose=bowl)
        source_rows.append(dict(physics_step=int(step), valid=bool(valid), failure=world.last_clearance))
    source_ok = len(source_rows) == len(data['physics_step']) and all(r['valid'] for r in source_rows)
    videos = {}
    expected = [(name+'.mp4',(1280,720)) for name in report['original_camera_names']] + [
        ('all_cameras.mp4',(1920,360)),('overview/overview_camera.mp4',(1280,720)),('overview/all_cameras.mp4',(640,360))]
    require(len(expected) == 6 and set(report['video_validation']) == {name for name,_ in expected}, 'Missing original six videos')
    for key,size in expected:
        actual = sha(directory/key)
        require(actual == report['video_validation'][key]['sha256'], 'Changed video: '+key)
        videos[key] = dict(sha256=actual, frames=decode_video(directory/key,report['frames'],expected_size=size,expected_dt=report['dt']))
    return dict(schema_version=1, kind='matched_hazard_neutral_independent_review',
        passed=bool(numeric['passed'] and source_ok and report['passed'] is True),
        accepted=False, task_success=False, physical_trial_passed=False,
        numeric=numeric, source_geometry_all_actual_states=source_ok, source_geometry=source_rows,
        videos=videos, input_sha256=inputs, visual_review='Pending; complete decode is not semantic visual inspection.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--neutral',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    try: result=review(args.neutral)
    except Exception as error:
        result=dict(passed=False,accepted=False,task_success=False,physical_trial_passed=False,error=repr(error))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False,default=lambda x:x.tolist())
        stream.write('\n')
    print(json.dumps(dict(passed=result['passed'],output=str(args.output))))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
