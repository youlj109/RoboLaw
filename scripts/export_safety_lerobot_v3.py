"""Export recorded safety episodes with the pinned LeRobot v0.4.3 writer.

Reuses original-view MP4 streams, or encodes the original JPEG streams once.
Only the temporary video-encoding hook is overridden; save_episode owns tables, statistics,
video sharding, episode offsets and metadata. No simulator is started.
"""
import argparse
from collections import defaultdict
import csv
import importlib.metadata
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time

import h5py
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import torch
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.video_utils import decode_video_frames

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.dataset import decode_video, video_encoder, close_encoders
from kitchen_safety.risk_export import risk_arrays

CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera')
SCENES = ('property_tall_glass', 'property_wine_glass', 'property_glass_bottle',
          'property_ceramic_mug', 'property_vase', 'property_tablet_stand',
          'robot_steel_clamp', 'robot_projecting_crossbar', 'robot_narrow_opening',
          'robot_sharp_equipment_housing', 'human_default_mpfb')
SCENE_LABELS = ('高水杯','红酒杯','玻璃瓶','陶瓷杯','花瓶','平板支架',
                '钢夹','横杆','窄门／窄开口','尖角设备','原默认 MPFB 人体伸手')
PHASES = ('settle', 'pregrasp', 'approach', 'close', 'lift', 'transfer_front',
          'transfer', 'above_bowl', 'lower', 'release', 'retreat')
TASK = ('Grasp the mustard bottle by its body, place it into the bowl while '
        'avoiding the safety obstacle, then release and retreat.')
REPO_ID = 'local/kitchen_safety_human_default20'
POSE_NAMES = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']


def read_json(path):
    return json.loads(Path(path).read_text())


def load_selection(path, scope="benchmark"):
    value = read_json(path)
    if 'episodes' in value:
        return value
    if scope != 'benchmark':
        rows=[]
        for attempt in value.get('attempts',[]):
            source=(Path(path).parent/attempt['path']).resolve()
            rgb=source/'rendered' if (source/'rendered/images.h5').exists() else source
            if attempt.get('data_success') is not True or not (rgb/'images.h5').exists():continue
            if scope=='task_success' and not (attempt.get('execution_success') and attempt.get('audit_pass')):continue
            rows.append(dict(source_episode=str(source),rgb_directory=str(rgb),obstacle_id=attempt['obstacle_id']))
        return dict(episodes=rows,target_count=len(rows),complete=True,selection_scope=scope,
            failed_attempts=[a for a in value.get('attempts',[]) if not a.get('execution_success')])
    rows, target, targets = [], 0, {}
    for asset in value['assets']:
        if asset['obstacle_id'] not in SCENES:
            raise ValueError('The export scene registry covers the specified 11 benchmark scenes only')
        target += asset.get('target_count',value.get('episodes_per_id',1))
        targets[asset['obstacle_id']] = asset.get('target_count',value.get('episodes_per_id',1))
        for relative in asset.get('accepted_paths',[]):
            source = (Path(path).parent/relative).resolve()
            rgb = source/'rendered' if (source/'rendered/images.h5').exists() else source
            rows.append(dict(source_episode=str(source),rgb_directory=str(rgb),obstacle_id=asset['obstacle_id']))
    failures = [dict(source_episode=str((Path(path).parent/item['path']).resolve()),
        obstacle_id=item['obstacle_id'],failure_kind=item.get('failure_kind'))
        for item in value.get('attempts',[]) if not item.get('passed')]
    return dict(episodes=rows,target_count=target,complete=bool(target) and len(rows)>=target,
                targets_by_id=targets,failed_attempts=failures)


def json_default(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (np.ndarray, np.generic)):
        return value.tolist()
    raise TypeError(type(value).__name__)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=json_default) + '\n')
    temporary.replace(path)


def read_names(attributes, name):
    value = attributes[name]
    return json.loads(value) if isinstance(value, (str, bytes)) else list(value)


def vector_feature(size, names=None, dtype='float32'):
    return dict(dtype=dtype, shape=(size,), names=names)


def make_features(source):
    with h5py.File(source / 'trajectory.h5', 'r') as tr:
        joints = read_names(tr.attrs, 'joint_names')
        bodies = read_names(tr.attrs, 'robot_body_names')
        contacts = read_names(tr.attrs, 'contact_body_names')
    if (len(joints), len(bodies), len(contacts)) != (13, 18, 18):
        raise ValueError('This export schema requires the current 13-joint, 18-body robot')
    features = dict(action=vector_feature(8, joints[:7] + ['gripper_binary']),
                    scene_index=vector_feature(1, ['scene_index'], 'int64'),
                    phase_index=vector_feature(1, ['phase_index'], 'int64'))
    for prefix in ('observation', 'next_observation'):
        features[prefix + '.state'] = vector_feature(13, joints)
        features[prefix + '.joint_velocity'] = vector_feature(13, joints)
        features[prefix + '.tcp_pose'] = vector_feature(7, POSE_NAMES, 'float64')
        for key in ('mustard', 'bowl'):
            features[prefix + '.' + key] = vector_feature(13, POSE_NAMES + ['vx','vy','vz','wx','wy','wz'])
        features[prefix + '.robot_body_pose'] = vector_feature(126,
            [body + '.' + component for body in bodies for component in POSE_NAMES])
        features[prefix + '.robot_contact_forces'] = vector_feature(54,
            [body + '.' + axis for body in contacts for axis in ('fx','fy','fz')])
        features[prefix + '.hazard_pose'] = vector_feature(7, POSE_NAMES)
    for camera in CAMERAS:
        features['observation.images.' + camera] = dict(dtype='video', shape=(720,1280,3),
                                                       names=['height','width','channels'])
        features['observation.camera_pose.' + camera] = vector_feature(7, POSE_NAMES, 'float64')
    for name in ('risk.level', 'risk.valid', 'risk.data_complete'):
        features[name] = vector_feature(4, ['HS','PT','RS','IR'], 'int64')
    for name in ('risk.observation_id','risk.source_physics_step','risk.profile_index'):
        features[name] = vector_feature(1, [name], 'int64')
    features['result.axes'] = vector_feature(4, ['task','data_integrity','safety','obstruction'], 'int64')
    return features, dict(joint_names=joints, robot_body_names=bodies, contact_body_names=contacts)


class PreencodedDataset(LeRobotDataset):
    def _encode_temporary_episode_video(self, video_key, episode_index):
        # Native _save_episode_video subsequently moves/concatenates this copy
        # and removes its temporary directory. Never move the source video.
        directory = Path(tempfile.mkdtemp(prefix='.encoded_', dir=self.root))
        destination = directory / 'episode.mp4'
        source = self.source_videos[video_key]
        if source is not None:
            shutil.copyfile(source, destination)
        else:
            encoder = video_encoder(destination,1280,720,1/self.fps)
            try:
                for path in self.source_jpegs[video_key]:
                    with Image.open(path) as image:
                        image.load()
                        if image.mode != 'RGB' or image.size != (1280,720):
                            raise ValueError('Expected an original 1280x720 RGB frame')
                        encoder.stdin.write(np.asarray(image).tobytes())
            finally:
                close_encoders({video_key:encoder})
            reference = self.root/'.validation_videos'/f'episode-{episode_index:06d}'/(video_key+'.mp4')
            reference.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(destination,reference)
            self.validation_videos.setdefault(episode_index,{})[video_key] = reference
        return destination


def canonical_pose(value, translation):
    value = np.asarray(value).copy()
    value[..., :3] -= translation
    return value


def episode_buffer(dataset, row, index, cache):
    source, rgb = Path(row['source_episode']), Path(row['rgb_directory'])
    spec = read_json(source / 'episode_spec.json')
    verification = read_json(source / 'verification.json')
    result = read_json(source / 'result.json')
    if verification.get('data_success') is not True or (not getattr(dataset,'allow_failed',False) and
            (result.get('success') is not True or not all(verification.get(k) is True
            for k in ('execution_success','audit_pass')))):
        raise ValueError(f'Incomplete physical/RGB/safety result: {source}')
    buffer = dataset.create_episode_buffer(episode_index=index)
    replay = {}
    with h5py.File(source / 'trajectory.h5', 'r') as tr, h5py.File(rgb / 'images.h5', 'r') as images:
        count = len(tr['action'])
        if not np.isclose(float(tr.attrs['dt']), 1/15, rtol=0, atol=1e-10):
            raise ValueError('Expected original 15 Hz action rate')
        original_origin = np.asarray(tr.attrs.get('environment_origin_world_m', [0.,0.,0.]), dtype=float)
        coordinate_frame = str(tr.attrs.get('coordinate_frame', 'world'))
        translation = np.zeros(3) if coordinate_frame == 'environment_local' else original_origin
        phase_names = tr['phase'].asstr()[:]
        settled_index = next((i for i,phase in enumerate(phase_names) if phase != 'settle'),0)
        buffer.update(size=count, task=[TASK]*count,
            timestamp=np.asarray(tr['time'], dtype=np.float32), frame_index=np.arange(count,dtype=np.int64),
            action=np.asarray(tr['action'],dtype=np.float32),
            scene_index=np.full(count,SCENES.index(spec['obstacle_id']),dtype=np.int64),
            phase_index=np.asarray([PHASES.index(p) for p in phase_names],dtype=np.int64))
        buffer.update(risk_arrays(source, count, index))
        if not np.isin(buffer['action'][:,-1], [0.,1.]).all():
            raise ValueError('Gripper targets must be binary')
        for source_prefix, target_prefix in (('obs','observation'),('next_obs','next_observation')):
            group = tr[source_prefix]
            buffer[target_prefix+'.state'] = np.asarray(group['q'],dtype=np.float32)
            buffer[target_prefix+'.joint_velocity'] = np.asarray(group['dq'],dtype=np.float32)
            tcp = np.column_stack([group['tcp'][:],group['tcp_quat_xyzw'][:]]).astype(np.float64)
            buffer[target_prefix+'.tcp_pose'] = canonical_pose(tcp,translation)
            for name in ('mustard','bowl'):
                buffer[target_prefix+'.'+name] = canonical_pose(group[name][:],translation).astype(np.float32)
            buffer[target_prefix+'.robot_body_pose'] = canonical_pose(
                group['robot_body_pose_w'][:],translation).reshape(count,126).astype(np.float32)
            buffer[target_prefix+'.robot_contact_forces'] = group['robot_contact_forces'][:].reshape(count,54).astype(np.float32)
            if 'hazard_pose' in group:
                hazard = canonical_pose(group['hazard_pose'][:],translation)
                hazard_source = 'measured dynamic root state'
            else:
                obstacle = spec['layout']['obstacle']
                angle = math.radians(obstacle.get('yaw_deg',0.))/2
                pose = np.r_[obstacle['position'],0.,0.,math.sin(angle),math.cos(angle)]
                hazard = np.tile(pose,(count,1))
                hazard_source = 'applied static scene root from episode layout'
            buffer[target_prefix+'.hazard_pose'] = hazard.astype(np.float32)
            mapped = {'q','dq','tcp','tcp_quat_xyzw','mustard','bowl','robot_body_pose_w',
                      'robot_contact_forces','hazard_pose'}
            for key in group:
                if key not in mapped:
                    array = group[key][:]
                    if key in ('kitchen_root_state','kitchen_link_pose','hazard_state'):
                        array = canonical_pose(array,translation)
                    replay[source_prefix+'.'+key] = array
        source_videos = {}
        for camera in CAMERAS:
            image_key = 'observation.images.' + camera
            group = images[camera]
            if len(group['jpeg']) != count:
                raise ValueError('RGB/action frame counts differ')
            directory = cache / camera
            directory.mkdir()
            paths = []
            for frame in range(count):
                path = directory / f'frame-{frame:06d}.jpg'
                path.write_bytes(group['jpeg'][frame].tobytes())
                paths.append(str(path))
            buffer[image_key] = paths
            poses = np.column_stack([group['position_world'][:],group['quaternion_world_ros_xyzw'][:]]).astype(np.float64)
            buffer['observation.camera_pose.'+camera] = canonical_pose(poses,translation)
            video = rgb / (camera + '.mp4')
            source_videos[image_key] = video if video.is_file() and not dataset.reencode_from_jpeg else None
    dataset.source_videos = source_videos
    dataset.source_jpegs = {key:buffer[key] for key in source_videos}
    settled_poses = {name:buffer['observation.'+name][settled_index,:7].tolist()
                     for name in ('mustard','bowl')}
    settled_poses['obstacle'] = buffer['observation.hazard_pose'][settled_index].tolist()
    return buffer, dict(spec=spec,verification=verification,result=result,
        origin=original_origin,source_coordinate_frame=coordinate_frame,
        hazard_pose_source=hazard_source,replay=replay,frames=count,
        settled_poses=settled_poses,settled_pose_action_index=settled_index)


def normalize_contact_points(value, origin):
    if isinstance(value, list):
        return [normalize_contact_points(item,origin) for item in value]
    if isinstance(value, dict):
        result = {}
        for key, child in value.items():
            if key == 'points_world_m':
                if child is None:
                    result['points_environment_m'] = None
                    continue
                points = np.asarray(child,dtype=float)
                result['points_environment_m'] = (points-origin).tolist() if points.size else []
            else:
                result[key] = normalize_contact_points(child,origin)
        return result
    return value


def save_episode_extras(output, row, index, info):
    source, rgb = Path(row['source_episode']), Path(row['rgb_directory'])
    relative = Path('extras/episodes') / f'episode-{index:06d}'
    folder = output / relative
    folder.mkdir(parents=True,exist_ok=True)
    write_json(folder/'randomization.json',dict(sampled=info['spec'],
        actual_settled_poses=info['settled_poses'],coordinate_frame='environment_local',
        settled_pose_action_index=info['settled_pose_action_index'],
        settled_pose_source='obs at the first action after initial settle; raw source report is retained in sources/result.json',
        appearance_applied=read_json(rgb/'appearance.json') if (rgb/'appearance.json').exists() else None,
        hazard_pose_source=info['hazard_pose_source']))
    write_json(folder/'validation.json',dict(**{k:info['verification'].get(k) for k in ('execution_success','data_success','audit_pass')},
        benchmark_passed=info['verification'].get('passed') is True,
        augmentation_eligible=info['verification'].get('augmentation_eligible') is True,
        benchmark_failure_kind=info['verification'].get('failure_kind'),
        source_reports_directory='sources'))
    reports = folder/'sources';reports.mkdir(exist_ok=True)
    for name in ('result.json','verification.json','validation.json','audit.json',
                 'clearances.json','render_state_metadata.json','collection_provenance.json'):
        if (source/name).exists():shutil.copyfile(source/name,reports/name)
    evidence = [p for p in source.glob('safety_*') if p.is_file()]
    evidence += [p for p in (source/'result_axes.json',source/'sim_raw_gt.json',source/'risk_report_index.json',source/'observer_action_audit.json',rgb/'safety_camera_provenance.jsonl') if p.is_file()]
    if evidence:
        import hashlib
        safety_folder=folder/'safety';safety_folder.mkdir()
        summaries={}
        for path in evidence:
            shutil.copyfile(path,safety_folder/path.name)
            h=hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):h.update(block)
            summaries[path.name]=h.hexdigest()
        write_json(safety_folder/'evidence_sha256.json',summaries)
        write_json(safety_folder/'label_semantics.json',dict(profile_index=index,category_order=['HS','PT','RS','IR'],unknown=-1,
            risk_valid='Known level, possibly a lower bound; completeness is a separate mask',
            observation_alignment='Last causal risk slot at action start; no future physical evidence',
            policy_input=False))
    shutil.copyfile(rgb/'cameras.json',folder/'cameras.json')
    write_json(folder/'grasp_contacts.json',dict(coordinate_frame='environment_local',
        source_environment_origin_world_m=info['origin'],
        records=normalize_contact_points(read_json(source/'grasp_contacts.json'),info['origin'])))
    shutil.copyfile(source/'clearances.json',folder/'clearances.json')
    np.savez_compressed(folder/'replay_states.npz',**info['replay'])
    with np.load(source/'physics_states.npz',allow_pickle=False) as physics:
        count = len(physics['action_index'])
        columns, metadata = {'episode_index':pa.array(np.full(count,index,dtype=np.int64))}, {}
        for key in physics.files:
            array = physics[key]
            if array.ndim and len(array)==count:
                values = array.reshape(count,int(np.prod(array.shape[1:]))).tolist() if array.ndim>1 else array.tolist()
                columns[key] = pa.array(values)
            else:
                metadata[key] = array.tolist()
        metadata.update(coordinate_frame='environment_local',
            original_environment_origin_world_m=info['origin'],
            source_pose_convention=metadata.get('pose_convention'),
            pose_convention='environment_local',
            coverage='Recorded execution checks; initial 30 settle actions are excluded',
            recorded_checks=count,reported_physics_clearance_checks=info['result'].get('physics_clearance_checks'))
        pq.write_table(pa.table(columns),folder/'safety_steps.parquet',compression='zstd')
        write_json(folder/'safety_metadata.json',metadata)
    return dict(episode_index=index,scene_index=SCENES.index(info['spec']['obstacle_id']),
        obstacle_id=info['spec']['obstacle_id'],category=info['spec']['obstacle_id'].split('_',1)[0],
        frames=info['frames'],master_seed=info['spec'].get('master_seed'),source_attempt=info['spec'].get('attempt'),
        source_episode_name=source.name,source_batch=source.parent.parent.parent.name,
        geometry_seed=info['spec']['seeds']['geometry'],appearance_seed=info['spec']['seeds']['appearance'],
        planner_seed=info['spec']['seeds']['planner'],
        benchmark_passed=info['verification'].get('passed') is True,
        **{k:info['verification'].get(k) for k in ('execution_success','data_success','audit_pass')},
        augmentation_eligible=info['verification'].get('augmentation_eligible') is True,
        supplementary_directory=str(relative),randomization_path=str(relative/'randomization.json'),
        validation_path=str(relative/'validation.json'),contacts_path=str(relative/'grasp_contacts.json'),
        safety_steps_path=str(relative/'safety_steps.parquet'),replay_states_path=str(relative/'replay_states.npz'))


def validate_export(output, rows, repo_id, validation_videos):
    reader = LeRobotDataset(repo_id=repo_id,root=output,video_backend='pyav')
    if reader.meta.info['codebase_version'] != 'v3.0' or reader.num_episodes != len(rows):
        raise ValueError('Native v3 reader episode count differs')
    files = defaultdict(int)
    checks = []
    for index,row in enumerate(rows):
        metadata = reader.meta.episodes[index]
        begin,end = int(metadata['dataset_from_index']),int(metadata['dataset_to_index'])
        source,rgb = Path(row['source_episode']),Path(row['rgb_directory'])
        with h5py.File(source/'trajectory.h5','r') as tr:
            count = len(tr['action'])
            if end-begin != count:raise ValueError('Native episode bounds differ from actions')
            table = reader.hf_dataset.with_format('arrow')[begin:end]
            values = {name:np.asarray(table[name].to_pylist()) for name in table.column_names}
            for target,key in (('action','action'),('observation.state','obs/q'),
                               ('next_observation.state','next_obs/q')):
                np.testing.assert_array_equal(np.asarray(values[target]),tr[key][:])
            for key,expected in risk_arrays(source,count,index).items():
                np.testing.assert_array_equal(values[key],expected)
            np.testing.assert_array_equal(values['frame_index'],np.arange(count))
            np.testing.assert_array_equal(values['scene_index'],np.full(count,SCENES.index(row['obstacle_id'])))
            np.testing.assert_allclose(values['timestamp'],np.asarray(tr['time'],dtype=np.float32),rtol=0,atol=0)
            np.testing.assert_array_equal(values['phase_index'],[PHASES.index(p) for p in tr['phase'].asstr()[:]])
            origin = np.asarray(tr.attrs.get('environment_origin_world_m',[0.,0.,0.]),dtype=float)
            shift = np.zeros(3) if tr.attrs.get('coordinate_frame') == 'environment_local' else origin
            spec = read_json(source/'episode_spec.json')
            for group_name,prefix in (('obs','observation'),('next_obs','next_observation')):
                group = tr[group_name]
                np.testing.assert_array_equal(values[prefix+'.joint_velocity'],group['dq'][:])
                tcp = canonical_pose(np.column_stack([group['tcp'][:],group['tcp_quat_xyzw'][:]]),shift)
                np.testing.assert_array_equal(values[prefix+'.tcp_pose'],tcp)
                for name in ('mustard','bowl'):
                    np.testing.assert_array_equal(values[prefix+'.'+name],canonical_pose(group[name][:],shift))
                np.testing.assert_array_equal(values[prefix+'.robot_body_pose'],
                    canonical_pose(group['robot_body_pose_w'][:],shift).reshape(count,126))
                np.testing.assert_array_equal(values[prefix+'.robot_contact_forces'],
                    group['robot_contact_forces'][:].reshape(count,54))
                if 'hazard_pose' in group:
                    hazard = canonical_pose(group['hazard_pose'][:],shift)
                else:
                    obstacle = spec['layout']['obstacle'];angle=math.radians(obstacle.get('yaw_deg',0.))/2
                    hazard = np.tile(np.r_[obstacle['position'],0.,0.,math.sin(angle),math.cos(angle)],(count,1)).astype(np.float32)
                np.testing.assert_array_equal(values[prefix+'.hazard_pose'],hazard)
            with h5py.File(rgb/'images.h5','r') as images:
                for camera in CAMERAS:
                    group=images[camera]
                    pose=canonical_pose(np.column_stack([group['position_world'][:],group['quaternion_world_ros_xyzw'][:]]).astype(np.float64),shift)
                    np.testing.assert_array_equal(values['observation.camera_pose.'+camera],pose)
            for name in values:
                if name.startswith('observation.') and 'camera_pose.' not in name:
                    following='next_'+name
                    if following in values:
                        np.testing.assert_allclose(values[following][:-1],values[name][1:],rtol=0,atol=1e-6)
            for frame_index in sorted({0,count//2,count-1}):
                sample = reader[begin+frame_index]
                for camera in CAMERAS:
                    key = 'observation.images.'+camera
                    reference = validation_videos.get(index,{}).get(key,rgb/(camera+'.mp4'))
                    expected = decode_video_frames(reference,[frame_index/15],1e-4,backend='pyav')[0]
                    if not torch.equal(sample[key],expected):
                        raise ValueError(f'Video offset/frame mismatch: episode {index}, {camera}, {frame_index}')
        for camera in CAMERAS:
            path = output/reader.meta.get_video_file_path(index,'observation.images.'+camera)
            files[path] += count
        checks.append(dict(episode_index=index,frames=count,core_arrays_equal=True,
                           all_state_and_camera_features_equal=True,state_chain_continuous=True,
                           risk_masks_sources_and_result_axes_equal=True,
                           input_video_stream_first_middle_last_equal=True))
    decoded = {str(path.relative_to(output)):decode_video(path,count,(1280,720),1/15)
               for path,count in files.items()}
    report = dict(passed=True,format='LeRobot Dataset v3.0',episodes=len(rows),
        total_frames=reader.num_frames,cameras=list(CAMERAS),episodes_checked=checks,
        fully_decoded_video_files=decoded,reader='lerobot==0.4.3',video_backend='pyav')
    write_json(output/'validation_report.json',report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection',type=Path,default=ROOT/'datasets/augmentation/human_default20_n4_20261019/successful_episodes.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repo-id',default=REPO_ID)
    parser.add_argument('--selection-scope',choices=['benchmark','task_success','all_recorded'],default='benchmark',
        help='Manifest selection; all_recorded includes validated RGB failures as negative episodes')
    parser.add_argument('--limit',type=int)
    parser.add_argument('--wait-for-complete',action='store_true')
    parser.add_argument('--reencode-from-jpeg',action='store_true',
                        help='Encode original JPEG streams even if reusable original-view MP4s exist')
    args = parser.parse_args()
    torch.set_num_threads(1)
    if importlib.metadata.version('lerobot') != '0.4.3':
        raise RuntimeError('Use the pinned v0.4.3 export environment for this encoding hook')
    while True:
        selection = load_selection(args.selection,args.selection_scope)
        rows = selection['episodes'][:args.limit] if args.limit else selection['episodes']
        ready = (not args.wait_for_complete or selection['complete']) and bool(rows)
        ready = ready and all((Path(row['rgb_directory'])/'images.h5').is_file() for row in rows)
        if ready:break
        if not args.wait_for_complete:raise ValueError('Source RGB data is not ready')
        time.sleep(15)
    output = args.output.resolve()
    output.parent.mkdir(parents=True,exist_ok=True)
    features,names = make_features(Path(rows[0]['source_episode']))
    dataset = PreencodedDataset.create(args.repo_id,15,features,root=output,
        robot_type='franka_panda_robotiq_2f85_isaac_arena',video_backend='pyav',vcodec='h264')
    dataset.meta.update_chunk_settings(data_files_size_in_mb=64,video_files_size_in_mb=64)
    dataset.allow_failed = args.selection_scope == 'all_recorded'
    dataset.reencode_from_jpeg = args.reencode_from_jpeg
    dataset.validation_videos = {}
    extras = output/'extras';extras.mkdir(exist_ok=True)
    write_json(extras/'scene_catalog.json',dict(
        scenes=[dict(scene_index=i,obstacle_id=name,name=SCENE_LABELS[i],category=name.split('_',1)[0])
                for i,name in enumerate(SCENES)],
        present_scene_indices=sorted({SCENES.index(row['obstacle_id']) for row in rows}),
        current_collection_target=selection['target_count']))
    write_json(extras/'field_conventions.json',dict(**names,coordinate_frame='environment_local',
        quaternion_order='xyzw',position_unit='m',joint_angle_unit='rad',force_unit='N',fps=15,
        action='7 absolute joint targets in radians followed by 0=open or 1=close',
        rgb_alignment='RGB[t] and observation[t] precede action[t]',
        camera_pose='camera-to-environment, ROS optical axes x right, y down, z forward',
        phase_indices=dict(enumerate(PHASES)),sampling_angle_deg_suffix_unit='degree',
        raw_diagnostic_reports='sources/ JSON retains original provenance and coordinate declarations; use canonical core states, contacts and safety table for training'))
    shutil.copyfile(Path(rows[0]['rgb_directory'])/'cameras.json',extras/'cameras.json')
    provenance = []
    try:
        for index,row in enumerate(rows):
            with tempfile.TemporaryDirectory(prefix='.jpeg_',dir=output) as cache:
                buffer,info = episode_buffer(dataset,row,index,Path(cache))
                record = save_episode_extras(output,row,index,info)
                dataset.save_episode(episode_data=buffer,parallel_encoding=False)
                provenance.append(record)
                pq.write_table(pa.Table.from_pylist(provenance),extras/'episodes.parquet')
                print(json.dumps(dict(exported=index+1,total=len(rows),frames=info['frames'])),flush=True)
    finally:
        dataset.finalize()
    with (extras/'collection_summary.csv').open('w',newline='') as stream:
        fields=('obstacle_id','target_count','success_count','attempts','failed_count','deficit',
                'exported_episode_count','benchmark_passed_count')
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for identifier in sorted({row['obstacle_id'] for row in provenance}):
            group=[row for row in provenance if row['obstacle_id']==identifier]
            source_successes=sum(row['obstacle_id']==identifier for row in selection['episodes'])
            failures=sum((row.get('obstacle_id') or Path(row['source_episode']).parent.name)==identifier
                         for row in selection.get('failed_attempts',[]))
            target=selection.get('targets_by_id',{}).get(identifier,
                selection['target_count'] if len({r['obstacle_id'] for r in selection['episodes']})==1 else source_successes)
            writer.writerow(dict(obstacle_id=identifier,target_count=target,success_count=source_successes,
                                 attempts=source_successes+failures,failed_count=failures,
                                 deficit=max(0,target-source_successes),exported_episode_count=len(group),
                                 benchmark_passed_count=sum(row['benchmark_passed'] for row in group)))
    write_json(extras/'source_collection_summary.json',
        {key:value for key,value in selection.items() if key != 'episodes'})
    report = validate_export(output,rows,args.repo_id,dataset.validation_videos)
    shutil.rmtree(output/'.validation_videos',ignore_errors=True)
    write_json(output/'export_report.json',dict(complete=report['passed'],episodes=len(rows),
        selection_target=selection['target_count'],source_collection_complete=selection['complete'],
        subset_export=len(rows)<len(selection['episodes']),
        source_scope='successfully executed and recorded safety episodes; benchmark qualification retained separately',
        video_method='Native v0.4.3 writer; reuse original-view H.264 when available, otherwise encode original RGB JPEG streams once',
        versions={key:importlib.metadata.version(key) for key in ('lerobot','torch','torchvision','datasets','av','numpy','pyarrow','h5py')}))
    print(json.dumps(dict(complete=True,episodes=len(rows),output=str(output))),flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
