#!/usr/bin/env python3
"""Render measured failure states with a stopped timeline; never execute a task.

Each image is explicitly diagnostic state replay, not a recorded execution frame.
No environment, actuator command, simulation reset or physics step is created.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import traceback

import numpy as np
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
KIND = 'physical_failure_diagnostic_state_replay'
SCOPE = 'Diagnostic images reconstructed from measured native rigid-body poses; zero new physical episodes/steps. Not execution-video frames, new task success, or safety acceptance.'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local(value):
    value = str(value)
    if value.startswith('/workspace/'):
        value = value[len('/workspace/'):]
    p = (ROOT / value).resolve() if not Path(value).is_absolute() else Path(value).resolve()
    p.relative_to(ROOT)
    return p


def relative(path):
    return str(Path(path).resolve().relative_to(ROOT))


def bound(value, digest):
    path = local(value)
    if not isinstance(digest, str) or len(digest) != 64 or not path.is_file() or sha(path) != digest:
        raise ValueError('Source hash mismatch: ' + str(value))
    return path


def atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    os.replace(temporary, path)


def select_historical_observation(result, expected_camera_config_sha256):
    """Skip only bound same-scene no-camera episodes, never foreign cameras."""
    for prior in reversed(result['prior_physical_attempts']):
        prior_path = bound(prior['result_path'], prior['result_sha256'])
        observation = json.loads(prior_path.read_text())
        if observation.get('scene_id') != result['scene_id']:
            raise ValueError('Diagnostic observation reference belongs to another scene')
        if not (observation.get('overview') or observation.get('camera_override', {}).get('applied')):
            continue
        if observation.get('config_sha256') != expected_camera_config_sha256:
            raise ValueError('Diagnostic observation reference belongs to another scene/config (historical camera)')
        return observation, dict(observation_reference_path=relative(prior_path),
            observation_reference_sha256=prior['result_sha256'])
    raise ValueError('No recorded camera in the physical history')


def load_record(manifest_path, row):
    """Hash-bound actual state, including background bodies, never nominal q FK."""
    raw_path = row['execution_result_path']
    result_path = local(raw_path) if not raw_path.startswith('executions/') else manifest_path.parent/raw_path
    result_path.resolve().relative_to(manifest_path.parent.resolve())
    if sha(result_path) != row['execution_result_sha256']:
        raise ValueError('Execution result hash mismatch')
    result = json.loads(result_path.read_text())
    if (result.get('kind') not in ('physical_execution','physical_failure_retry_execution') or result.get('executed') is not True
            or result.get('execution_status') not in ('failed', 'error')
            or any(result.get(k) != row.get(k) for k in ('scene_id', 'task_id', 'variant_id', 'execution_status'))):
        raise ValueError('Only actual executed retry failures can be replayed')
    config = bound(result['config_path'], result['config_sha256'])
    planning = bound(result['planning_result_path'], result['planning_result_sha256'])
    planned = json.loads(planning.read_text())
    for path, digest in planned['composition']['source_paths_sha256'].items():
        bound(path, digest)
    evidence = result['physics_evidence']
    states_path = bound(evidence['q_path'], evidence['q_sha256'])
    metadata_path = bound(evidence['metadata_path'], evidence['metadata_sha256'])
    if states_path.parent != result_path.parent or metadata_path.parent != result_path.parent:
        raise ValueError('State evidence belongs to another physical episode')
    metadata = json.loads(metadata_path.read_text())
    with np.load(states_path, allow_pickle=False) as archive:
        arrays = {key:archive[key] for key in archive.files}
    validate_states(arrays, metadata)
    if evidence.get('samples') != len(arrays['physics_step']):
        raise ValueError('Execution result/state sample count mismatch')
    overlay = result['physical_overlay_binding']
    overlay_path = bound(overlay['path'], overlay['sha256_start'])
    if overlay.get('sha256_end') != overlay['sha256_start'] or overlay.get('unchanged') is not True:
        raise ValueError('Physical background overlay was not unchanged')
    binding = dict(execution_result_path=relative(result_path), execution_result_sha256=sha(result_path),
        config_path=relative(config), config_sha256=sha(config), planning_result_path=relative(planning),
        planning_result_sha256=sha(planning), physics_states_path=relative(states_path), physics_states_sha256=sha(states_path),
        physics_metadata_path=relative(metadata_path), physics_metadata_sha256=sha(metadata_path),
        physical_overlay_path=relative(overlay_path), physical_overlay_sha256=sha(overlay_path),
        source_paths_sha256=planned['composition']['source_paths_sha256'])
    revision = None
    if 'scene_revision' in result:
        if result.get('retained_scene_revision') is True or row.get('retained_scene_revision') is True:
            if any(result.get(key) is not value or row.get(key) is not value for key,value in
                    dict(retained_scene_revision=True,scene_settings_changed_this_round=False,scene_settings_changed_once=True).items()):
                raise ValueError('Actual result and manifest retained-layout scope differ')
        from scripts.extensions.replicator.scene_revision_review import validate_scene_revision
        revision_document=json.loads(bound(result['scene_revision']['path'],result['scene_revision']['sha256']).read_text())
        if revision_document.get('retry_round') == 8:
            from scripts.extensions.replicator.scene_revision_review_round8 import validate_scene_revision
        if result.get('retained_scene_revision') is True:
            from scripts.extensions.replicator.retained_scene_revision_review import validate_retained_scene_revision
            retained_parent = result['source_previous_retry_manifest']
            parent_round = json.loads(bound(retained_parent['path'], retained_parent['sha256']).read_text()).get('retry_round')
            if parent_round == 9:
                from scripts.extensions.replicator.retained_scene_revision_review_round10 import validate_retained_scene_revision
            elif parent_round == 10:
                from scripts.extensions.replicator.retained_scene_revision_review_round11 import validate_retained_scene_revision
            elif parent_round == 11:
                from scripts.extensions.replicator.retained_scene_revision_review_round12 import validate_retained_scene_revision
            elif parent_round == 12:
                from scripts.extensions.replicator.retained_scene_revision_review_round13 import validate_retained_scene_revision
            if result.get('scene_settings_changed_this_round') is not False:
                raise ValueError('Retained scene cannot claim a new layout change')
            revision = validate_retained_scene_revision(ROOT,result['scene_revision'],
                result['source_previous_retry_manifest'],expected_scene_id=result['scene_id'],current=row)
        else:
            revision = validate_scene_revision(ROOT,result['scene_revision'],expected_scene_id=result['scene_id'],current=row)
        if result['scene_revision'] != row['scene_revision']: raise ValueError('Result scene revision differs')
        if (dict(path=result['config_path'],sha256=result['config_sha256']) != revision['document']['new_config']
                or planned.get('scene_revision') != result['scene_revision']
                or planned.get('composition') != revision['document']['new_composition']
                or planned.get('config_path') != result['config_path']
                or planned.get('config_sha256') != result['config_sha256']):
            raise ValueError('Rendered actual state/planning sources differ from the new scene revision')
        binding.update(scene_revision=result['scene_revision'],layout_changed=True,
            camera_reference_scope='historical_camera_pose_only')
        if revision_document.get('retry_round') == 8:
            binding['historical_camera_config']=revision['historical_camera_config']
        if result.get('retained_scene_revision') is True:
            binding.update(retained_scene_revision=True,scene_settings_changed_this_round=False)
    observation = result
    if result['kind'] == 'physical_failure_retry_execution':
        # Later rounds also contain earlier no-render retries. Walk the frozen
        # history to the newest actual camera observation, never invent one.
        expected_camera_config = revision.get('historical_camera_config',revision['document']['old_config'])['sha256'] if revision else result['config_sha256']
        observation, camera_binding = select_historical_observation(result, expected_camera_config)
        binding.update(camera_binding)
    # No-render retry has no camera. Reuse the last hash-bound camera observation
    # (including the clear U-banana camera retake), never claim retry camera RGB.
    result = dict(result, diagnostic_observation_reference=observation)
    return result, json.loads(config.read_text()), arrays, metadata, binding


def validate_states(arrays, metadata):
    required = ('physics_step', 'phase', 'action_index', 'source_trajectory_index', 'q', 'robot_body_pose_w',
                'task_object_states', 'all_rigid_body_pose_w')
    if any(key not in arrays for key in required):
        raise ValueError('Missing measured state arrays')
    n = len(arrays['physics_step']); paths = metadata.get('all_rigid_body_paths', [])
    if (n < 1 or len(set(paths)) != len(paths) or not paths
            or any(not p.startswith('/World/envs/env_0/') for p in paths)
            or not str(metadata.get('quaternion_convention', '')).startswith('xyzw')
            or metadata.get('samples') != n):
        raise ValueError('Invalid native state metadata')
    if any(len(arrays[k]) != n for k in required) or not np.all(np.diff(arrays['physics_step']) == 1):
        raise ValueError('State samples are not aligned/continuous')
    shapes = {'all_rigid_body_pose_w':(n,len(paths),7),
              'robot_body_pose_w':(n,len(metadata['robot_body_names']),7),
              'q':(n,len(metadata['joint_names'])),
              'task_object_states':(n,len(metadata['object_names']),13)}
    for key, shape in shapes.items():
        if arrays[key].shape != shape or not np.isfinite(arrays[key]).all():
            raise ValueError('Invalid measured state: '+key)
    for key in ('all_rigid_body_pose_w', 'robot_body_pose_w', 'task_object_states'):
        if not np.allclose(np.linalg.norm(arrays[key][...,3:7], axis=-1),1.,atol=2e-3):
            raise ValueError('Invalid measured xyzw quaternion: '+key)


def select_frames(arrays, maximum=6):
    """Phase endpoints expose grasp, transport and release; never interpolate."""
    phases = arrays['phase'].astype(str); selected = []
    for phase in ('close','lift','transfer','lower','release'):
        ids = np.flatnonzero(phases == phase)
        if len(ids): selected.append(int(ids[-1]))
    selected.append(len(phases)-1)
    selected = sorted(set(selected))
    # Early errors may omit normal phases. Include observed coverage only.
    for i in np.linspace(0,len(phases)-1,min(maximum,len(phases))).astype(int):
        if len(selected) >= maximum: break
        selected = sorted(set([*selected,int(i)]))
    return [dict(state_index=i, physics_step=int(arrays['physics_step'][i]), phase=str(phases[i]),
                 action_index=int(arrays['action_index'][i]), source_trajectory_index=int(arrays['source_trajectory_index'][i]))
            for i in selected[:maximum]]


def load_frame_hints(path, manifest):
    """Load optional analyst labels; scene result/state bindings are checked below.

    A top-level manifest SHA is an analysis snapshot, which may predate completion.
    Per-scene result and recorded-state SHA are always required and authoritative.
    """
    if path is None: return {}, None
    path=local(path);data=json.loads(path.read_text())
    if not isinstance(data,dict) or not isinstance(data.get('entries'),list):
        raise ValueError('Frame hints require an entries array')
    known={row['scene_id']:row for row in manifest['entries']};entries={}
    for row in data['entries']:
        if (not isinstance(row,dict) or not isinstance(row.get('scene_id'),str)
                or row['scene_id'] not in known or row['scene_id'] in entries):
            raise ValueError('Unknown or duplicate frame-hint scene')
        actual=known[row['scene_id']]
        if actual.get('execution_status') not in ('failed','error') or actual.get('executed') is not True:
            raise ValueError('Frame hints must bind an actual executed failure')
        entries[row['scene_id']]=row
    provenance=dict(path=relative(path),sha256=sha(path))
    snapshot=data.get('source_manifest',data.get('manifest_snapshot'))
    if snapshot is not None:provenance['analysis_manifest_snapshot']=snapshot
    return entries,provenance


def select_frames_with_hints(arrays, binding, hint=None):
    """Keep default six plus exact measured event states; never interpolate/truncate."""
    defaults=select_frames(arrays)
    if hint is None:return defaults
    canonical='frames' in hint
    if canonical and 'suggested_six_diagnostic_frames' in hint:
        raise ValueError('Ambiguous frame-hint arrays')
    frames=hint.get('frames' if canonical else 'suggested_six_diagnostic_frames')
    if not isinstance(frames,list) or len(frames)>32:
        raise ValueError('Invalid frame-hint array')
    if canonical:
        references=[('execution_result',dict(path=hint.get('execution_result_path'),sha256=hint.get('execution_result_sha256'))),
                    ('physics_states',dict(path=hint.get('physics_states_path'),sha256=hint.get('physics_states_sha256')))]
        if 'physics_metadata_path' in hint or 'physics_metadata_sha256' in hint:
            references.append(('physics_metadata',dict(path=hint.get('physics_metadata_path'),sha256=hint.get('physics_metadata_sha256'))))
    else:
        references=[('execution_result',hint.get('result')),('physics_states',hint.get('physics_states'))]
        if 'physics_metadata' in hint:references.append(('physics_metadata',hint['physics_metadata']))
    for name,ref in references:
        if (not isinstance(ref,dict) or not isinstance(ref.get('path'),str)
                or ref.get('sha256')!=binding[name+'_sha256']
                or local(ref['path'])!=local(binding[name+'_path'])):
            raise ValueError('Frame-hint '+name+' path/SHA mismatch')
    selected={frame['state_index']:dict(frame,selection='default_phase_endpoint',events=[]) for frame in defaults}
    for hint_frame in frames:
        if not isinstance(hint_frame,dict):raise ValueError('Frame hint must be an object')
        index=hint_frame.get('state_index')
        if type(index) is not int or not 0<=index<len(arrays['phase']):
            raise ValueError('Frame-hint state_index outside actual recording')
        actual=dict(state_index=index,physics_step=int(arrays['physics_step'][index]),phase=str(arrays['phase'][index]),
            action_index=int(arrays['action_index'][index]),source_trajectory_index=int(arrays['source_trajectory_index'][index]))
        for key in ('physics_step','phase','action_index','source_trajectory_index'):
            if key in hint_frame and hint_frame[key]!=actual[key]:
                raise ValueError('Frame-hint '+key+' differs from actual recorded state')
        label=hint_frame.get('event',hint_frame.get('label'))
        reason=hint_frame.get('reason','')
        if (not isinstance(label,str) or not label.strip() or len(label)>200
                or not isinstance(reason,str) or len(reason)>2000):
            raise ValueError('Frame-hint event label/reason invalid')
        row=selected.setdefault(index,dict(actual,selection='measured_event_hint',events=[]))
        event=dict(event=label,reason=reason)
        if event not in row['events']:row['events'].append(event)
    if len(selected)>10:raise ValueError('Default and event frames exceed the total limit of 10')
    return [selected[index] for index in sorted(selected)]


def mapped_body_path(path, object_names):
    prefix='/World/envs/env_0/'
    if not path.startswith(prefix): raise ValueError('Unknown native body namespace')
    tail=path[len(prefix):]
    if tail.startswith('kitchen/'): return '/World/kitchen/Asset/'+tail[len('kitchen/'):]
    if tail == 'Robot': return '/World/robot/Asset/panda_link0'
    if tail.startswith('Robot/'): return '/World/robot/Asset/'+tail[len('Robot/'):]
    if tail == 'hazard' or tail.startswith('hazard/'):
        return '/World/hazard/Asset'+tail[len('hazard'):]
    for name in object_names:
        if tail == name or tail.startswith(name+'/'):
            return '/World/objects/'+name+'/Asset'+tail[len(name):]
    raise ValueError('Unmapped measured rigid body: '+path)


def prepare_stage(config_path, stage_path, result, metadata, binding):
    from pxr import Gf, Usd, UsdGeom
    from kitchen_safety.extensions.replicator.scene_build_runtime import compose
    compose(config_path, stage_path)
    stage=Usd.Stage.Open(str(stage_path))
    kitchen=stage.GetPrimAtPath('/World/kitchen/Asset')
    kitchen.GetReferences().ClearReferences()
    kitchen.GetReferences().AddReference(str(local(binding['physical_overlay_path'])))
    if stage.GetCompositionErrors(): raise ValueError('Diagnostic USD has composition errors')
    mapping=[];cache=UsdGeom.XformCache()
    for path in metadata['all_rigid_body_paths']:
        target=mapped_body_path(path, metadata['object_names']);prim=stage.GetPrimAtPath(target)
        if not prim: raise ValueError('Measured rigid-body prim missing: '+target)
        ancestors=[];p=prim
        while p and not p.IsPseudoRoot():
            ancestors.append(p.GetPath());p=p.GetParent()
        for p in reversed(ancestors):
            candidate=stage.GetPrimAtPath(p)
            if candidate.IsInstance(): candidate.SetInstanceable(False)
        prim=stage.GetPrimAtPath(target)
        scale=Gf.Transform(cache.GetLocalToWorldTransform(prim)).GetScale()
        mapping.append(dict(source_path=path,target_path=target,world_scale=list(scale)))
    if len({x['target_path'] for x in mapping}) != len(mapping):
        raise ValueError('Multiple actual bodies map to one diagnostic prim')
    observation=result.get('diagnostic_observation_reference',result)
    overview=observation.get('overview',{});camera_spec=observation.get('camera_override',{}).get('applied',{})
    eye=camera_spec.get('eye',overview.get('position_world'));target=camera_spec.get('target',overview.get('target_world'))
    if eye is None or target is None: raise ValueError('Recorded observation camera is missing')
    camera=UsdGeom.Camera(stage.GetPrimAtPath('/World/overview_camera'))
    camera.ClearXformOpOrder();camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
    camera.GetFocalLengthAttr().Set(float(camera_spec.get('focal_length_mm',18.)))
    camera.GetHorizontalApertureAttr().Set(float(camera_spec.get('horizontal_aperture_mm',36.)))
    camera.GetVerticalApertureAttr().Set(float(camera_spec.get('vertical_aperture_mm',20.25)))
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*camera_spec.get('clipping_range_m',[.05,1000.])))
    stage.GetRootLayer().Save()
    return stage,mapping


def apply_state(stage, mapping, poses):
    from pxr import Gf, UsdGeom
    for item,pose in zip(mapping,poses):
        transform=Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]),Gf.Vec3d(*map(float,pose[3:6]))))
        transform.SetTranslateOnly(Gf.Vec3d(*map(float,pose[:3])))
        transform=Gf.Matrix4d().SetScale(Gf.Vec3d(*item['world_scale']))*transform
        xform=UsdGeom.Xformable(stage.GetPrimAtPath(item['target_path']))
        xform.ClearXformOpOrder();xform.SetResetXformStack(True)
        xform.AddTransformOp(opSuffix='measuredDiagnostic').Set(transform)
    cache=UsdGeom.XformCache()
    for item,pose in zip(mapping,poses):
        transform=Gf.Transform(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(item['target_path'])))
        quat=transform.GetRotation().GetQuat();observed=np.array([*quat.GetImaginary(),quat.GetReal()])
        if (not np.allclose(transform.GetTranslation(),pose[:3],atol=1e-6)
                or abs(float(np.dot(observed,pose[3:7]))) < 1-1e-5
                or not np.allclose(transform.GetScale(),item['world_scale'],atol=1e-6)):
            raise ValueError('Diagnostic world pose readback differs: '+item['source_path'])


def detail_camera(stage, arrays, state_index, high=False):
    """Observation-only tracking view of the measured first task object."""
    from pxr import Gf, UsdGeom
    close=np.flatnonzero(arrays['phase'].astype(str)=='close')
    anchor=int(close[-1]) if len(close) else 0
    origin=arrays['task_object_states'][anchor,0,:3].astype(float)
    outward=arrays['robot_body_pose_w'][anchor,0,:3].astype(float)-origin
    outward[2]=0
    length=float(np.linalg.norm(outward))
    if length<.05:raise ValueError('Cannot choose diagnostic oblique direction')
    outward/=length;side=np.array([-outward[1],outward[0],0.])
    target=arrays['task_object_states'][state_index,0,:3].astype(float)+[0,0,.06]
    eye=(target+.25*outward-.20*side+[0,0,.85] if high
         else target+.7*outward+.55*side+[0,0,.45])
    camera=UsdGeom.Camera(stage.GetPrimAtPath('/World/overview_camera'))
    camera.ClearXformOpOrder()
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
    camera.GetFocalLengthAttr().Set(24.)
    return dict(mode='measured_object_tracking_high_oblique' if high else 'measured_object_tracking_oblique',eye=eye.tolist(),target=target.tolist(),focal_length_mm=24.,
                note='Diagnostic camera follows actual task object 0; source geometry and measured body poses unchanged.')


def main():
    # Parse without importing Kit so CPU preflight genuinely cannot launch it.
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scene',default='')
    parser.add_argument('--detail-view',action='store_true',help='Supplementary oblique camera following measured task object; no physical changes')
    parser.add_argument('--top-view',action='store_true',help='Supplementary steep view into a sink; observation only, no physical changes')
    parser.add_argument('--exposure-ev',type=float,default=0.,help='Diagnostic RTX camera exposure compensation, in [-6, 0]')
    parser.add_argument('--frame-hints',type=Path,help='Hash-bound measured event indices; defaults retained, at most 10 total images per scene')
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--width',type=int,default=960);parser.add_argument('--height',type=int,default=640)
    args=parser.parse_args();args.manifest=local(args.manifest);args.output=local(args.output)
    if args.detail_view and args.top_view:raise ValueError('Select only one diagnostic camera override')
    if not -6.<=args.exposure_ev<=0.:raise ValueError('Invalid diagnostic exposure compensation')
    manifest=json.loads(args.manifest.read_text());manifest_sha=sha(args.manifest)
    if manifest.get('stage') != 'complete': raise ValueError('Wait for the physical retry manifest to complete')
    selected=set(args.scene.split(',')) if args.scene else set()
    rows=[r for r in manifest['entries'] if r.get('execution_status') in ('failed','error') and r.get('executed') is True and (not selected or r['scene_id'] in selected)]
    if selected-set(r['scene_id'] for r in rows):raise ValueError('Selected scene is not an executed failure')
    if len({r['scene_id'] for r in rows}) != len(rows): raise ValueError('Duplicate physical scene')
    hints,hints_binding=load_frame_hints(args.frame_hints,manifest)
    # Validate every supplied hint before creating output or launching Kit, even
    # when --scene selects a subset. No stale/foreign hint is silently ignored.
    by_id={row['scene_id']:row for row in manifest['entries']}
    for sid,hint in hints.items():
        _,_,hint_arrays,_,hint_source=load_record(args.manifest,by_id[sid])
        select_frames_with_hints(hint_arrays,hint_source,hint)
    checks=[]
    if args.dry_run:
        for row in rows:
            result,layout,arrays,metadata,binding=load_record(args.manifest,row)
            with tempfile.TemporaryDirectory(prefix='robolaw-diagnostic-') as temp:
                stage,mapping=prepare_stage(local(binding['config_path']),Path(temp)/'scene.usda',result,metadata,binding)
                frames=select_frames_with_hints(arrays,binding,hints.get(row['scene_id']))
                for frame in frames:
                    apply_state(stage,mapping,arrays['all_rigid_body_pose_w'][frame['state_index']])
                    if args.detail_view or args.top_view:detail_camera(stage,arrays,frame['state_index'],high=args.top_view)
            checks.append(dict(scene_id=row['scene_id'],frames=len(frames),selected_frames=frames,mapped_bodies=len(mapping)))
        print(json.dumps(dict(passed=True,kind=KIND,scene_count=len(checks),entries=checks,frame_hints=hints_binding,physics_launched=False,rendering_launched=False),ensure_ascii=False));return 0
    # GPU rendering is only requested by the coordinator invoking this mode.
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    kit_args=get_isaaclab_arena_cli_parser().parse_args([]);kit_args.enable_cameras=True
    os.environ.setdefault('ISAACLAB_ARENA_FORCE_EXIT_ON_COMPLETE','1')
    progress_path=args.output/'progress.json'
    progress=dict(schema_version=1,kind=KIND,scope=SCOPE,stage='rendering',total=len(rows),manifest_path=relative(args.manifest),manifest_sha256=manifest_sha,frame_hints=hints_binding,entries=[])
    observation_policy=dict(detail_view=args.detail_view,exposure_ev=args.exposure_ev)
    if args.top_view:observation_policy['top_view']=True
    progress['observation_policy']=observation_policy
    if progress_path.exists():
        old=json.loads(progress_path.read_text())
        if not args.resume or old.get('manifest_sha256') != manifest_sha or old.get('frame_hints') != hints_binding: raise ValueError('Existing diagnostic batch/source/hints differs')
        if old.get('observation_policy',dict(detail_view=False,exposure_ev=0.))!=observation_policy:raise ValueError('Diagnostic observation policy changed')
        progress=old
    atomic(progress_path,progress)
    with SimulationAppContext(kit_args) as context:
        from PIL import Image,ImageDraw
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        timeline=omni.timeline.get_timeline_interface();usd=omni.usd.get_context();app=context.app_launcher.app
        import carb
        from scripts.extensions.replicator.visual_scene_previews import EXPOSURE_KEYS, exposure_override
        settings=carb.settings.get_settings()
        for row in rows:
            sid=row['scene_id'];destination=args.output/'scenes'/sid;report_path=destination/'diagnostic.json'
            previous=next((x for x in progress['entries'] if x['scene_id']==sid),None)
            if previous and previous.get('status')=='ready' and args.resume:
                report=json.loads(report_path.read_text())
                if sha(report_path)!=previous['report_sha256'] or any(sha(local(f['image_path']))!=f['image_sha256'] for f in report['frames']):raise ValueError('Existing diagnostic image/report changed')
                continue
            if destination.exists():raise FileExistsError('Preserve prior render attempt: '+str(destination))
            destination.mkdir(parents=True);started=time.monotonic();annotator=product=None;original_exposure=None
            entry=dict(scene_id=sid,status='rendering',execution_result_sha256=row['execution_result_sha256'])
            progress['entries']=[x for x in progress['entries'] if x['scene_id']!=sid]+[entry];atomic(progress_path,progress)
            try:
                result,layout,arrays,metadata,binding=load_record(args.manifest,row)
                stage,mapping=prepare_stage(local(binding['config_path']),destination/'diagnostic_scene.usda',result,metadata,binding)
                timeline.stop();timeline.set_current_time(0.);usd.open_stage(str(destination/'diagnostic_scene.usda'))
                for _ in range(240):
                    app.update()
                    if usd.get_stage_loading_status()[2]==0:break
                else:raise TimeoutError('Diagnostic stage loading timeout')
                stage=usd.get_stage();product=rep.create.render_product('/World/overview_camera',(args.width,args.height))
                annotator=rep.AnnotatorRegistry.get_annotator('rgb',device='cpu');annotator.attach(product)
                # Opening a USD stage can restore its RTX settings. Apply camera
                # exposure after stage loading, then verify at every captured frame.
                original_exposure={key:settings.get(key) for key in EXPOSURE_KEYS}
                applied_exposure=exposure_override(original_exposure,args.exposure_ev) if args.exposure_ev else original_exposure
                for key,value in applied_exposure.items():settings.set(key,value)
                report=dict(schema_version=1,kind=KIND,scope=SCOPE,scene_id=sid,task_id=row['task_id'],variant_id=row['variant_id'],
                    diagnostic_only=True,physical_episode_started=False,physics_steps_requested=0,new_task_success_claimed=False,
                    actual_execution_status=result['execution_status'],timeline_playing=False,binding=binding,frame_hints=hints_binding,
                    mapped_bodies=mapping,frames=[],appearance_note='Recorded camera pose/optics and source geometry; diagnostic lighting may differ from execution camera.')
                report.update(observation_policy=observation_policy,renderer_settings=dict(original=original_exposure,applied=applied_exposure))
                if args.detail_view or args.top_view:report['appearance_note']='Supplementary camera follows measured object from an oblique or steep upper direction; original geometry and measured poses retained. Camera differs from execution.'
                selected_frames=select_frames_with_hints(arrays,binding,hints.get(sid))
                for ordinal,frame in enumerate(selected_frames):
                    apply_state(stage,mapping,arrays['all_rigid_body_pose_w'][frame['state_index']])
                    if args.detail_view or args.top_view:frame['diagnostic_camera']=detail_camera(stage,arrays,frame['state_index'],high=args.top_view)
                    for _ in range(24):app.update()
                    exposure_readback={key:settings.get(key) for key in EXPOSURE_KEYS}
                    if exposure_readback!=applied_exposure:raise RuntimeError('Diagnostic exposure settings changed after stage load')
                    frame['renderer_settings_readback']=exposure_readback
                    rgb=np.asarray(annotator.get_data())
                    if timeline.is_playing() or timeline.get_current_time()!=0:raise RuntimeError('Diagnostic timeline unexpectedly advanced')
                    if rgb.shape!=(args.height,args.width,4) or float(rgb[:,:,:3].std())<=2:raise ValueError('Missing/blank diagnostic RGB')
                    image=Image.fromarray(rgb[:,:,:3].astype(np.uint8));draw=ImageDraw.Draw(image)
                    draw.rectangle((0,0,args.width,28),fill=(0,0,0));draw.text((8,8),f'DIAGNOSTIC STATE REPLAY | {frame["phase"]} | measured step {frame["physics_step"]} | no new execution'+(' | TOP VIEW' if args.top_view else ' | OBLIQUE' if args.detail_view else ''),fill=(255,255,255))
                    if frame.get('events'):
                        # Event labels remain separate from actual NPZ phase.
                        label='; '.join(event['event'] for event in frame['events']).encode('ascii','replace').decode()[:145]
                        draw.rectangle((0,28,args.width,48),fill=(0,0,0));draw.text((8,33),'EVENT: '+label,fill=(255,220,120))
                    path=destination/f'frame_{ordinal:02d}.jpg';image.save(path,quality=92)
                    report['frames'].append(dict(frame,image_path=relative(path),image_sha256=sha(path)))
                if sha(args.manifest)!=manifest_sha:raise ValueError('Physical manifest changed while rendering')
                if hints_binding and sha(local(hints_binding['path']))!=hints_binding['sha256']:raise ValueError('Frame hints changed while rendering')
                load_record(args.manifest,row)
                report.update(status='ready',elapsed_seconds=time.monotonic()-started)
                atomic(report_path,report);entry.update(status='ready',report_path=relative(report_path),report_sha256=sha(report_path),frame_count=len(report['frames']))
                print('PHYSICAL_FAILURE_DIAGNOSTIC_READY',sid,len(report['frames']),flush=True)
            except Exception as error:
                report=dict(kind=KIND,scope=SCOPE,scene_id=sid,status='error',diagnostic_only=True,physics_steps_requested=0,error=repr(error),traceback=traceback.format_exc())
                atomic(report_path,report);entry.update(status='error',report_path=relative(report_path),report_sha256=sha(report_path),error=repr(error))
                print('PHYSICAL_FAILURE_DIAGNOSTIC_FAILED',sid,repr(error),flush=True)
            finally:
                if original_exposure is not None:
                    for key,value in original_exposure.items():settings.set(key,value)
                if annotator is not None:annotator.detach()
                if product is not None:product.destroy()
                timeline.stop();atomic(progress_path,progress)
        progress['stage']='complete';progress['counts']={status:sum(e['status']==status for e in progress['entries']) for status in ('ready','error')};atomic(progress_path,progress)
    return 0

if __name__=='__main__':raise SystemExit(main())
