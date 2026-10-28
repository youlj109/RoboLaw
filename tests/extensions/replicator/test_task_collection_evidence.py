"""CPU schema tests; does not claim a runtime helper has been exercised in Kit."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.task_collection_evidence import (
    ROOT, TASK_CODE, BANANA_RECIPE, STAND_SOURCE_KEY, STAND_RUNTIME_PATH,
    TaskRunProvenance, audit_provenance, audit_predicate_evidence, sensor_identity)
from scripts.extensions.replicator.task_camera_audit import usd_authored_quaternion
from scipy.spatial.transform import Rotation


def write_predicates(directory):
    task = "kitchen_bench_replicator_g_shape_banana_bowl"
    state = np.zeros((5, 2, 13));state[:, :, 6] = 1.
    state[:, 0, :3] = [[1.,0.,.01],[1.,0.,.01],[1.,0.,.07],[0.,0.,.01],[0.,0.,.01]]
    force = np.zeros((5, 1, 1, 3));force[3:, 0, 0, 2] = .1
    data = dict(physics_step=np.arange(5), phase=np.array(["reset","initial_settle","lift","release","final_settle"]),
        action_index=np.arange(-1,4), task_object_states=state,destination_contact_force=force)
    trajectory = dict(after_physics_step=np.arange(1,5))
    metadata = dict(task_id=task, task_object_names=["banana","bowl"], physics_dt_s=.1,action_dt_s=.1,reset_physics_step=0)
    root = "/World/envs/env_0/"
    sensor = dict(contact_sensor_name="contact_sensor_banana", sensed_native_paths=[root+"banana"],
        resolved_filter_prim_paths=[root+"bowl"], actual_pick_up_path=root+"banana",actual_destination_path=root+"bowl",
        native_filter_count=1,force_matrix_shape=[1,1,1,3])
    info = dict(schema_version=1,task_id=task,pick_up_object="banana",destination_location="bowl",
        actual_spawned_local_bounds={"banana":dict(min=[-.01]*3,max=[.01]*3),"bowl":dict(min=[-.1,-.1,0.],max=[.1,.1,.2])},
        sensor_identity=sensor,sensor_identity_at_end=deepcopy(sensor), thresholds=dict(force_threshold=.1,velocity_threshold=.003,
            support_cone_half_angle_rad=np.pi/4,placement_consecutive_steps=1), physics_dt_s=.1,action_dt_s=.1,reset_physics_step=0,
        input_sha256={})
    rows = dict(physics_step=np.arange(5),phase=data['phase'],action_index=data['action_index'],object_pose=state[:,0,:7],
        destination_pose=state[:,1,:7],object_centroid_world=state[:,0,:3],object_linear_velocity=state[:,0,7:10],
        destination_contact_force=force[:,0,0],has_original_rest_pose=np.array([False,True,True,True,True]),
        original_rest_position=np.tile([1.,0.,.01],(5,1)),original_above_rest=np.array([False,False,True,False,False]),
        original_object_on_destination=np.array([False,False,False,True,True]),terminal_success=np.array([False,False,False,True,True]),
        terminal_failure=np.zeros(5,dtype=bool))
    for kind,ids in [("physics",slice(None)),("actions",slice(1,None))]:
        path=directory/("task_predicate_"+kind+".npz");np.savez(path,**{k:v[ids] for k,v in rows.items()})
        info['input_sha256'][path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    (directory/'task_predicate_metadata.json').write_text(json.dumps(info))
    result=dict(original_task_success=True,final_original_destination_predicate=True,
        terminal=dict(first_success=dict(physics_step=3,terms=dict(success=True))))
    return metadata,data,trajectory,result


def test_runtime_predicate_evidence_recomputes_complete_sequence(tmp_path):
    args=write_predicates(tmp_path)
    r=audit_predicate_evidence(tmp_path,*args)
    assert r['passed'] and r['first_rest_physics_step']==1 and r['first_success_physics_step']==3


def test_predicate_cannot_change_sensor_or_original_threshold(tmp_path):
    args=write_predicates(tmp_path);p=tmp_path/'task_predicate_metadata.json';r=json.loads(p.read_text())
    r['thresholds']['velocity_threshold']=.01;p.write_text(json.dumps(r))
    with pytest.raises(ValueError,match='thresholds'):audit_predicate_evidence(tmp_path,*args)
    args=write_predicates(tmp_path);r=json.loads(p.read_text());r['sensor_identity']['resolved_filter_prim_paths']=['/World/envs/env_0/floor'];p.write_text(json.dumps(r))
    with pytest.raises(ValueError,match='sensor identity'):audit_predicate_evidence(tmp_path,*args)


def test_predicate_rejects_missing_action_or_changed_bound_file(tmp_path):
    args=write_predicates(tmp_path);p=tmp_path/'task_predicate_actions.npz'
    with np.load(p) as r:values={k:r[k][:-1] for k in r.files}
    np.savez(p,**values)
    with pytest.raises(ValueError,match='hash'):audit_predicate_evidence(tmp_path,*args)


def test_predicate_cannot_claim_false_raw_support(tmp_path):
    args=write_predicates(tmp_path);args[1]['destination_contact_force'][-1,0,0,2]=.099
    with pytest.raises(ValueError,match='actual physics'):audit_predicate_evidence(tmp_path,*args)


def test_new_provenance_captures_real_dependencies_before_initialization():
    assert all((ROOT/name).is_file() for name in TASK_CODE)
    recorder=TaskRunProvenance(ROOT/'configs/extensions/replicator/g_shape_banana_bowl_source_layout.json')
    assert set(TASK_CODE)<=set(recorder.entries)
    assert BANANA_RECIPE in recorder.entries
    assert BANANA_RECIPE not in TASK_CODE
    bowl=TaskRunProvenance(ROOT/'configs/extensions/replicator/g_shape_bowl_sink_source_layout.json')
    assert BANANA_RECIPE not in bowl.entries
    assert recorder.check()['passed']
    with pytest.raises(ValueError,match='scene was not bound'):recorder.finish_task(ROOT,0)


def test_provenance_requires_full_archived_execution_sources(tmp_path):
    p=tmp_path/'p.json';p.write_text(json.dumps(dict(passed=True,task_id='t',source_layout={},checked_physics_step=2,entries={})))
    with pytest.raises(ValueError,match='snapshots missing'):
        audit_provenance(p,dict(task_id='t',source_layout={}),2)


def test_nonunit_source_quaternion_uses_usd_op_not_euclidean_normalization():
    q=[.399,-.195,-.393,.805]
    authored=usd_authored_quaternion(q)
    assert abs(np.linalg.norm(authored)-1.)<1e-14
    assert abs(authored[3]-.805)<1e-14
    discrepancy=np.rad2deg((Rotation.from_quat(q).inv()*Rotation.from_quat(authored)).magnitude())
    assert .023 < discrepancy < .024
    # A source unit quaternion retains exactly the same rotation.
    q=Rotation.from_euler('xyz',[.2,.4,-.3]).as_quat()
    assert (Rotation.from_quat(q).inv()*Rotation.from_quat(usd_authored_quaternion(q))).magnitude()<1e-14


def test_camera_audit_retains_original_threshold_and_resolution(tmp_path):
    import h5py
    from scripts.extensions.replicator.task_camera_audit import authored_camera_audit
    from scripts.extensions.replicator.task_audit_sidecars import source_camera_config
    from kitchen_safety.dataset import RGB_ALIGNMENT
    source=source_camera_config();configs={};bodies=np.zeros((2,2,7));bodies[:,:,6]=1.
    meta=dict(robot_body_names=['panda_link0','base_link'],action_dt_s=.1)
    data=dict(physics_step=np.arange(2),robot_body_pose_w=bodies)
    traj=dict(before_physics_step=np.array([0,1]),camera_physics_steps=np.array([[0,0],[1,1]]))
    with h5py.File(tmp_path/'images.h5','w') as f:
        f.attrs.update(frames=2,dt=.1,alignment=RGB_ALIGNMENT)
        for name,config in source.items():
            spawn=config['spawn'];w,h=config['width'],config['height'];offset=config['offset']
            configs[name]=dict(prim_path=config['prim_path'],width=w,height=h,
                intrinsics=[[w*spawn['focal_length']/spawn['horizontal_aperture'],0,w/2],[0,h*spawn['focal_length']/spawn['vertical_aperture'],h/2],[0,0,1]],
                offset=dict(position=offset['pos'],quaternion_xyzw=offset['rot'],convention=offset['convention']))
            g=f.create_group(name);g.create_dataset('jpeg',data=np.zeros((2,1),dtype=np.uint8))
            g.create_dataset('position_world',data=np.tile(offset['pos'],(2,1)))
            q=(Rotation.from_quat(usd_authored_quaternion(offset['rot']))*Rotation.from_quat([1,0,0,0])).as_quat()
            g.create_dataset('quaternion_world_ros_xyzw',data=np.tile(q,(2,1)))
    p=tmp_path/'cameras.json';p.write_text(json.dumps(configs))
    assert authored_camera_audit(tmp_path,meta,data,traj)['passed']
    configs['external_camera_2']['width']=640;p.write_text(json.dumps(configs))
    assert not authored_camera_audit(tmp_path,meta,data,traj)['passed']
    configs['external_camera_2']['width']=1280;p.write_text(json.dumps(configs))
    with h5py.File(tmp_path/'images.h5','r+') as f:
        row=f['external_camera_2/quaternion_world_ros_xyzw'][1]
        f['external_camera_2/quaternion_world_ros_xyzw'][1]=(Rotation.from_quat(row)*Rotation.from_euler('x',.021,degrees=True)).as_quat()
    r=authored_camera_audit(tmp_path,meta,data,traj)
    assert not r['passed'] and r['cameras']['external_camera_2']['rotation_max_error_deg']>.02


def provenance_fixture(tmp_path, monkeypatch):
    import kitchen_safety.extensions.replicator.task_collection_evidence as module
    monkeypatch.setattr(module,'TASK_CODE',('code.py',))
    digest=lambda text:hashlib.sha256(text.encode()).hexdigest()
    layout=dict(task_id='t');layout_text=json.dumps(layout)
    profile=dict(task_id='t',source_layout_sha256=digest(layout_text),source_kitchen_sha256='k',source_object_sha256=dict(banana='b'))
    rows={}
    def add(key,kind,value,source=False):
        h=digest(value) if source else value
        rows[key]=dict(path=key,kind=kind,initial_sha256=h,checked_sha256=h,unchanged=True,matches_declared_source=True)
        if source:rows[key]['source_text']=value
    add('code.py','execution_source','x=1\n',True)
    add('layout.json','source_layout_input',layout_text,True)
    add('profile.json','controller_input',json.dumps(profile),True)
    add('source.yaml','source_task_yaml','env_name: t\n',True)
    add('kitchen.usd','source_kitchen','k')
    add('object.usd','task_asset_root','b')
    add('stand.usd','composed_robot_on_stand','s')
    add('overlay.usd','composed_kitchen_overlay','o')
    background=dict(source_sha256='k',recipe_sha256=rows['code.py']['initial_sha256'],overlay_sha256='o',recipe_source_path='code.py')
    meta=dict(task_id='t',source_layout=layout,background_physics=background,task_object_names=['banana'],source_hashes=dict(banana='b'),
        source_relations=dict(source_yaml='source.yaml',source_yaml_sha256=rows['source.yaml']['initial_sha256'],
            assets_sha256={'kitchen.usd':'k','object.usd':'b','stand.usd':'s'}))
    report=dict(passed=True,task_id='t',source_layout=layout,checked_physics_step=5,background_physics=background,entries=rows)
    path=tmp_path/'provenance.json';path.write_text(json.dumps(report))
    return path,meta,report


def test_provenance_actual_inputs_cannot_be_omitted_under_green_flag(tmp_path,monkeypatch):
    p,meta,report=provenance_fixture(tmp_path,monkeypatch)
    assert audit_provenance(p,meta,5)['passed']
    for key in ('object.usd','stand.usd','source.yaml','layout.json','profile.json'):
        altered=deepcopy(report);del altered['entries'][key];p.write_text(json.dumps(altered))
        with pytest.raises(ValueError):audit_provenance(p,meta,5)


def test_provenance_profile_must_bind_actual_source_layout_and_payload(tmp_path,monkeypatch):
    p,meta,report=provenance_fixture(tmp_path,monkeypatch)
    row=report['entries']['profile.json'];profile=json.loads(row['source_text']);profile['source_object_sha256']['banana']='other'
    row['source_text']=json.dumps(profile);h=hashlib.sha256(row['source_text'].encode()).hexdigest();row['initial_sha256']=row['checked_sha256']=h
    p.write_text(json.dumps(report))
    with pytest.raises(ValueError,match='object inventory'):audit_provenance(p,meta,5)


def test_known_old_bowl_stand_alias_requires_exact_path_role_hash_and_uniqueness(tmp_path,monkeypatch):
    import kitchen_safety.extensions.replicator.task_collection_evidence as module
    monkeypatch.setattr(module,'BOWL_TASK','t')
    p,meta,report=provenance_fixture(tmp_path,monkeypatch)
    meta['source_relations']['assets_sha256'].pop('stand.usd')
    meta['source_relations']['assets_sha256'][STAND_SOURCE_KEY]='s'
    row=report['entries'].pop('stand.usd');row['path']=STAND_RUNTIME_PATH
    report['entries'][STAND_RUNTIME_PATH]=row
    p.write_text(json.dumps(report))
    result=audit_provenance(p,meta,5)
    assert result['passed'] and result['accepted_input_aliases'][0]['recorded_path']==STAND_RUNTIME_PATH
    for changed in ('key','path','hash','kind','duplicate'):
        other=deepcopy(report);entry=other['entries'][STAND_RUNTIME_PATH]
        if changed=='key':other['entries']['/different'+STAND_RUNTIME_PATH]=other['entries'].pop(STAND_RUNTIME_PATH)
        elif changed=='path':entry['path']='/home/another/.cache/'+STAND_RUNTIME_PATH.split('/.cache/')[1]
        elif changed=='hash':entry['initial_sha256']=entry['checked_sha256']='wrong'
        elif changed=='kind':entry['kind']='task_asset_root'
        else:other['entries']['duplicate.usd']={**entry,'path':'duplicate.usd'}
        p.write_text(json.dumps(other))
        with pytest.raises(ValueError,match='Required original input'):audit_provenance(p,meta,5)
    p.write_text(json.dumps(report));wrong_meta=deepcopy(meta)
    wrong_meta['source_relations']['assets_sha256']['runtime/home/other/'+Path(STAND_SOURCE_KEY).name]='s'
    del wrong_meta['source_relations']['assets_sha256'][STAND_SOURCE_KEY]
    with pytest.raises(ValueError,match='Required original input'):audit_provenance(p,wrong_meta,5)
    monkeypatch.setattr(module,'BOWL_TASK','other_task')
    with pytest.raises(ValueError,match='Required original input'):audit_provenance(p,meta,5)


def test_recipe_and_inherited_profile_require_exact_bound_inputs(tmp_path,monkeypatch):
    p,meta,report=provenance_fixture(tmp_path,monkeypatch)
    entries=report['entries'];text='base = 1\n';digest=hashlib.sha256(text.encode()).hexdigest()
    entries['base.py']=dict(path='base.py',kind='execution_source',source_text=text,initial_sha256=digest,
        checked_sha256=digest,unchanged=True,matches_declared_source=True)
    entries['base.usda']=dict(path='base.usda',kind='inherited_kitchen_overlay',initial_sha256='inherited',
        checked_sha256='inherited',unchanged=True,matches_declared_source=True)
    profile=meta['background_physics']
    profile.update(base_recipe_source_path='base.py',base_recipe_sha256=digest,base_overlay_path='base.usda',
        base_overlay_sha256='inherited',base_profile=dict(source_sha256='k',recipe_sha256=digest,overlay_sha256='inherited'))
    report['background_physics']=deepcopy(profile);p.write_text(json.dumps(report))
    assert audit_provenance(p,meta,5)['passed']
    for key in ('base.py','base.usda'):
        changed=deepcopy(report);del changed['entries'][key];p.write_text(json.dumps(changed))
        with pytest.raises(ValueError,match='Inherited physical input'):audit_provenance(p,meta,5)
    changed=deepcopy(report);changed['entries']['base.usda']['kind']='composed_kitchen_overlay';p.write_text(json.dumps(changed))
    with pytest.raises(ValueError,match='Inherited physical input'):audit_provenance(p,meta,5)
    changed=deepcopy(report);changed['background_physics']['recipe_source_path']='wrong.py';p.write_text(json.dumps(changed))
    with pytest.raises(ValueError,match='Full physical profile'):audit_provenance(p,meta,5)


def test_bind_captures_each_declared_source_asset_and_actual_recipe_before_use(tmp_path):
    from types import SimpleNamespace as N
    recorder=TaskRunProvenance.__new__(TaskRunProvenance)
    recorder.root=tmp_path;recorder.entries={};recorder.layout={'task_id':'t'};recorder.task_id='t';recorder.scene_bound=False
    for name,text in {'leaf.py':'leaf=1\n','base.py':'base=1\n','source.usda':'source','base.usda':'base',
            'leaf.usda':'leaf','task.yaml':'task','declared_stand.usd':'stand','actual_stand.usd':'stand'}.items():
        (tmp_path/name).write_text(text)
    digest=lambda name:hashlib.sha256((tmp_path/name).read_bytes()).hexdigest()
    recorder.capture('leaf.py','execution_source',snapshot=True)
    recorder.capture('base.py','execution_source',snapshot=True)
    profile=dict(source=str(tmp_path/'source.usda'),source_sha256=digest('source.usda'),
        recipe_source_path=str(tmp_path/'leaf.py'),recipe_sha256=digest('leaf.py'),overlay_sha256=digest('leaf.usda'),
        base_recipe_source_path=str(tmp_path/'base.py'),base_recipe_sha256=digest('base.py'),
        base_overlay_path=str(tmp_path/'base.usda'),base_overlay_sha256=digest('base.usda'),
        base_profile=dict(source_sha256=digest('source.usda'),recipe_sha256=digest('base.py'),overlay_sha256=digest('base.usda')))
    definition=N(resolved_task_layout=recorder.layout,background_physics_provenance=profile,
        scene=N(assets={'kitchen':N(usd_path=str(tmp_path/'leaf.usda'))}),
        embodiment=N(scene_config=N(robot=N(spawn=N(usd_path=str(tmp_path/'actual_stand.usd'))))),
        source_relation_audit=dict(source_yaml=str(tmp_path/'task.yaml'),source_yaml_sha256=digest('task.yaml'),
            assets_sha256={'declared_stand.usd':digest('declared_stand.usd')}))
    recorder.bind_environment(definition)
    assert recorder.scene_bound and recorder.check()['passed']
    assert recorder.entries['leaf.py']['expected_sha256']==digest('leaf.py')
    assert recorder.entries['base.py']['expected_sha256']==digest('base.py')
    assert recorder.entries['base.usda']['kind']=='inherited_kitchen_overlay'
    assert recorder.entries['declared_stand.usd']['expected_sha256']==digest('declared_stand.usd')
    assert recorder.entries['actual_stand.usd']['kind']=='composed_robot_on_stand'
    del recorder.entries['leaf.py']
    with pytest.raises(ValueError,match='before initialization'):recorder.bind_environment(definition)
