"""Small source-binding regressions without Kit, CUDA or physical evidence."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import render_physical_retry_failures as replay

ROOT=Path(__file__).resolve().parents[3]

def load(path,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

MODULES=(load('kitchen_safety/extensions/replicator/scene_revision.py','runtime_revision_gate'),
         load('scripts/extensions/replicator/scene_revision_review.py','publication_revision_gate'))

class SceneRevisionBindings(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.old=dict(task_id='task',kitchen_usd='asset.usd',robot_position=[0,0,0],
            objects={'bowl':{'position':[1,2,3]}},obstacle=dict(position=[0.,0.,0.],yaw_deg=0.,usd_path='asset.usd',static=True))
        self.new=deepcopy(self.old);self.new['obstacle']['position']=[.05,.12,0.];self.new['obstacle']['yaw_deg']=-10.
        self.asset=self.text('asset.usd','unchanged source asset')
        self.oldconfig=self.json('old.json',self.old);self.newconfig=self.json('new.json',self.new)
        self.oldstage=self.text('old.usda','old composed stage');self.newstage=self.text('new.usda','new composed stage')
        self.assets={'asset.usd':self.asset['sha256']}
        plan=self.json('original_plan.json',dict(composition=dict(source_paths_sha256=self.assets)))
        self.row=dict(scene_id='scene',task_id='task',variant_id='human',result_path=plan['path'],result_sha256=plan['sha256'],
            config_path=self.oldconfig['path'],config_sha256=self.oldconfig['sha256'],source_scene_path=self.oldstage['path'],source_scene_sha256=self.oldstage['sha256'],
            execution_status='planning_failed',executed=False,attempts=0)
        self.previous=dict(retry_round=6,stage='complete',entries=[self.row])
        self.prior=self.json('previous.json',self.previous)
        self.doc=dict(kind='hazard_pose_revision',retry_round=7,scene_id='scene',task_id='task',variant_id='human',
            prior_manifest=self.prior,prior_row_source={k:self.row[k] for k in MODULES[0].SOURCE_KEYS},
            old_config=self.oldconfig,new_config=self.newconfig,old_source_scene=self.oldstage,new_source_scene=self.newstage,
            changed_fields=['obstacle.position','obstacle.yaw_deg'],new_composition=dict(
                source_paths_sha256=self.assets,layout_sha256=self.newconfig['sha256'],scene_sha256=self.newstage['sha256']))

    def text(self,name,text):
        path=self.root/name;path.write_text(text)
        return dict(path=name,sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def json(self,name,value):return self.text(name,json.dumps(value))

    def check(self,doc=None,current=None):
        document=self.doc if doc is None else doc
        binding=self.json('revision.json',document)
        for module in MODULES:
            with self.subTest(module=module.__name__):
                module.validate_scene_revision(self.root,binding,expected_scene_id='scene',current=current)

    def reject(self,doc=None,current=None):
        document=self.doc if doc is None else doc;binding=self.json('revision.json',document)
        for module in MODULES:
            with self.subTest(module=module.__name__):
                with self.assertRaises(ValueError):module.validate_scene_revision(self.root,binding,expected_scene_id='scene',current=current)

    def test_new_human_pose_preserves_old_config_and_assets(self):
        self.check()
        binding=self.json('revision.json',self.doc)
        current=dict(self.row,scene_revision=binding,layout_changed=True,config_path=self.newconfig['path'],config_sha256=self.newconfig['sha256'],
            source_scene_path=self.newstage['path'],source_scene_sha256=self.newstage['sha256'])
        self.check(current=current)
        self.assertEqual(json.loads((self.root/'old.json').read_text()),self.old)

    def test_task_object_edit_cannot_hide_behind_new_matching_hashes(self):
        changed=deepcopy(self.new);changed['objects']['bowl']['position'][0]+=.01
        doc=deepcopy(self.doc);doc['new_config']=self.json('new.json',changed)
        doc['new_composition']['layout_sha256']=doc['new_config']['sha256']
        self.reject(doc)

    def test_old_manifest_failure_cannot_gain_an_execution(self):
        previous=deepcopy(self.previous);previous['entries'][0].update(executed=True,attempts=1)
        doc=deepcopy(self.doc);doc['prior_manifest']=self.json('previous.json',previous)
        self.reject(doc)

    def test_old_source_snapshot_cannot_be_rewritten_to_new_layout(self):
        doc=deepcopy(self.doc);doc['prior_row_source']['config_sha256']=self.newconfig['sha256']
        self.reject(doc)

    def test_same_asset_name_with_modified_bytes_is_rejected(self):
        self.text('asset.usd','modified collider')
        self.reject()

    def test_new_asset_bindings_cannot_replace_original_asset_set(self):
        replacement=self.text('replacement.usd','different collider')
        doc=deepcopy(self.doc);doc['new_composition']['source_paths_sha256']={'replacement.usd':replacement['sha256']}
        self.reject(doc)

    def test_current_row_cannot_use_the_old_source_as_new(self):
        binding=self.json('revision.json',self.doc)
        current=dict(self.row,scene_revision=binding,layout_changed=True,config_path=self.newconfig['path'],config_sha256=self.newconfig['sha256'])
        self.reject(current=current)

    def test_changed_fields_must_exactly_match_the_pose_delta(self):
        for fields in (['obstacle.position'],['obstacle.position','obstacle.yaw_deg','obstacle.position']):
            doc=deepcopy(self.doc);doc['changed_fields']=fields;self.reject(doc)

    def test_nonfinite_hazard_pose_is_rejected_even_if_hashes_match(self):
        changed=deepcopy(self.new);changed['obstacle']['yaw_deg']=float('nan')
        doc=deepcopy(self.doc);doc['new_config']=self.json('new.json',changed)
        doc['new_composition']['layout_sha256']=doc['new_config']['sha256'];self.reject(doc)

class SceneRevisionRendererBindings(unittest.TestCase):
    def setUp(self):
        self.fixture=SceneRevisionBindings('test_new_human_pose_preserves_old_config_and_assets')
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        f=self.fixture;self.root=f.root;self.revision=f.json('revision.json',f.doc)
        self.folder=self.root/'executions'/'scene';self.folder.mkdir(parents=True)
        poses=np.zeros((2,1,7));poses[...,6]=1
        obj=np.zeros((2,1,13));obj[...,6]=1
        np.savez(self.folder/'physics_states.npz',physics_step=np.array([0,1]),phase=np.array(['reset','final_settle']),
            action_index=np.array([0,1]),source_trajectory_index=np.array([0,1]),q=np.zeros((2,1)),
            robot_body_pose_w=poses,task_object_states=obj,all_rigid_body_pose_w=poses)
        states=self.bind(self.folder/'physics_states.npz')
        metadata=f.json('executions/scene/metadata.json',dict(samples=2,all_rigid_body_paths=['/World/envs/env_0/Robot'],
            robot_body_names=['link'],joint_names=['joint'],object_names=['bowl'],quaternion_convention='xyzw'))
        overlay=f.text('overlay.usda','unchanged physical background')
        self.camera=dict(scene_id='scene',config_sha256=f.oldconfig['sha256'],overview=dict(eye=[1,2,3],target=[0,0,0]))
        camera=f.json('camera.json',self.camera)
        self.planned=dict(config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],
            composition=deepcopy(f.doc['new_composition']),scene_revision=self.revision)
        candidate=f.json('candidate.json',self.planned)
        self.result=dict(kind='physical_failure_retry_execution',scene_id='scene',task_id='task',variant_id='human',
            execution_status='failed',executed=True,config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],
            planning_result_path=candidate['path'],planning_result_sha256=candidate['sha256'],scene_revision=self.revision,
            physics_evidence=dict(q_path=states['path'],q_sha256=states['sha256'],metadata_path=metadata['path'],metadata_sha256=metadata['sha256'],samples=2),
            physical_overlay_binding=dict(path=overlay['path'],sha256_start=overlay['sha256'],sha256_end=overlay['sha256'],unchanged=True),
            prior_physical_attempts=[dict(result_path=camera['path'],result_sha256=camera['sha256'])])

    def bind(self,path):return dict(path=str(path.relative_to(self.root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def load(self,result=None,planned=None):
        f=self.fixture;r=deepcopy(self.result if result is None else result)
        if planned is not None:
            candidate=f.json('candidate.json',planned);r.update(planning_result_path=candidate['path'],planning_result_sha256=candidate['sha256'])
        binding=f.json('executions/scene/result.json',r)
        row=dict(f.row,config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],
            source_scene_path=f.newstage['path'],source_scene_sha256=f.newstage['sha256'],scene_revision=self.revision,
            layout_changed=True,executed=True,execution_status='failed',execution_result_path=binding['path'],execution_result_sha256=binding['sha256'])
        with patch.object(replay,'ROOT',self.root):return replay.load_record(self.root/'manifest.json',row)

    def test_new_layout_is_rendered_with_only_old_camera_pose_reused(self):
        result,layout,arrays,metadata,binding=self.load()
        self.assertEqual(layout,self.fixture.new)
        self.assertEqual(result['diagnostic_observation_reference']['config_sha256'],self.fixture.oldconfig['sha256'])
        self.assertEqual(binding['camera_reference_scope'],'historical_camera_pose_only')
        self.assertEqual(binding['config_sha256'],self.fixture.newconfig['sha256'])

    def test_new_revision_cannot_render_old_config_from_actual_result(self):
        changed=deepcopy(self.result);changed.update(config_path=self.fixture.oldconfig['path'],config_sha256=self.fixture.oldconfig['sha256'])
        with self.assertRaisesRegex(ValueError,'sources differ from the new scene revision'):self.load(result=changed)

    def test_candidate_revision_composition_and_config_must_match(self):
        for field in ('scene_revision','composition','config_path','config_sha256'):
            changed=deepcopy(self.planned)
            if field=='scene_revision':changed[field]=dict(path='wrong.json',sha256='0'*64)
            elif field=='composition':changed[field]['scene_sha256']=self.fixture.oldstage['sha256']
            elif field=='config_path':changed[field]=self.fixture.oldconfig['path']
            else:changed[field]=self.fixture.oldconfig['sha256']
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'sources differ from the new scene revision'):
                self.load(planned=changed)

    def test_camera_reference_must_still_belong_to_original_config(self):
        changed=deepcopy(self.camera);changed['config_sha256']=self.fixture.newconfig['sha256']
        binding=self.fixture.json('camera.json',changed)
        result=deepcopy(self.result);result['prior_physical_attempts'][0]['result_sha256']=binding['sha256']
        with self.assertRaisesRegex(ValueError,'another scene/config'):self.load(result=result)

if __name__=='__main__':unittest.main()
