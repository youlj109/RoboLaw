"""Two revision generations retain root inputs and the root camera reference."""
from copy import deepcopy
import unittest
from kitchen_safety.extensions.replicator import scene_revision_round8 as runtime
from scripts.extensions.replicator import scene_revision_review_round8 as review
from tests.extensions.replicator import test_scene_revision_bindings as prior_tests

MODULES=(runtime,review)

def add_round8(f):
    """Bind r8 to a real JSON r7 revision, without mocking either validator."""
    parent_ref=f.json('revision7.json',f.doc)
    r7row=dict(f.row,config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],
        source_scene_path=f.newstage['path'],source_scene_sha256=f.newstage['sha256'],scene_revision=parent_ref,layout_changed=True,
        execution_status='planning_failed',executed=False,attempts=0,physical_attempt_number=4,
        prior_physical_attempts=[dict(physical_attempt_number=i) for i in (1,2,3)])
    prior=f.json('manifest7.json',dict(retry_round=7,stage='complete',entries=[r7row]))
    new=deepcopy(f.new);new['obstacle']['position'][0]+=.12;new['obstacle']['yaw_deg']=0.
    config=f.json('config8.json',new);stage=f.text('scene8.usda','second new human pose')
    document=dict(kind='hazard_pose_revision',retry_round=8,scene_id='scene',task_id='task',variant_id='human',
        prior_manifest=prior,previous_revision=parent_ref,prior_row_source={k:r7row[k] for k in runtime.SOURCE_KEYS},
        old_config=f.newconfig,new_config=config,old_source_scene=f.newstage,new_source_scene=stage,
        changed_fields=['obstacle.position','obstacle.yaw_deg'],new_composition=dict(source_paths_sha256=f.assets,layout_sha256=config['sha256'],scene_sha256=stage['sha256']))
    return document,r7row,new


class Round8RevisionBindings(unittest.TestCase):
    def setUp(self):
        self.f=prior_tests.SceneRevisionBindings('test_new_human_pose_preserves_old_config_and_assets');self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.doc,self.r7row,self.new=add_round8(self.f)

    def verify(self,doc=None):
        binding=self.f.json('revision8.json',self.doc if doc is None else doc)
        return [m.validate_scene_revision(self.f.root,binding,expected_scene_id='scene') for m in MODULES]

    def reject(self,doc):
        binding=self.f.json('revision8.json',doc)
        for m in MODULES:
            with self.subTest(module=m.__name__),self.assertRaises(ValueError):
                m.validate_scene_revision(self.f.root,binding,expected_scene_id='scene')

    def test_root_sources_and_camera_skip_unexecuted_round7_layout(self):
        for result in self.verify():
            self.assertEqual(result['root_source'],{k:self.f.row[k] for k in runtime.SOURCE_KEYS})
            self.assertEqual(result['historical_camera_config'],self.f.oldconfig)
            self.assertEqual(result['document']['old_config'],self.f.newconfig)
            self.assertNotEqual(result['historical_camera_config'],result['document']['old_config'])
            self.assertEqual(result['original_row']['physical_attempt_number'],4)
            self.assertEqual(len(result['original_row']['prior_physical_attempts']),3)
            self.assertEqual(result['original_row']['attempts'],0)

    def test_root_task_object_or_collision_change_cannot_hide_in_second_revision(self):
        for key in ('objects','collision'):
            config=deepcopy(self.new)
            if key=='objects':config['objects']['bowl']['position'][0]+=.01
            else:config['obstacle']['static']=False
            doc=deepcopy(self.doc);doc['new_config']=self.f.json('config8.json',config)
            doc['new_composition']['layout_sha256']=doc['new_config']['sha256'];self.reject(doc)

    def test_round7_planning_failure_cannot_be_promoted_into_an_episode(self):
        prior=deepcopy(self.r7row);prior.update(executed=True,attempts=1)
        doc=deepcopy(self.doc);doc['prior_manifest']=self.f.json('manifest7.json',dict(retry_round=7,stage='complete',entries=[prior]))
        self.reject(doc)

    def test_previous_revision_must_match_the_actual_parent_row(self):
        doc=deepcopy(self.doc);doc['previous_revision']=deepcopy(doc['previous_revision']);doc['previous_revision']['sha256']='0'*64
        self.reject(doc)

    def test_changed_root_asset_invalidates_both_revisions(self):
        self.f.text('asset.usd','modified original collider')
        self.reject(self.doc)


class Round8RendererBindings(unittest.TestCase):
    def setUp(self):
        self.fixture=prior_tests.SceneRevisionRendererBindings('test_new_layout_is_rendered_with_only_old_camera_pose_reused')
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        f=self.fixture.fixture;self.doc,_,new=add_round8(f)
        self.r7_config=f.newconfig
        self.revision=f.json('revision8.json',self.doc)
        f.newconfig=self.doc['new_config'];f.newstage=self.doc['new_source_scene'];f.new=new
        self.fixture.revision=self.revision
        self.fixture.planned.update(config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],scene_revision=self.revision,composition=deepcopy(self.doc['new_composition']))
        candidate=f.json('candidate.json',self.fixture.planned)
        self.fixture.result.update(config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],scene_revision=self.revision,
            planning_result_path=candidate['path'],planning_result_sha256=candidate['sha256'])

    def test_new_state_uses_round8_layout_and_original_camera_not_round7(self):
        result,layout,_,_,binding=self.fixture.load()
        self.assertEqual(layout,self.fixture.fixture.new)
        self.assertEqual(binding['historical_camera_config'],self.fixture.fixture.oldconfig)
        self.assertEqual(result['diagnostic_observation_reference']['config_sha256'],self.fixture.fixture.oldconfig['sha256'])

    def test_result_cannot_rebind_second_revision_to_round7_config(self):
        result=deepcopy(self.fixture.result);result.update(config_path=self.r7_config['path'],config_sha256=self.r7_config['sha256'])
        with self.assertRaisesRegex(ValueError,'sources differ from the new scene revision'):self.fixture.load(result=result)

    def test_second_revision_candidate_cannot_reuse_parent_composition(self):
        planned=deepcopy(self.fixture.planned);planned['composition']=deepcopy(self.fixture.fixture.doc['new_composition'])
        with self.assertRaisesRegex(ValueError,'sources differ from the new scene revision'):self.fixture.load(planned=planned)

    def test_round7_camera_metadata_is_not_accepted_as_root_observation(self):
        camera=deepcopy(self.fixture.camera);camera['config_sha256']=self.r7_config['sha256']
        ref=self.fixture.fixture.json('camera.json',camera);result=deepcopy(self.fixture.result)
        result['prior_physical_attempts'][0]['result_sha256']=ref['sha256']
        with self.assertRaisesRegex(ValueError,'another scene/config'):self.fixture.load(result=result)


if __name__=='__main__':unittest.main()
