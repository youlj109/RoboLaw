"""Round8 must extend the selected round7 revision without rewriting its roots."""
from copy import deepcopy
import unittest
from tests.extensions.replicator import test_scene_revision_bindings as old
from scripts.extensions.replicator.scene_revision_review_round8 import validate_scene_revision,SOURCE_KEYS

class Revision8Tests(unittest.TestCase):
    def setUp(self):
        self.f=old.SceneRevisionBindings('test_new_human_pose_preserves_old_config_and_assets');self.f.setUp();self.addCleanup(self.f.doCleanups)
        f=self.f;self.r7=f.json('r7revision.json',f.doc)
        self.previous_row=dict(f.row,scene_revision=self.r7,layout_changed=True,config_path=f.newconfig['path'],config_sha256=f.newconfig['sha256'],
            source_scene_path=f.newstage['path'],source_scene_sha256=f.newstage['sha256'])
        previous=f.json('r7manifest.json',dict(retry_round=7,stage='complete',entries=[self.previous_row]))
        current=deepcopy(f.new);current['obstacle']['position'][0]+=.1
        self.config=f.json('r8.json',current);self.stage=f.text('r8.usda','r8 selected stage')
        self.doc=dict(kind='hazard_pose_revision',retry_round=8,scene_id='scene',task_id='task',variant_id='human',
            prior_manifest=previous,previous_revision=self.r7,prior_row_source={k:self.previous_row[k] for k in SOURCE_KEYS},
            old_config=f.newconfig,new_config=self.config,old_source_scene=f.newstage,new_source_scene=self.stage,
            changed_fields=['obstacle.position'],new_composition=dict(source_paths_sha256=f.assets,layout_sha256=self.config['sha256'],scene_sha256=self.stage['sha256']))

    def check(self,doc=None):
        return validate_scene_revision(self.f.root,self.f.json('r8revision.json',self.doc if doc is None else doc),expected_scene_id='scene')

    def test_root_source_and_historical_camera_remain_original_not_round7(self):
        checked=self.check();self.assertEqual(checked['root_source'],self.f.doc['prior_row_source'])
        self.assertEqual(checked['historical_camera_config'],self.f.oldconfig)
        self.assertNotEqual(checked['historical_camera_config'],self.doc['old_config'])

    def test_previous_revision_must_be_the_prior_rows_exact_binding(self):
        changed=deepcopy(self.doc);changed['previous_revision']={'path':'wrong','sha256':'0'*64}
        with self.assertRaisesRegex(ValueError,'previous selected layout'):self.check(changed)

    def test_matching_hashes_cannot_hide_an_original_object_change(self):
        changed=deepcopy(self.doc);value=deepcopy(self.f.new);value['obstacle']['position'][0]+=.1;value['objects']['bowl']['position'][2]+=.01
        changed['new_config']=self.f.json('r8.json',value);changed['new_composition']['layout_sha256']=changed['new_config']['sha256']
        with self.assertRaisesRegex(ValueError,'beyond'):self.check(changed)

    def test_original_config_cannot_replace_selected_round7_old_config(self):
        changed=deepcopy(self.doc);changed['old_config']=self.f.oldconfig
        with self.assertRaisesRegex(ValueError,'chain'):self.check(changed)

    def test_previous_planning_failure_cannot_claim_a_physical_episode(self):
        changed=deepcopy(self.doc);bad=dict(self.previous_row,attempts=1,executed=True)
        changed['prior_manifest']=self.f.json('r7manifest.json',dict(retry_round=7,stage='complete',entries=[bad]))
        with self.assertRaisesRegex(ValueError,'unresolved unexecuted'):self.check(changed)

if __name__=='__main__':unittest.main()
