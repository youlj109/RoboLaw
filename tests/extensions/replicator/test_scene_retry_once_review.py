"""Regression checks for cross-round scene-change and failed-worker provenance."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('scene_retry_review_runner', ROOT/'scripts/extensions/replicator/quick_trajectory_retry.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class OneSceneChangeReviewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source, self.output = self.root/'prior', self.root/'new'
        self.old_path, self.new_path = self.root/'old.json', self.root/'new.json'
        self.old = dict(task_id='test', robot_position=[0,0,0],
                        obstacle=dict(position=[0,0,0],yaw_deg=0,usd_path='same.usd'))
        self.new = deepcopy(self.old); self.new['obstacle']['position'][0] = .1
        runner.atomic_json(self.old_path, self.old); runner.atomic_json(self.new_path,self.new)
        self.change = dict(adjustments=1,source_config_path=str(self.old_path),
            source_config_sha256=runner.sha(self.old_path),config_path=str(self.new_path),config_sha256=runner.sha(self.new_path))
        self.prior_path = self.source/'scenes'/'153'/'result.json'

    def source_manifest(self):
        rows = []
        for index in range(154):
            sid = str(index); status = 'failed' if index==153 else 'generated'
            result = dict(scene_id=sid,status=status,stage='pregrasp' if index==153 else 'complete',
                composition=dict(source_paths_sha256={},layout_sha256=runner.sha(self.old_path)))
            if index==153:
                result.update(scene_settings_changed_once=True,scene_adjustment=dict(adjustments=1),layout_fixed=False)
            runner.atomic_json(self.source/'scenes'/sid/'result.json',result)
            rows.append(dict(scene_id=sid,status=status,stage=result['stage'],
                             config_path=str(self.old_path),config_sha256=runner.sha(self.old_path)))
        runner.atomic_json(self.source/'progress.json',dict(entries=rows,retry_round=8,
            checks_not_performed=['physics_execution'],scene_settings_changed_once=True))
        recipe = dict(action='plan',layout_fixed=False,config_sha256=runner.sha(self.old_path),
            result_sha256=runner.sha(self.prior_path),scene_adjustment=self.change)
        manifest = self.root/'manifest.json'
        runner.atomic_json(manifest,dict(schema_version=1,layout_fixed=False,
            scene_settings_changed_once=True,recipes={'153':recipe}))
        return manifest

    def test_second_scene_change_rejected_before_output_even_if_counter_reset_to_one(self):
        manifest=self.source_manifest()
        with patch.object(runner,'ROOT',self.root):
            with self.assertRaises(ValueError):
                runner.initialize(self.output,self.source,manifest)
        self.assertFalse(self.output.exists())

    def test_new_user_round_is_bound_to_source_and_one_output(self):
        manifest=self.source_manifest()
        progress=json.loads((self.source/'progress.json').read_text());progress['stage']='complete'
        runner.atomic_json(self.source/'progress.json',progress)
        auth=self.root/'authorization.json'
        runner.atomic_json(auth,dict(user_request='Continue one new round',source_run='prior',output_run='new',
            source_progress_sha256=runner.sha(self.source/'progress.json'),source_retry_round=8,retry_round=9,
            scene_ids=['153'],max_scene_adjustments_per_scene_this_round=1,max_planning_passes_per_scene_this_round=1))
        doc=json.loads(manifest.read_text());doc['scene_adjustment_authorization']=dict(path='authorization.json',sha256=runner.sha(auth))
        runner.atomic_json(manifest,doc)
        with patch.object(runner,'ROOT',self.root):
            with self.assertRaises(ValueError):runner.initialize(self.root/'other',self.source,manifest)
            result=runner.initialize(self.output,self.source,manifest)
            self.assertEqual(result['retry_round'],9)
            self.assertEqual(result['scene_adjustment_count'],1)
            with self.assertRaises(FileExistsError):runner.initialize(self.output,self.source,manifest)
        # Changing even the source progress invalidates the authorization.
        progress['retry_round']=9;runner.atomic_json(self.source/'progress.json',progress)
        with patch.object(runner,'ROOT',self.root):
            with self.assertRaises(ValueError):runner.initialize(self.output,self.source,manifest)

    def test_failed_adjusted_worker_cannot_label_old_scene_composition_as_new(self):
        old_composition = dict(source_paths_sha256={},layout_sha256=runner.sha(self.old_path),scene_sha256='a'*64)
        runner.atomic_json(self.prior_path,dict(scene_id='153',composition=old_composition,start_q=[0]*7))
        row = dict(scene_id='153',config_path=str(self.new_path),config_sha256=runner.sha(self.new_path),
            prior_result_path=str(self.prior_path),prior_result_sha256=runner.sha(self.prior_path),
            retry_round=8,scene_adjustment=self.change,recipe=dict(action='plan',scene_adjustment=self.change))
        with patch.object(runner,'ROOT',self.root):
            result = runner.error_result(row,self.output,'INTERRUPTED_ONCE','No second attempt')
        self.assertEqual(result.get('scene_adjustment'),self.change)
        self.assertIs(result.get('scene_settings_changed_once'),True)
        self.assertIs(result.get('layout_fixed'),False)
        if result.get('composition'):
            self.assertEqual(result['composition'].get('layout_sha256'),runner.sha(self.new_path))
        self.assertEqual(result.get('prior_result_sha256'),runner.sha(self.prior_path))
        self.assertEqual(result.get('phases'),[])
        self.assertEqual(result.get('artifacts'),{})

    def test_failed_adjusted_worker_retains_only_matching_current_composition(self):
        old_composition = dict(source_paths_sha256={},layout_sha256=runner.sha(self.old_path),scene_sha256='a'*64)
        runner.atomic_json(self.prior_path,dict(scene_id='153',composition=old_composition,start_q=[0]*7))
        row = dict(scene_id='153',config_path=str(self.new_path),config_sha256=runner.sha(self.new_path),
            prior_result_path=str(self.prior_path),prior_result_sha256=runner.sha(self.prior_path),
            retry_round=8,scene_adjustment=self.change,recipe=dict(action='plan',scene_adjustment=self.change))
        for current_layout_hash in (runner.sha(self.old_path),runner.sha(self.new_path)):
            with self.subTest(current_layout_hash=current_layout_hash):
                current_composition = dict(source_paths_sha256={},layout_sha256=current_layout_hash,scene_sha256='b'*64)
                runner.atomic_json(self.output/'scenes'/'153'/'result.json',dict(
                    config_sha256=runner.sha(self.new_path),composition=current_composition))
                with patch.object(runner,'ROOT',self.root):
                    result = runner.error_result(row,self.output,'INTERRUPTED_ONCE','No second attempt')
                self.assertEqual(result.get('previous_composition'),old_composition)
                if current_layout_hash == runner.sha(self.new_path):
                    self.assertEqual(result.get('composition'),current_composition)
                else:
                    self.assertNotIn('composition',result)
                self.assertEqual(result.get('phases'),[])
                self.assertEqual(result.get('artifacts'),{})


if __name__=='__main__':
    unittest.main()
