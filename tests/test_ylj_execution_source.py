"""CPU-only regression gates for adopted revision selection and provenance."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from kitchen_safety.ylj_execution_source import execution_index, required_source_bindings, load_selected_inputs
from kitchen_safety.ylj_risk_capture import select_entry, input_preflight


class AdoptedRevisionSource(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(); self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.base = []
        self.published = dict(partial_preview=False, entries=[])
        for i in range(154):
            sid = 'scene_' + str(i)
            entry = dict(scene_id=sid, task_id='kitchen_bench_task', variant_id='property_tall_glass',
                         config_path='config.json', config_sha256='c',
                         source_result_path='old_result.json', source_result_sha256='r',
                         artifacts={'trajectory': {'path':'old.npz','sha256':'t'}},
                         source_scene={'path':'scene.usda','sha256':'s'})
            self.base.append(entry)
            result = dict(path='native_' + sid + '.json', sha256='physical')
            self.published['entries'].append(dict(scene_id=sid, task_id=entry['task_id'], variant_id=entry['variant_id'],
                current_revision_result=result, current_revision_config=dict(path='config.json',sha256='c'),
                current_revision=None, first_pass=dict(result=result,executed=True,task_success=True,execution_status='success')))
        self.write('configs/extensions/replicator/final_154_scenes_20261028.json',dict(entries=self.base))
        self.lock = dict(source_commit='new_commit', publication=dict(path='published.json',sha256=''),
                         legacy_index_sha256=self.sha('configs/extensions/replicator/final_154_scenes_20261028.json'), retry_plans={})
        self.sync()

    def write(self, path, value):
        target=self.root/path;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(value));return dict(path=path,sha256=self.sha(path))

    def sha(self,path):return hashlib.sha256((self.root/path).read_bytes()).hexdigest()

    def sync(self):
        self.lock['publication']=self.write('published.json',self.published)
        self.write('configs/risk_integration/ylj_execution_source.json',self.lock)

    def retry(self):
        original = select_entry(dict(entries=self.base),'scene_0')
        retry = dict(original, source_trajectory_path='old.npz',source_trajectory_sha256='t',
                     trajectory_path='new.npz',trajectory_sha256='new_sha',
                     adjustment=dict(kind='timing',description='slower',evidence_path='evidence.json',evidence_sha256='e'))
        result=dict(path='retry_result.json',sha256='retry_result_sha')
        row=self.published['entries'][0];row.update(current_revision_result=result,
            retry=dict(result=result,executed=True,task_success=True,execution_status='success',
                       adjusted_trajectory=dict(path='new.npz',sha256='new_sha')))
        self.lock['retry_plans']['1']=self.write('retry_plan.json',dict(entries=[retry],execution_settings=dict(close_seconds=2.,release_seconds=2.)))
        self.sync();return retry

    def test_legacy_index_is_exactly_unchanged(self):
        self.assertEqual(execution_index(self.root,'legacy'),dict(entries=self.base))

    def test_current_selects_exact_154_published_first_pass_bindings(self):
        result=execution_index(self.root,'current');self.assertEqual(len(result['entries']),154)
        self.assertEqual(result['source_commit'],'new_commit')
        self.assertEqual(result['entries'][0]['ylj_source_binding']['selected_layer'],'first_pass')

    def test_timing_retry_replaces_actual_npz_and_preserves_original_binding(self):
        original=self.retry();e=select_entry(execution_index(self.root,'current'),'scene_0')
        self.assertEqual(e['trajectory_path'],'new.npz');self.assertEqual(e['ylj_retry_entry'],original)
        self.assertEqual(e['ylj_source_binding']['selected_layer'],'retry')
        self.assertIn('old.npz',required_source_bindings(e));self.assertIn('new.npz',required_source_bindings(e))

    def test_changed_publication_fails_instead_of_falling_back(self):
        self.write('published.json',dict(partial_preview=False,entries=[]))
        with self.assertRaisesRegex(ValueError,'SHA'):execution_index(self.root,'current')

    def test_partial_publication_is_rejected(self):
        self.published['partial_preview']=True;self.sync()
        with self.assertRaisesRegex(ValueError,'incomplete'):execution_index(self.root,'current')

    def test_missing_scene_identity_is_rejected(self):
        self.published['entries'].pop();self.sync()
        with self.assertRaisesRegex(ValueError,'154'):execution_index(self.root,'current')

    def test_unexecuted_published_success_is_rejected(self):
        self.published['entries'][0]['first_pass']['executed']=False;self.sync()
        with self.assertRaisesRegex(ValueError,'executed success'):execution_index(self.root,'current')

    def test_retry_trajectory_hash_must_match_publication(self):
        self.retry();self.published['entries'][0]['retry']['adjusted_trajectory']['sha256']='wrong';self.sync()
        with self.assertRaisesRegex(ValueError,'trajectory'):execution_index(self.root,'current')

    def test_duplicate_result_lineage_is_rejected(self):
        row=self.published['entries'][0];row['camera_retake']=deepcopy(row['first_pass']);self.sync()
        with self.assertRaisesRegex(ValueError,'one exact'):execution_index(self.root,'current')

    def test_missing_new_inputs_fail_before_loading_runtime(self):
        self.retry();index=execution_index(self.root,'current')
        with patch('kitchen_safety.ylj_risk_capture.ROOT',self.root),patch('kitchen_safety.ylj_execution_source.load_selected_inputs') as loader:
            preflight=input_preflight({'task_id':'task'},index,'scene_0',{'human_collision_paths':[],'pt_targets':{}},allow_added_obstacle=True)
        self.assertFalse(preflight['ready']);self.assertIn('new.npz',preflight['missing_inputs']);loader.assert_not_called()

    def test_runtime_selects_retry_loader_and_cannot_use_old_loader(self):
        self.retry();entry=select_entry(execution_index(self.root,'current'),'scene_0')
        with patch('kitchen_safety.extensions.replicator.retry_physical_execution.load_retry_inputs',return_value=('new','layout','data')) as retry_loader,patch('kitchen_safety.extensions.replicator.planned_execution.load_inputs') as old_loader:
            self.assertEqual(load_selected_inputs(self.root,entry),('new','layout','data'))
        retry_loader.assert_called_once_with(entry['ylj_retry_entry']);old_loader.assert_not_called()

    def test_mutated_selected_entry_is_rejected_at_runtime(self):
        self.retry();entry=select_entry(execution_index(self.root,'current'),'scene_0');entry['trajectory_path']='old.npz'
        with self.assertRaisesRegex(ValueError,'after preflight'):load_selected_inputs(self.root,entry)


if __name__=='__main__':unittest.main()
