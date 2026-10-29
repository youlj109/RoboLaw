"""Round11 retains round8 geometry and keeps result/history/notes separate."""
from copy import deepcopy
from http.server import ThreadingHTTPServer
import json
import threading
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request

from scripts.extensions.replicator import progress_dashboard as dashboard
from scripts.extensions.replicator.package_trajectory_demos import sha
from scripts.extensions.replicator.retained_scene_revision_review_round11 import validate_retained_scene_revision
from scripts.extensions.replicator import retained_scene_revision_review_round11 as retained11
from tests.extensions.replicator import test_physical_retry10_dashboard as round10


class Retry11DashboardTests(unittest.TestCase):
    def setUp(self):
        previous=round10.Retry10DashboardTests('test_151_original_and_retained_round7_success_gain_only_one_round10_success'
            if '151' in self._testMethodName else 'test_pending_retained_layout_is_visible_without_inventing_revision_or_episode')
        previous.setUp();self.addCleanup(previous.doCleanups);self.previous=previous
        size=patch.object(retained11,'COHORT_SIZE',1);size.start();self.addCleanup(size.stop)
        self.f=previous.f;self.progress=previous.progress;self.sid=previous.sid
        for key,value in [('PHYSICAL_RETRY11_DEMOS','exports/retry11/'),('PHYSICAL_RETRY11_PLAN','configs/retry11.json'),
            ('PHYSICAL_RETRY11_PLANNING','reports/retry11planning/'),('PHYSICAL_RETRY11_DIAGNOSTICS','reports/retry11diagnostic/'),
            ('PHYSICAL_RETRY11_DETAIL_DIAGNOSTICS','reports/retry11detail/'),('PHYSICAL_RETRY11_DARK_DIAGNOSTICS','reports/retry11dark/'),
            ('PHYSICAL_RETRY11_OVERVIEW_DIAGNOSTICS','reports/retry11overview/'),('PHYSICAL_RETRY11_TOP_DIAGNOSTICS','reports/retry11top/'),('PHYSICAL_RETRY11_TOTAL',1)]:
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)

    def preparation(self,planning_failed=False,pending=False,prior_actual=False):
        self.previous.initialize(planning_failed=True,prior_actual=prior_actual)
        f=self.f;old=self.previous.row
        self.previous_ref=dict(path=dashboard.PHYSICAL_RETRY10_DEMOS+'manifest.json',sha256=sha(f.root/dashboard.PHYSICAL_RETRY10_DEMOS/'manifest.json'))
        self.history=deepcopy(old['prior_physical_attempts'])
        if old['executed']:
            self.history.append(dict(result_path=old['execution_result_path'],result_sha256=old['execution_result_sha256'],
                **{k:self.previous.result.get(k) for k in ('execution_status','task_success','executed','video_sha256')}))
        self.config_ref=self.previous.config_ref;self.scene_ref=self.previous.scene_ref;self.revision_ref=self.previous.revision_ref
        self.flags=dict(layout_changed=True,retained_scene_revision=True,scene_settings_changed_this_round=False,scene_settings_changed_once=True)
        self.planning=dict(source_previous_retry_manifest=self.previous_ref,entries=[dict(scene_id=self.sid,status='pending',attempts=0)])
        if pending:f.write(dashboard.PHYSICAL_RETRY11_PLANNING+'progress.json',self.planning);return
        raw=f.write(dashboard.PHYSICAL_RETRY11_PLANNING+'scenes/'+self.sid+'/trajectory.npz',b'round11 raw')
        adjusted=f.write(dashboard.PHYSICAL_RETRY11_PLANNING+'retimed.npz',b'round11 retimed')
        self.candidate={k:old[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256','scene_revision')}
        self.candidate.update(**self.flags,composition=deepcopy(self.previous.candidate['composition']),
            status='failed' if planning_failed else 'generated',planning_only=True,executed=False,attempts=1,
            trajectory_complete=not planning_failed,artifacts={} if planning_failed else dict(trajectory=dict(path=f.rel(raw),sha256=sha(raw))))
        self.candidate_path=f.write(dashboard.PHYSICAL_RETRY11_PLANNING+'scenes/'+self.sid+'/result.json',self.candidate)
        self.candidate_ref=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path))
        self.planning['entries']=[dict(scene_id=self.sid,status=self.candidate['status'],attempts=1,scene_revision=self.revision_ref,
            candidate_planning_result=self.candidate_ref,trajectory=None if planning_failed else dict(path=f.rel(adjusted),sha256=sha(adjusted)),recipe=dict(description='Retain r8 layout'))]
        f.write(dashboard.PHYSICAL_RETRY11_PLANNING+'progress.json',self.planning)
        self.planrow=dict(self.previous.planrow,**self.flags,planning_status=self.candidate['status'],candidate_planning_result=self.candidate_ref,
            previous_retry_result=dict(path=self.history[-1]['result_path'],sha256=self.history[-1]['result_sha256']),physical_attempt_number=len(self.history)+1,
            trajectory_path=None if planning_failed else f.rel(adjusted),trajectory_sha256=None if planning_failed else sha(adjusted),failure_reason='IK_FAIL' if planning_failed else None)

    def initialize(self,planning_failed=False,prior_actual=False):
        self.preparation(planning_failed,prior_actual=prior_actual)
        self.plan=dict(self.previous.plan,retry_round=11,source_previous_retry_manifest=self.previous_ref,layout_changed=False,layout_changed_from_original=True,entries=[self.planrow])
        path=self.f.write(dashboard.PHYSICAL_RETRY11_PLAN,self.plan)
        self.row=dict(self.planrow,retry_schema_version=2,retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),prior_physical_attempts=self.history,
            execution_status='planning_failed' if planning_failed else 'pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(self.previous.manifest,retry_round=11,stage='complete' if planning_failed else 'executing',source_previous_retry_manifest=self.previous_ref,
            retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),entries=[self.row])
        self.save_manifest()

    def save_manifest(self):self.f.write(dashboard.PHYSICAL_RETRY11_DEMOS+'manifest.json',self.manifest)

    def terminal(self,status='success',prior_actual=False):
        self.initialize(prior_actual=prior_actual);f=self.f
        path=f.root/dashboard.PHYSICAL_RETRY11_DEMOS/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'round11 states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=23))
        self.row.update(execution_status=status,executed=True,task_success=status=='success',attempts=1)
        self.result=dict(self.row,kind='physical_failure_retry_execution',status='complete',planning_result_path=self.candidate_ref['path'],planning_result_sha256=self.candidate_ref['sha256'],
            original_planning_result=dict(path=self.row['result_path'],sha256=self.row['result_sha256']),rendering_enabled=False,video_expected=False,
            original_task_success=status=='success',final_original_destination_predicate=status=='success',source_previous_retry_manifest=self.previous_ref,
            source_inputs_unchanged=True,execution_sources_unchanged=True,render_generation_delta=0,render_state_start={'passed':True},render_state_end={'passed':True},
            physics_evidence=dict(q_path=f.rel(states),q_sha256=sha(states),metadata_path=f.rel(meta),metadata_sha256=sha(meta),complete=True,continuous=True))
        f.write(path,self.result);self.row.update(execution_result_path=f.rel(path),execution_result_sha256=sha(path));self.manifest['stage']='complete';self.save_manifest()

    def serve(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(lambda:(server.shutdown(),server.server_close(),thread.join()))
        return f'http://127.0.0.1:{server.server_port}'

    def test_151_original_and_retained_round7_success_gain_only_one_round11_success(self):
        self.terminal();original=self.progress.physical_retries
        def groups(*args,**kwargs):
            value=original(*args,**kwargs)
            if kwargs.get('retry_round')==7:value=dict(value,counts=dict(value['counts'],success=1))
            return value
        with patch.object(self.progress,'physical_retries',groups):data=self.progress.physical_execution()
        row=data['failure_retry_round11']['entries'][0]
        self.assertFalse(row['stale'],row['reason']);self.assertTrue(row['result_valid'])
        self.assertEqual(data['combined_latest_counts']['success'],151)
        self.assertEqual(data['current_revision_counts']['success'],153)
        self.assertEqual(data['current_revision_counts']['revision_success_by_round'],{'7':1,'8':0,'9':0,'10':0,'11':1})
        self.assertEqual(row['config_sha256'],self.previous.row['config_sha256'])
        self.assertTrue(row['retained_scene_revision']);self.assertFalse(row['scene_settings_changed_this_round'])
        self.assertEqual(row['physical_attempt_number'],self.previous.row['physical_attempt_number'])
        self.assertEqual(self.history,self.previous.row['prior_physical_attempts'])

    def test_pending_retained_layout_is_visible_without_inventing_revision_or_episode(self):
        self.preparation(pending=True);retry=self.progress.physical_execution()['failure_retry_round11'];row=retry['entries'][0]
        self.assertTrue(retry['planning_present']);self.assertFalse(retry['manifest_present'])
        self.assertFalse(row['stale'],row['reason']);self.assertEqual(row['execution_status'],'planning_pending')
        self.assertEqual(row['attempts'],0);self.assertFalse(row['executed'])
        self.assertEqual(row['scene_revision'],self.revision_ref);self.assertEqual(row['selected_layout_revision'],8)

    def test_planning_failed_keeps_full_prior_history_and_zero_new_episode(self):
        self.initialize(planning_failed=True,prior_actual=True)
        retry=self.progress.physical_execution()['failure_retry_round11'];row=retry['entries'][0]
        self.assertFalse(row['stale'],row['reason']);self.assertEqual(retry['counts']['planning_failed'],1);self.assertEqual(retry['counts']['failed'],0)
        self.assertFalse(row['executed']);self.assertEqual(row['attempts'],0);self.assertIsNone(row['result_url'])
        self.assertEqual(self.history,self.previous.row['prior_physical_attempts'])
        self.assertEqual(row['physical_attempt_number'],self.previous.row['physical_attempt_number'])

    def test_retained_helper_rejects_changed_source_flags_previous_status_or_sha(self):
        self.initialize(planning_failed=True)
        def checked(row=None,previous=None):return validate_retained_scene_revision(self.f.root,self.revision_ref,previous or self.previous_ref,expected_scene_id=self.sid,current=row or self.row)
        self.assertTrue(checked()['retained_scene_revision'])
        for key,value in [('config_sha256','0'*64),('source_scene_sha256','0'*64),('layout_changed',False),
            ('retained_scene_revision',False),('scene_settings_changed_this_round',True),('scene_settings_changed_once',False)]:
            with self.subTest(key=key),self.assertRaises(ValueError):checked(dict(self.row,**{key:value}))
        with self.assertRaises(ValueError):checked(previous=dict(self.previous_ref,sha256='0'*64))
        path=self.f.root/self.previous_ref['path'];before=path.read_bytes()
        prior=json.loads(before);prior['entries'][0].update(attempts=1,executed=True)
        self.f.write(path,prior)
        with self.assertRaisesRegex(ValueError,'invented an actual episode'):checked(previous=dict(self.previous_ref,sha256=sha(path)))
        path.write_bytes(before)

    def test_bound_source_tamper_withholds_current_success(self):
        self.terminal()
        for binding in (self.config_ref,self.scene_ref,self.revision_ref):
            path=self.f.root/binding['path'];before=path.read_bytes();path.write_bytes(b'changed retained source')
            with self.subTest(path=binding['path']):
                data=self.progress.physical_execution()
                self.assertEqual(data['failure_retry_round11']['counts']['success'],0)
                self.assertEqual(data['failure_retry_round11']['counts']['stale'],1)
                self.assertEqual(data['combined_latest_counts']['success'],2)
            path.write_bytes(before)

    def test_round11_notes_use_new_result_even_with_same_config_and_leave_1_to_10_unchanged(self):
        self.terminal('failed',prior_actual=True)
        prior={n:self.progress.physical_retry_notes_path(n) for n in range(1,11)}
        for n,path in prior.items():self.f.write(path,dict(entries=[],sentinel=f'round{n} unchanged'))
        before={n:p.read_bytes() for n,p in prior.items()};baseurl=self.serve();url=baseurl+'/api/physical-retry-round11-annotations'
        payload=dict(scene_id=self.sid,note='r9 only',expected_revision=0,config_sha256=self.row['config_sha256'],result_sha256=self.row['execution_result_sha256'])
        request=urllib.request.Request(url,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
        with urllib.request.urlopen(request) as response:self.assertEqual(json.load(response)['entry']['revision'],1)
        with urllib.request.urlopen(url) as response:self.assertEqual(json.load(response)['entries'][0]['note'],'r9 only')
        self.assertEqual(self.row['config_sha256'],self.previous.row['config_sha256'])
        with self.assertRaises(dashboard.AnnotationError):self.progress.save_physical_retry_annotation(dict(payload,expected_revision=1,result_sha256=self.previous.previous.previous.row['execution_result_sha256']),11)
        for n,p in prior.items():self.assertEqual(p.read_bytes(),before[n])

    def test_production_two_scene_retention_checks_each_prior_zero_episode(self):
        self.initialize(planning_failed=True)
        path=self.f.root/self.previous_ref['path'];prior=json.loads(path.read_text())
        with patch.object(retained11,'COHORT_SIZE',2):
            with self.assertRaisesRegex(ValueError,'two-scene'):
                validate_retained_scene_revision(self.f.root,self.revision_ref,self.previous_ref,expected_scene_id=self.sid,current=self.row)
            sibling=deepcopy(prior['entries'][0]);sibling['scene_id']='second_unresolved_scene'
            prior['entries'].append(sibling);self.f.write(path,prior)
            reference=dict(self.previous_ref,sha256=sha(path))
            checked=validate_retained_scene_revision(self.f.root,self.revision_ref,reference,expected_scene_id=self.sid,current=self.row)
            self.assertEqual(checked['retained_from_manifest'],reference)
            for change in (dict(attempts=1),dict(executed=True),dict(execution_status='failed'),dict(execution_result_path='invented.json')):
                broken=deepcopy(prior);broken['entries'][1].update(change);self.f.write(path,broken)
                with self.subTest(change=change),self.assertRaisesRegex(ValueError,'invented an actual episode'):
                    validate_retained_scene_revision(self.f.root,self.revision_ref,dict(reference,sha256=sha(path)),expected_scene_id=self.sid,current=self.row)

    def test_round10_and_11_bound_artifacts_serve_but_orphans_and_raw_arrays_do_not(self):
        self.terminal();url=self.serve();data=self.progress.physical_execution()
        for field in ('failure_retry_round11','failure_retry_round11'):
            row=data[field]['entries'][0];self.assertFalse(row['stale'],row['reason'])
            with urllib.request.urlopen(url+row['planning_result_url']) as response:self.assertEqual(json.load(response)['scene_id'],self.sid)
        with urllib.request.urlopen(url+data['failure_retry_round11']['entries'][0]['result_url']) as response:self.assertEqual(json.load(response)['execution_status'],'success')
        for prefix in (dashboard.PHYSICAL_RETRY10_PLANNING,dashboard.PHYSICAL_RETRY11_PLANNING):
            orphan=prefix+'scenes/orphan/result.json';self.f.write(orphan,dict(unbound=True))
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(url+'/artifact/'+orphan)
            self.assertEqual(caught.exception.code,404);caught.exception.close()
            self.assertFalse(dashboard.allowed_artifact(prefix+'scenes/'+self.sid+'/trajectory.npz'))
        for prefix in (dashboard.PHYSICAL_RETRY10_DIAGNOSTICS,dashboard.PHYSICAL_RETRY11_DIAGNOSTICS):
            self.assertTrue(dashboard.allowed_artifact(prefix+'scenes/'+self.sid+'/frame_00.jpg'))
            self.assertFalse(dashboard.allowed_artifact(prefix+'scenes/'+self.sid+'/physics_states.npz'))


if __name__=='__main__':unittest.main()
