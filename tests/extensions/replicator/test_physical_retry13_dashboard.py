"""Round13 retains round8 geometry and keeps result/history/notes separate."""
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
from scripts.extensions.replicator.retained_scene_revision_review_round13 import validate_retained_scene_revision
from scripts.extensions.replicator import retained_scene_revision_review_round13 as retained13
from tests.extensions.replicator import test_physical_retry12_dashboard as round12


class Retry13DashboardTests(unittest.TestCase):
    def setUp(self):
        previous=round12.Retry12DashboardTests('test_151_original_and_retained_round7_success_gain_only_one_round12_success'
            if '151' in self._testMethodName else 'test_pending_retained_layout_is_visible_without_inventing_revision_or_episode')
        previous.setUp();self.addCleanup(previous.doCleanups);self.previous=previous
        self.f=previous.f;self.progress=previous.progress;self.sid=previous.sid
        for key,value in [('PHYSICAL_RETRY13_DEMOS','exports/retry13/'),('PHYSICAL_RETRY13_PLAN','configs/retry13.json'),
            ('PHYSICAL_RETRY13_PLANNING','reports/retry13planning/'),('PHYSICAL_RETRY13_DIAGNOSTICS','reports/retry13diagnostic/'),
            ('PHYSICAL_RETRY13_DETAIL_DIAGNOSTICS','reports/retry13detail/'),('PHYSICAL_RETRY13_DARK_DIAGNOSTICS','reports/retry13dark/'),
            ('PHYSICAL_RETRY13_OVERVIEW_DIAGNOSTICS','reports/retry13overview/'),('PHYSICAL_RETRY13_TOP_DIAGNOSTICS','reports/retry13top/'),('PHYSICAL_RETRY13_TOTAL',1)]:
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)

    def preparation(self,planning_failed=False,pending=False,prior_actual=False):
        self.previous.initialize(planning_failed=True,prior_actual=prior_actual)
        f=self.f;old=self.previous.row
        self.previous_ref=dict(path=dashboard.PHYSICAL_RETRY12_DEMOS+'manifest.json',sha256=sha(f.root/dashboard.PHYSICAL_RETRY12_DEMOS/'manifest.json'))
        self.history=deepcopy(old['prior_physical_attempts'])
        if old['executed']:
            self.history.append(dict(result_path=old['execution_result_path'],result_sha256=old['execution_result_sha256'],
                **{k:self.previous.result.get(k) for k in ('execution_status','task_success','executed','video_sha256')}))
        self.config_ref=self.previous.config_ref;self.scene_ref=self.previous.scene_ref;self.revision_ref=self.previous.revision_ref
        self.flags=dict(layout_changed=True,retained_scene_revision=True,scene_settings_changed_this_round=False,scene_settings_changed_once=True)
        self.planning=dict(source_previous_retry_manifest=self.previous_ref,entries=[dict(scene_id=self.sid,status='pending',attempts=0)])
        if pending:f.write(dashboard.PHYSICAL_RETRY13_PLANNING+'progress.json',self.planning);return
        raw=f.write(dashboard.PHYSICAL_RETRY13_PLANNING+'scenes/'+self.sid+'/trajectory.npz',b'round13 raw')
        adjusted=f.write(dashboard.PHYSICAL_RETRY13_PLANNING+'retimed.npz',b'round13 retimed')
        self.candidate={k:old[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256','scene_revision')}
        self.candidate.update(**self.flags,composition=deepcopy(self.previous.candidate['composition']),
            status='failed' if planning_failed else 'generated',planning_only=True,executed=False,attempts=1,
            trajectory_complete=not planning_failed,artifacts={} if planning_failed else dict(trajectory=dict(path=f.rel(raw),sha256=sha(raw))))
        self.candidate_path=f.write(dashboard.PHYSICAL_RETRY13_PLANNING+'scenes/'+self.sid+'/result.json',self.candidate)
        self.candidate_ref=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path))
        self.planning['entries']=[dict(scene_id=self.sid,status=self.candidate['status'],attempts=1,scene_revision=self.revision_ref,
            candidate_planning_result=self.candidate_ref,trajectory=None if planning_failed else dict(path=f.rel(adjusted),sha256=sha(adjusted)),recipe=dict(description='Retain r8 layout'))]
        f.write(dashboard.PHYSICAL_RETRY13_PLANNING+'progress.json',self.planning)
        self.planrow=dict(self.previous.planrow,**self.flags,planning_status=self.candidate['status'],candidate_planning_result=self.candidate_ref,
            previous_retry_result=dict(path=self.history[-1]['result_path'],sha256=self.history[-1]['result_sha256']),physical_attempt_number=len(self.history)+1,
            trajectory_path=None if planning_failed else f.rel(adjusted),trajectory_sha256=None if planning_failed else sha(adjusted),failure_reason='IK_FAIL' if planning_failed else None)

    def initialize(self,planning_failed=False,prior_actual=False):
        self.preparation(planning_failed,prior_actual=prior_actual)
        self.plan=dict(self.previous.plan,retry_round=13,source_previous_retry_manifest=self.previous_ref,layout_changed=False,layout_changed_from_original=True,entries=[self.planrow])
        path=self.f.write(dashboard.PHYSICAL_RETRY13_PLAN,self.plan)
        self.row=dict(self.planrow,retry_schema_version=2,retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),prior_physical_attempts=self.history,
            execution_status='planning_failed' if planning_failed else 'pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(self.previous.manifest,retry_round=13,stage='complete' if planning_failed else 'executing',source_previous_retry_manifest=self.previous_ref,
            retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),entries=[self.row])
        self.save_manifest()

    def save_manifest(self):self.f.write(dashboard.PHYSICAL_RETRY13_DEMOS+'manifest.json',self.manifest)

    def terminal(self,status='success',prior_actual=False):
        self.initialize(prior_actual=prior_actual);f=self.f
        path=f.root/dashboard.PHYSICAL_RETRY13_DEMOS/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'round13 states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=23))
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

    def test_151_original_and_retained_round7_success_gain_only_one_round13_success(self):
        self.terminal();original=self.progress.physical_retries
        def groups(*args,**kwargs):
            value=original(*args,**kwargs)
            if kwargs.get('retry_round')==7:value=dict(value,counts=dict(value['counts'],success=1))
            return value
        with patch.object(self.progress,'physical_retries',groups):data=self.progress.physical_execution()
        row=data['failure_retry_round13']['entries'][0]
        self.assertFalse(row['stale'],row['reason']);self.assertTrue(row['result_valid'])
        self.assertEqual(data['combined_latest_counts']['success'],151)
        self.assertEqual(data['current_revision_counts']['success'],154)
        self.assertEqual(data['current_revision_counts']['revision_success_by_round'],{'7':1,'8':0,'9':0,'10':0,'11':1,'12':0,'13':1})
        self.assertEqual(row['config_sha256'],self.previous.row['config_sha256'])
        self.assertTrue(row['retained_scene_revision']);self.assertFalse(row['scene_settings_changed_this_round'])
        self.assertEqual(row['physical_attempt_number'],self.previous.row['physical_attempt_number'])
        self.assertEqual(self.history,self.previous.row['prior_physical_attempts'])

    def test_pending_retained_layout_is_visible_without_inventing_revision_or_episode(self):
        self.preparation(pending=True);retry=self.progress.physical_execution()['failure_retry_round13'];row=retry['entries'][0]
        self.assertTrue(retry['planning_present']);self.assertFalse(retry['manifest_present'])
        self.assertFalse(row['stale'],row['reason']);self.assertEqual(row['execution_status'],'planning_pending')
        self.assertEqual(row['attempts'],0);self.assertFalse(row['executed'])
        self.assertEqual(row['scene_revision'],self.revision_ref);self.assertEqual(row['selected_layout_revision'],8)

    def test_planning_failed_keeps_full_prior_history_and_zero_new_episode(self):
        self.initialize(planning_failed=True,prior_actual=True)
        retry=self.progress.physical_execution()['failure_retry_round13'];row=retry['entries'][0]
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
                self.assertEqual(data['failure_retry_round13']['counts']['success'],0)
                self.assertEqual(data['failure_retry_round13']['counts']['stale'],1)
                self.assertEqual(data['combined_latest_counts']['success'],2)
            path.write_bytes(before)

    def test_round13_notes_use_new_result_even_with_same_config_and_leave_1_to_12_unchanged(self):
        self.terminal('failed',prior_actual=True)
        prior={n:self.progress.physical_retry_notes_path(n) for n in range(1,13)}
        for n,path in prior.items():self.f.write(path,dict(entries=[],sentinel=f'round{n} unchanged'))
        before={n:p.read_bytes() for n,p in prior.items()};baseurl=self.serve();url=baseurl+'/api/physical-retry-round13-annotations'
        payload=dict(scene_id=self.sid,note='r13 only',expected_revision=0,config_sha256=self.row['config_sha256'],result_sha256=self.row['execution_result_sha256'])
        request=urllib.request.Request(url,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
        with urllib.request.urlopen(request) as response:self.assertEqual(json.load(response)['entry']['revision'],1)
        with urllib.request.urlopen(url) as response:self.assertEqual(json.load(response)['entries'][0]['note'],'r13 only')
        self.assertEqual(self.row['config_sha256'],self.previous.row['config_sha256'])
        with self.assertRaises(dashboard.AnnotationError):self.progress.save_physical_retry_annotation(dict(payload,expected_revision=1,result_sha256=self.history[-1]['result_sha256']),13)
        for n,p in prior.items():self.assertEqual(p.read_bytes(),before[n])

    def test_151_original_and_previous_success_remain_153_when_new_plan_fails(self):
        self.initialize(planning_failed=True)
        original=self.progress.physical_retries
        def groups(*args,**kwargs):
            value=original(*args,**kwargs)
            if kwargs.get('retry_round')==7:value=dict(value,counts=dict(value['counts'],success=1))
            return value
        with patch.object(self.progress,'physical_retries',groups):data=self.progress.physical_execution()
        self.assertEqual(data['combined_latest_counts']['success'],151)
        self.assertEqual(data['current_revision_counts']['success'],153)
        self.assertEqual(data['failure_retry_round13']['counts']['planning_failed'],1)
        self.assertEqual(data['failure_retry_round13']['counts']['failed'],0)

    def test_single_unresolved_cohort_and_deep_prior_success_binding_are_required(self):
        self.initialize(planning_failed=True)
        def checked(previous=None):
            return validate_retained_scene_revision(self.f.root,self.revision_ref,previous or self.previous_ref,
                expected_scene_id=self.sid,current=self.row)
        self.assertTrue(checked()['retained_scene_revision'])
        self.assertEqual(retained13.COHORT_SIZE,1)
        path=self.f.root/self.previous_ref['path'];before=path.read_bytes();document=json.loads(before)
        sibling=deepcopy(document['entries'][0]);sibling['scene_id']='unexpected_extra_scene';document['entries'].append(sibling)
        self.f.write(path,document)
        with self.assertRaisesRegex(ValueError,'single-scene'):checked(dict(self.previous_ref,sha256=sha(path)))
        path.write_bytes(before)
        success_path=self.previous.sibling_result_path;success_bytes=success_path.read_bytes()
        success_path.write_bytes(b'prior round11 native success changed')
        with self.assertRaisesRegex(ValueError,'SHA differs'):checked()
        success_path.write_bytes(success_bytes)

    def test_round12_and_13_bound_artifacts_serve_but_orphans_and_raw_arrays_do_not(self):
        self.terminal();url=self.serve();data=self.progress.physical_execution()
        for field in ('failure_retry_round12','failure_retry_round13'):
            row=data[field]['entries'][0];self.assertFalse(row['stale'],row['reason'])
            with urllib.request.urlopen(url+row['planning_result_url']) as response:self.assertEqual(json.load(response)['scene_id'],self.sid)
        with urllib.request.urlopen(url+data['failure_retry_round13']['entries'][0]['result_url']) as response:self.assertEqual(json.load(response)['execution_status'],'success')
        for prefix in (dashboard.PHYSICAL_RETRY12_PLANNING,dashboard.PHYSICAL_RETRY13_PLANNING):
            orphan=prefix+'scenes/orphan/result.json';self.f.write(orphan,dict(unbound=True))
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(url+'/artifact/'+orphan)
            self.assertEqual(caught.exception.code,404);caught.exception.close()
            self.assertFalse(dashboard.allowed_artifact(prefix+'scenes/'+self.sid+'/trajectory.npz'))
        for prefix in (dashboard.PHYSICAL_RETRY12_DIAGNOSTICS,dashboard.PHYSICAL_RETRY13_DIAGNOSTICS):
            self.assertTrue(dashboard.allowed_artifact(prefix+'scenes/'+self.sid+'/frame_00.jpg'))
            self.assertFalse(dashboard.allowed_artifact(prefix+'scenes/'+self.sid+'/physics_states.npz'))


if __name__=='__main__':unittest.main()
