"""New-layout results must not promote the retained original-layout score."""
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
from scripts.extensions.replicator.scene_revision_review import SOURCE_KEYS,validate_scene_revision
from tests.extensions.replicator import test_package_reviewable_physical_demos as base
from tests.extensions.replicator import test_physical_retry6_dashboard as round6


class Retry7DashboardTests(unittest.TestCase):
    def setUp(self):
        oldwrite=base.ReviewableDemoPackagingTests.write
        def bound_geometry(f,path,value):
            if isinstance(value,dict):
                value=deepcopy(value)
                if set(value)=={'task_id'}:value['obstacle']=dict(position=[0.,0.,0.],yaw_deg=0.)
                if 'composition' in value:value['composition'].setdefault('source_paths_sha256',{})
            return oldwrite(f,path,value)
        previous=round6.Retry6DashboardTests('test_151_prior_successes_gain_only_one_valid_round6_success' if '151' in self._testMethodName else 'test_live_planning_after_two_skips_consumes_no_physical_episode')
        with patch.object(base.ReviewableDemoPackagingTests,'write',bound_geometry):previous.setUp()
        self.addCleanup(previous.doCleanups);self.previous=previous
        self.f=previous.f;self.progress=previous.progress;self.sid=previous.sid
        for key,value in [('PHYSICAL_RETRY7_DEMOS','exports/retry7/'),('PHYSICAL_RETRY7_PLAN','configs/retry7.json'),
            ('PHYSICAL_RETRY7_PLANNING','reports/retry7planning/'),('PHYSICAL_RETRY7_DIAGNOSTICS','reports/retry7diagnostic/'),
            ('PHYSICAL_RETRY7_DETAIL_DIAGNOSTICS','reports/retry7detail/'),('PHYSICAL_RETRY7_DARK_DIAGNOSTICS','reports/retry7dark/'),
            ('PHYSICAL_RETRY7_OVERVIEW_DIAGNOSTICS','reports/retry7overview/'),('PHYSICAL_RETRY7_TOP_DIAGNOSTICS','reports/retry7top/'),('PHYSICAL_RETRY7_TOTAL',1)]:
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)

    def preparation(self,planning_failed=False,revision_ready=True,earlier_actual=False):
        self.previous.initialize(prior_planning_failed=not earlier_actual,planning_failed=True)
        f=self.f;old=self.previous.row
        self.previous_ref=dict(path=dashboard.PHYSICAL_RETRY6_DEMOS+'manifest.json',sha256=sha(f.root/dashboard.PHYSICAL_RETRY6_DEMOS/'manifest.json'))
        self.history=deepcopy(old['prior_physical_attempts'])
        self.newconfig=json.loads((f.root/old['config_path']).read_text());self.newconfig['obstacle']['position']=[.05,.12,0.];self.newconfig['obstacle']['yaw_deg']=-10.
        cp=f.write('configs/new_layout.json',self.newconfig);sp=f.write('reports/new_source.usda',b'new hazard pose only')
        self.config_ref=dict(path=f.rel(cp),sha256=sha(cp));self.scene_ref=dict(path=f.rel(sp),sha256=sha(sp))
        original=json.loads((f.root/old['result_path']).read_text())
        self.revision_doc=dict(kind='hazard_pose_revision',retry_round=7,scene_id=self.sid,task_id=old['task_id'],variant_id=old['variant_id'],
            prior_manifest=self.previous_ref,prior_row_source={k:old[k] for k in SOURCE_KEYS},
            old_config=dict(path=old['config_path'],sha256=old['config_sha256']),new_config=self.config_ref,
            old_source_scene=dict(path=old['source_scene_path'],sha256=old['source_scene_sha256']),new_source_scene=self.scene_ref,
            changed_fields=['obstacle.position','obstacle.yaw_deg'],new_composition=dict(source_paths_sha256=original['composition']['source_paths_sha256'],layout_sha256=sha(cp),scene_sha256=sha(sp)))
        rp=f.write('reports/scene_revision.json',self.revision_doc);self.revision_ref=dict(path=f.rel(rp),sha256=sha(rp))
        validate_scene_revision(f.root,self.revision_ref,expected_scene_id=self.sid)
        self.planning=dict(source_previous_retry_manifest=self.previous_ref,entries=[dict(scene_id=self.sid,status='pending',attempts=0)])
        if not revision_ready:
            f.write(dashboard.PHYSICAL_RETRY7_PLANNING+'progress.json',self.planning);return
        raw=f.write(dashboard.PHYSICAL_RETRY7_PLANNING+'scenes/'+self.sid+'/trajectory.npz',b'round7 raw')
        adjusted=f.write(dashboard.PHYSICAL_RETRY7_PLANNING+'retimed.npz',b'round7 retimed')
        self.candidate={k:old[k] for k in ('scene_id','task_id','variant_id')}
        self.candidate.update(config_path=self.config_ref['path'],config_sha256=self.config_ref['sha256'],scene_revision=self.revision_ref,layout_changed=True,
            composition=self.revision_doc['new_composition'],status='failed' if planning_failed else 'generated',planning_only=True,executed=False,attempts=1,
            trajectory_complete=not planning_failed,artifacts={} if planning_failed else dict(trajectory=dict(path=f.rel(raw),sha256=sha(raw))))
        self.candidate_path=f.write(dashboard.PHYSICAL_RETRY7_PLANNING+'scenes/'+self.sid+'/result.json',self.candidate)
        self.candidate_ref=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path))
        self.planning['entries']=[dict(scene_id=self.sid,status=self.candidate['status'],attempts=1,scene_revision=self.revision_ref,
            candidate_planning_result=self.candidate_ref,trajectory=None if planning_failed else dict(path=f.rel(adjusted),sha256=sha(adjusted)),recipe=dict(description='New human pose'))]
        f.write(dashboard.PHYSICAL_RETRY7_PLANNING+'progress.json',self.planning)
        self.planrow=dict(self.previous.planrow,config_path=self.config_ref['path'],config_sha256=self.config_ref['sha256'],source_scene_path=self.scene_ref['path'],source_scene_sha256=self.scene_ref['sha256'],
            scene_revision=self.revision_ref,layout_changed=True,planning_status=self.candidate['status'],candidate_planning_result=self.candidate_ref,
            previous_retry_result=dict(path=self.history[-1]['result_path'],sha256=self.history[-1]['result_sha256']),physical_attempt_number=len(self.history)+1,
            trajectory_path=None if planning_failed else f.rel(adjusted),trajectory_sha256=None if planning_failed else sha(adjusted),failure_reason='IK_FAIL' if planning_failed else None)

    def initialize(self,planning_failed=False,earlier_actual=False):
        self.preparation(planning_failed,earlier_actual=earlier_actual)
        self.plan=dict(self.previous.plan,retry_round=7,source_previous_retry_manifest=self.previous_ref,layout_changed=True,entries=[self.planrow])
        path=self.f.write(dashboard.PHYSICAL_RETRY7_PLAN,self.plan)
        self.row=dict(self.planrow,retry_schema_version=2,retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),prior_physical_attempts=self.history,
            execution_status='planning_failed' if planning_failed else 'pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(self.previous.manifest,retry_round=7,stage='complete' if planning_failed else 'executing',source_previous_retry_manifest=self.previous_ref,
            retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),entries=[self.row])
        self.save_manifest()

    def save_manifest(self):self.f.write(dashboard.PHYSICAL_RETRY7_DEMOS+'manifest.json',self.manifest)

    def terminal(self,status='success',earlier_actual=False):
        self.initialize(earlier_actual=earlier_actual);f=self.f
        path=f.root/dashboard.PHYSICAL_RETRY7_DEMOS/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'round7 states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=23))
        self.row.update(execution_status=status,executed=True,task_success=status=='success',attempts=1)
        self.result=dict(self.row,kind='physical_failure_retry_execution',status='complete',planning_result_path=self.candidate_ref['path'],planning_result_sha256=self.candidate_ref['sha256'],
            original_planning_result=dict(path=self.row['result_path'],sha256=self.row['result_sha256']),rendering_enabled=False,video_expected=False,
            original_task_success=status=='success',final_original_destination_predicate=status=='success',source_previous_retry_manifest=self.previous_ref,
            source_inputs_unchanged=True,execution_sources_unchanged=True,render_generation_delta=0,render_state_start={'passed':True},render_state_end={'passed':True},
            physics_evidence=dict(q_path=f.rel(states),q_sha256=sha(states),metadata_path=f.rel(meta),metadata_sha256=sha(meta),complete=True,continuous=True))
        f.write(path,self.result);self.row.update(execution_result_path=f.rel(path),execution_result_sha256=sha(path));self.manifest['stage']='complete';self.save_manifest()

    def test_151_original_successes_stay_151_while_current_revision_becomes_152(self):
        self.terminal();data=self.progress.physical_execution();r=data['failure_retry_round7']['entries'][0]
        self.assertFalse(r['stale'],r['reason']);self.assertTrue(r['result_valid']);self.assertEqual(r['config_sha256'],self.config_ref['sha256'])
        self.assertEqual(data['combined_latest_counts']['success'],151);self.assertEqual(data['current_revision_counts']['success'],152)
        self.assertEqual(data['current_revision_counts']['revision_success'],1);self.assertEqual(data['failure_retry_round6']['counts']['planning_failed'],1)
        self.assertEqual(r['physical_attempt_number'],self.previous.row['physical_attempt_number']);self.assertEqual(len(self.history),3)

    def test_planning_failed_consumes_zero_episode_and_does_not_promote_counts(self):
        self.initialize(planning_failed=True);data=self.progress.physical_execution();retry=data['failure_retry_round7'];r=retry['entries'][0]
        self.assertFalse(r['stale'],r['reason']);self.assertEqual(retry['counts']['planning_failed'],1);self.assertEqual(retry['counts']['failed'],0)
        self.assertFalse(r['executed']);self.assertEqual(r['attempts'],0);self.assertFalse(r['result_valid']);self.assertIsNone(r['result_url'])
        self.assertEqual(data['combined_latest_counts']['success'],2);self.assertEqual(data['current_revision_counts']['success'],2)
        self.assertEqual(r['original_config_sha256'],self.previous.row['config_sha256']);self.assertNotEqual(r['config_sha256'],r['original_config_sha256'])

    def test_unfrozen_revision_preparation_is_visible_but_not_executed(self):
        self.preparation(revision_ready=False);data=self.progress.physical_execution();retry=data['failure_retry_round7'];r=retry['entries'][0]
        self.assertTrue(retry['planning_present']);self.assertFalse(retry['manifest_present']);self.assertFalse(r['stale'],r['reason'])
        self.assertTrue(r['revision_preparation_pending']);self.assertEqual(r['execution_status'],'planning_pending');self.assertEqual(r['attempts'],0)
        self.assertEqual(data['current_revision_counts']['success'],2)

    def test_revision_config_or_scene_tamper_withholds_new_success_only(self):
        self.terminal()
        for ref in (self.config_ref,self.scene_ref,self.revision_ref):
            path=self.f.root/ref['path'];before=path.read_bytes();path.write_bytes(b'changed new version')
            data=self.progress.physical_execution()
            with self.subTest(path=ref['path']):
                self.assertEqual(data['combined_latest_counts']['success'],2);self.assertEqual(data['current_revision_counts']['success'],2)
                self.assertEqual(data['failure_retry_round7']['counts']['success'],0);self.assertEqual(data['failure_retry_round7']['counts']['stale'],1)
            path.write_bytes(before)

    def test_forged_planning_failure_episode_is_stale(self):
        self.initialize(planning_failed=True);self.row.update(attempts=1,executed=True);self.save_manifest()
        retry=self.progress.physical_execution()['failure_retry_round7'];self.assertEqual(retry['counts']['stale'],1);self.assertEqual(retry['counts']['failed'],0)

    def test_new_notes_do_not_change_any_prior_round_file(self):
        self.terminal('failed',earlier_actual=True)
        # Round6 had no actual episode, so a stale pre-existing note must remain
        # untouched rather than being silently rebound to the new version.
        previous_paths={n:self.progress.physical_retry_notes_path(n) for n in range(1,7)}
        for n,path in previous_paths.items():
            path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(dict(kind='independent_prior_note_fixture',retry_round=n,entries=[])))
        snapshots={n:p.read_bytes() for n,p in previous_paths.items()}
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress));thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            url=f'http://127.0.0.1:{server.server_port}/api/physical-retry-round7-annotations'
            payload=dict(scene_id=self.sid,note='New-layout annotation',expected_revision=0,config_sha256=self.config_ref['sha256'],result_sha256=self.row['execution_result_sha256'])
            def post(value):return urllib.request.urlopen(urllib.request.Request(url,data=json.dumps(value).encode(),headers={'Content-Type':'application/json'},method='POST'))
            with post(payload) as response:self.assertEqual(json.load(response)['entry']['revision'],1)
            with urllib.request.urlopen(url) as response:self.assertEqual(json.load(response)['entries'][0]['note'],'New-layout annotation')
            with self.assertRaises(urllib.error.HTTPError) as caught:post(dict(payload,expected_revision=1,config_sha256=self.previous.row['config_sha256']))
            self.assertEqual(caught.exception.code,409);caught.exception.close()
            for n,p in previous_paths.items():self.assertEqual(p.read_bytes(),snapshots[n])
        finally:server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
