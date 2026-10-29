"""Round4 provenance, skipped episodes and notes across three prior retries."""
from copy import deepcopy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

from scripts.extensions.replicator import progress_dashboard as dashboard
from scripts.extensions.replicator.package_trajectory_demos import sha
from tests.extensions.replicator import test_package_reviewable_physical_demos as base
from tests.extensions.replicator import test_physical_retry_dashboard as round1
from tests.extensions.replicator import test_physical_retry3_dashboard as round3


def add_prior_successes(f, count):
    """Add genuinely file-bound successes before the retry fixture is built."""
    original=f.first_entries[2]
    planning=json.loads((f.root/original['result_path']).read_text())
    for number in range(count):
        sid=f'prior_success_{number:03d}'
        source=dict(f.sources[2],scene_id=sid)
        planned=dict(planning,scene_id=sid)
        result=f.write(f.source/'scenes'/sid/'result.json',planned)
        row=dict(original,scene_id=sid,result_path=f.rel(result),result_sha256=sha(result),
            execution_result_path=f'executions/{sid}/result.json',video_path=f'videos/{sid}.mp4')
        video=f.write(f.first/row['video_path'],b'file-bound historical video')
        row['video_sha256']=sha(video)
        actual=dict(row,kind='physical_execution',planning_result_path=row['result_path'],
            planning_result_sha256=row['result_sha256'],runtime_settings=f.settings)
        f.bind(f.first,row,actual);f.sources.append(source);f.first_entries.append(row)
    f.write(f.progress,dict(stage='complete',entries=f.sources))
    f.first_manifest['source_progress_sha256']=sha(f.progress)
    f.write(f.first/'manifest.json',f.first_manifest)
    f.retake_manifest['source_progress_sha256']=sha(f.progress);f.save_retake_manifest()


class Retry4DashboardTests(unittest.TestCase):
    def setUp(self):
        previous=round3.Retry3DashboardTests('test_repeated_planning_failure_keeps_last_physical_episode_and_counts')
        self.baseline=148 if '148' in self._testMethodName else 2
        if self.baseline==148:
            original=base.ReviewableDemoPackagingTests.setUp
            def expanded(f):
                original(f);add_prior_successes(f,146)
            with patch.object(base.ReviewableDemoPackagingTests,'setUp',expanded):previous.setUp()
        else:previous.setUp()
        self.addCleanup(previous.doCleanups);self.previous=previous
        self.f=previous.f;self.progress=previous.progress;self.sid=previous.sid
        for key,value in [('PHYSICAL_RETRY4_DEMOS','exports/retry4/'),('PHYSICAL_RETRY4_PLAN','configs/retry4.json'),
                          ('PHYSICAL_RETRY4_PLANNING','reports/retry4planning/'),('PHYSICAL_RETRY4_DIAGNOSTICS','reports/retry4diagnostic/'),
                          ('PHYSICAL_RETRY4_DETAIL_DIAGNOSTICS','reports/retry4detail/'),('PHYSICAL_RETRY4_DARK_DIAGNOSTICS','reports/retry4dark/'),
                          ('PHYSICAL_RETRY4_OVERVIEW_DIAGNOSTICS','reports/retry4overview/'),('PHYSICAL_RETRY4_TOP_DIAGNOSTICS','reports/retry4top/'),
                          ('PHYSICAL_RETRY4_TOTAL',1)]:
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)

    def preparation(self,prior_planning_failed=True,planning_failed=False):
        if prior_planning_failed:self.previous.initialize(planning_failed=True)
        else:self.previous.terminal('failed',previous_planning_failed=False)
        f=self.f;old=self.previous.row
        self.previous_ref=dict(path=dashboard.PHYSICAL_RETRY3_DEMOS+'manifest.json',
            sha256=sha(f.root/dashboard.PHYSICAL_RETRY3_DEMOS/'manifest.json'))
        self.history=deepcopy(old['prior_physical_attempts'])
        if old['executed']:
            self.history.append(dict(result_path=old['execution_result_path'],result_sha256=old['execution_result_sha256'],
                physical_attempt_number=old['physical_attempt_number'],execution_status='failed',task_success=False,executed=True,video_sha256=None))
        raw=f.write(dashboard.PHYSICAL_RETRY4_PLANNING+'scenes/'+self.sid+'/trajectory.npz',b'round4 raw trajectory')
        adjusted=f.write(dashboard.PHYSICAL_RETRY4_PLANNING+'retimed.npz',b'round4 retimed trajectory')
        self.candidate={k:old[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256')}
        self.candidate.update(status='failed' if planning_failed else 'generated',planning_only=True,executed=False,attempts=1,
            trajectory_complete=not planning_failed,artifacts={} if planning_failed else dict(trajectory=dict(path=f.rel(raw),sha256=sha(raw))))
        self.candidate_path=f.write(dashboard.PHYSICAL_RETRY4_PLANNING+'scenes/'+self.sid+'/result.json',self.candidate)
        self.candidate_ref=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path))
        self.planning=dict(source_previous_retry_manifest=self.previous_ref,entries=[dict(scene_id=self.sid,status=self.candidate['status'],
            candidate_planning_result=self.candidate_ref,trajectory=None if planning_failed else dict(path=f.rel(adjusted),sha256=sha(adjusted)),
            recipe={'description':'One frozen round4 candidate'})])
        f.write(dashboard.PHYSICAL_RETRY4_PLANNING+'progress.json',self.planning)
        self.planrow=dict(self.previous.planrow,planning_status=self.candidate['status'],candidate_planning_result=self.candidate_ref,
            previous_retry_result=dict(path=self.history[-1]['result_path'],sha256=self.history[-1]['result_sha256']),
            physical_attempt_number=len(self.history)+1,trajectory_path=None if planning_failed else f.rel(adjusted),
            trajectory_sha256=None if planning_failed else sha(adjusted),failure_reason='SEGMENT_DEVIATION_LIMIT' if planning_failed else None)

    def initialize(self,prior_planning_failed=True,planning_failed=False):
        self.preparation(prior_planning_failed,planning_failed)
        self.plan=dict(self.previous.plan,retry_round=4,source_previous_retry_manifest=self.previous_ref,entries=[self.planrow])
        path=self.f.write(dashboard.PHYSICAL_RETRY4_PLAN,self.plan)
        self.row=dict(self.planrow,retry_schema_version=2,retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),
            prior_physical_attempts=self.history,execution_status='planning_failed' if planning_failed else 'pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(self.previous.manifest,retry_round=4,stage='complete' if planning_failed else 'executing',
            source_previous_retry_manifest=self.previous_ref,retry_plan_path=self.f.rel(path),retry_plan_sha256=sha(path),entries=[self.row])
        self.save_manifest()

    def save_manifest(self):self.f.write(dashboard.PHYSICAL_RETRY4_DEMOS+'manifest.json',self.manifest)

    def terminal(self,status='success',prior_planning_failed=True):
        self.initialize(prior_planning_failed);f=self.f
        path=f.root/dashboard.PHYSICAL_RETRY4_DEMOS/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'round4 native states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=23))
        self.row.update(execution_status=status,executed=True,task_success=status=='success',attempts=1)
        self.result=dict(self.row,kind='physical_failure_retry_execution',status='complete',planning_result_path=self.candidate_ref['path'],
            planning_result_sha256=self.candidate_ref['sha256'],original_planning_result=dict(path=self.row['result_path'],sha256=self.row['result_sha256']),
            rendering_enabled=False,video_expected=False,original_task_success=status=='success',final_original_destination_predicate=status=='success',
            source_previous_retry_manifest=self.previous_ref,source_inputs_unchanged=True,execution_sources_unchanged=True,
            render_generation_delta=0,render_state_start={'passed':True},render_state_end={'passed':True},
            physics_evidence=dict(q_path=f.rel(states),q_sha256=sha(states),metadata_path=f.rel(meta),metadata_sha256=sha(meta),complete=True,continuous=True),
            physical_overlay_binding=self.previous.previous.old.result['physical_overlay_binding'])
        f.write(path,self.result);self.row.update(execution_result_path=f.rel(path),execution_result_sha256=sha(path))
        self.manifest['stage']='complete';self.save_manifest()

    def test_live_planning_after_two_skips_consumes_no_physical_episode(self):
        self.preparation();data=self.progress.physical_execution();fourth=data['failure_retry_round4'];row=fourth['entries'][0]
        self.assertFalse(fourth['manifest_present']);self.assertTrue(fourth['planning_present'])
        self.assertEqual(fourth['counts']['stale'],0);self.assertEqual(row['attempts'],0);self.assertFalse(row['executed'])
        self.assertEqual(row['physical_attempt_number'],self.previous.row['physical_attempt_number'])
        self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_planning_failure_is_zero_episode_and_preserves_latest_actual(self):
        self.initialize(planning_failed=True);data=self.progress.physical_execution();fourth=data['failure_retry_round4'];row=fourth['entries'][0]
        self.assertEqual(fourth['counts']['planning_failed'],1);self.assertEqual(fourth['counts']['failed'],0)
        self.assertEqual(fourth['counts']['stale'],0);self.assertFalse(row['result_valid']);self.assertIsNone(row['result_url'])
        self.assertEqual(row['attempts'],0);self.assertFalse(row['executed']);self.assertEqual(len(self.history),3)
        self.assertEqual(self.row['previous_retry_result']['path'],self.previous.previous.old.row['execution_result_path'])
        self.assertEqual(data['combined_latest_counts']['success'],2)
        self.assertEqual(data['failure_retry_round2']['counts']['planning_failed'],1)
        self.assertEqual(data['failure_retry_round3']['counts']['planning_failed'],1)
        with self.assertRaises(dashboard.AnnotationError):
            self.progress.save_physical_retry_annotation(dict(scene_id=self.sid,note='No actual result',expected_revision=0,
                config_sha256=self.row['config_sha256'],result_sha256=self.candidate_ref['sha256']),4)

    def test_148_prior_successes_gain_only_one_valid_round4_success(self):
        self.terminal();data=self.progress.physical_execution();row=data['failure_retry_round4']['entries'][0]
        self.assertEqual(data['counts']['success'],148);self.assertEqual(data['combined_latest_counts']['success'],149)
        self.assertEqual(data['combined_latest_counts']['failed'],0);self.assertEqual(data['combined_latest_counts']['total'],149)
        self.assertEqual(row['physical_attempt_number'],4);self.assertTrue(row['result_valid']);self.assertEqual(row['attempts'],1)
        self.assertEqual(len(self.history),3);self.assertEqual(data['camera_retakes']['saved'],2)
        self.assertEqual(self.progress.physical_execution()['combined_latest_counts']['success'],149)

    def test_round4_success_includes_actual_round3_failure_in_history(self):
        self.terminal(prior_planning_failed=False);data=self.progress.physical_execution();row=data['failure_retry_round4']['entries'][0]
        self.assertTrue(row['result_valid']);self.assertEqual(row['physical_attempt_number'],6);self.assertEqual(len(self.history),5)
        self.assertEqual(self.row['previous_retry_result']['path'],self.previous.row['execution_result_path'])
        self.assertEqual(data['combined_latest_counts']['success'],3)

    def test_stale_current_or_earlier_actual_evidence_withholds_success(self):
        self.terminal(prior_planning_failed=False)
        for path in (self.candidate_path,self.f.root/self.row['execution_result_path'],self.f.root/self.previous.row['execution_result_path']):
            original=path.read_bytes();path.write_bytes(b'changed bound evidence')
            data=self.progress.physical_execution()
            with self.subTest(path=path.name):
                self.assertEqual(data['failure_retry_round4']['counts']['success'],0)
                self.assertEqual(data['failure_retry_round4']['counts']['stale'],1)
                self.assertEqual(data['combined_latest_counts']['success'],2)
            path.write_bytes(original)

    def test_forged_physical_attempt_on_planning_failure_is_stale(self):
        self.initialize(planning_failed=True);self.row['attempts']=1;self.row['executed']=True;self.save_manifest()
        data=self.progress.physical_execution();self.assertEqual(data['failure_retry_round4']['counts']['stale'],1)
        self.assertEqual(data['failure_retry_round4']['counts']['planning_failed'],0)
        self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_round4_http_notes_are_isolated_from_all_three_prior_rounds(self):
        self.terminal('failed',prior_planning_failed=False)
        payload=dict(scene_id=self.sid,note='',expected_revision=0,config_sha256=self.row['config_sha256'],result_sha256='')
        fixtures={1:self.previous.previous.old,2:self.previous.previous,3:self.previous}
        for number,fixture in fixtures.items():
            payload.update(note=f'round{number} saved',result_sha256=fixture.row['execution_result_sha256'])
            self.progress.save_physical_retry_annotation(payload,number)
        originals={n:self.progress.physical_retry_notes_path(n).read_bytes() for n in fixtures}
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            url=f'http://127.0.0.1:{server.server_port}/api/physical-retry-round4-annotations'
            payload.update(note='round4 independent',result_sha256=self.row['execution_result_sha256'])
            def post(value):return urllib.request.urlopen(urllib.request.Request(url,data=json.dumps(value).encode(),headers={'Content-Type':'application/json'},method='POST'))
            with post(payload) as response:self.assertEqual(json.load(response)['entry']['revision'],1)
            with urllib.request.urlopen(url) as response:self.assertEqual(json.load(response)['entries'][0]['note'],'round4 independent')
            for bad in (payload,dict(payload,expected_revision=1,result_sha256=self.previous.row['execution_result_sha256'])):
                with self.assertRaises(urllib.error.HTTPError) as caught:post(bad)
                self.assertEqual(caught.exception.code,409);caught.exception.close()
            self.assertEqual(self.progress.physical_retry_annotations(4)['entries'][0]['revision'],1)
            for n,content in originals.items():self.assertEqual(self.progress.physical_retry_notes_path(n).read_bytes(),content)
        finally:server.shutdown();server.server_close();thread.join()

    def test_round4_diagnostic_image_becomes_unavailable_after_source_changes(self):
        self.terminal('failed');self.diagnostics=self.f.root/dashboard.PHYSICAL_RETRY4_DIAGNOSTICS
        round1.PhysicalRetryDashboardTests.diagnostic(self)
        row=self.progress.physical_execution()['failure_retry_round4']['entries'][0]
        self.assertEqual(len(row['diagnostics']),1)
        image=self.f.root/self.report['frames'][0]['image_path'];image.write_bytes(b'changed pixels')
        row=self.progress.physical_execution()['failure_retry_round4']['entries'][0]
        self.assertEqual(row['diagnostics'],[]);self.assertTrue(row['diagnostic_error']);self.assertTrue(row['result_valid'])


if __name__=='__main__':unittest.main()
