"""Third-round UI provenance across a skipped physical episode and old notes."""
from copy import deepcopy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
import unittest
import urllib.request
import urllib.error
from unittest.mock import patch

from scripts.extensions.replicator import progress_dashboard as dashboard
from scripts.extensions.replicator.package_trajectory_demos import sha
from tests.extensions.replicator import test_physical_retry_dashboard as round1
from tests.extensions.replicator import test_physical_retry2_dashboard as round2


class Retry3DashboardTests(unittest.TestCase):
    def setUp(self):
        fixture=round2.Retry2DashboardTests('test_planning_failed_is_neither_physical_failure_nor_attempt')
        fixture.setUp();self.addCleanup(fixture.doCleanups);self.previous=fixture
        self.f=fixture.f;self.progress=fixture.progress;self.sid=fixture.sid
        for key,value in [('PHYSICAL_RETRY3_DEMOS','exports/retry3/'),('PHYSICAL_RETRY3_PLAN','configs/retry3.json'),
                          ('PHYSICAL_RETRY3_PLANNING','reports/retry3planning/'),('PHYSICAL_RETRY3_DIAGNOSTICS','reports/retry3diagnostic/'),
                          ('PHYSICAL_RETRY3_DETAIL_DIAGNOSTICS','reports/retry3detail/'),('PHYSICAL_RETRY3_DARK_DIAGNOSTICS','reports/retry3dark/'),
                          ('PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS','reports/retry3overview/'),('PHYSICAL_RETRY3_TOP_DIAGNOSTICS','reports/retry3top/'),
                          ('PHYSICAL_RETRY3_TOTAL',1)]:
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)

    def preparation(self,previous_planning_failed=True,planning_failed=False):
        if previous_planning_failed:self.previous.initialize(failed=True)
        else:self.previous.terminal('failed')
        f=self.f;old=self.previous.row
        self.previous_ref=dict(path=dashboard.PHYSICAL_RETRY2_DEMOS+'manifest.json',sha256=sha(f.root/dashboard.PHYSICAL_RETRY2_DEMOS/'manifest.json'))
        self.history=deepcopy(old['prior_physical_attempts'])
        if old['executed']:
            self.history.append(dict(result_path=old['execution_result_path'],result_sha256=old['execution_result_sha256'],
                physical_attempt_number=old['physical_attempt_number'],execution_status='failed',task_success=False,executed=True,video_sha256=None))
        raw=f.write(dashboard.PHYSICAL_RETRY3_PLANNING+'scenes/'+self.sid+'/trajectory.npz',b'third-round raw trajectory')
        adjusted=f.write(dashboard.PHYSICAL_RETRY3_PLANNING+'retimed.npz',b'third-round retimed trajectory')
        self.candidate={k:old[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256')}
        self.candidate.update(status='failed' if planning_failed else 'generated',planning_only=True,executed=False,attempts=1,
            trajectory_complete=not planning_failed,artifacts={} if planning_failed else dict(trajectory=dict(path=f.rel(raw),sha256=sha(raw))))
        self.candidate_path=f.write(dashboard.PHYSICAL_RETRY3_PLANNING+'scenes/'+self.sid+'/result.json',self.candidate)
        self.candidate_ref=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path))
        self.planning=dict(source_previous_retry_manifest=self.previous_ref,entries=[dict(scene_id=self.sid,
            status=self.candidate['status'],candidate_planning_result=self.candidate_ref,
            trajectory=None if planning_failed else dict(path=f.rel(adjusted),sha256=sha(adjusted)),recipe={'description':'One third-round candidate'})])
        f.write(dashboard.PHYSICAL_RETRY3_PLANNING+'progress.json',self.planning)
        self.planrow=dict(self.previous.planrow,planning_status=self.candidate['status'],candidate_planning_result=self.candidate_ref,
            previous_retry_result=dict(path=self.history[-1]['result_path'],sha256=self.history[-1]['result_sha256']),
            physical_attempt_number=len(self.history)+1,trajectory_path=None if planning_failed else f.rel(adjusted),
            trajectory_sha256=None if planning_failed else sha(adjusted),failure_reason='IK_FAIL' if planning_failed else None)

    def initialize(self,previous_planning_failed=True,planning_failed=False):
        self.preparation(previous_planning_failed,planning_failed)
        self.plan=dict(self.previous.plan,retry_round=3,source_previous_retry_manifest=self.previous_ref,entries=[self.planrow])
        plan_path=self.f.write(dashboard.PHYSICAL_RETRY3_PLAN,self.plan)
        self.row=dict(self.planrow,retry_schema_version=2,retry_plan_path=self.f.rel(plan_path),retry_plan_sha256=sha(plan_path),
            prior_physical_attempts=self.history,execution_status='planning_failed' if planning_failed else 'pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(self.previous.manifest,retry_round=3,stage='complete' if planning_failed else 'executing',
            source_previous_retry_manifest=self.previous_ref,retry_plan_path=self.f.rel(plan_path),retry_plan_sha256=sha(plan_path),entries=[self.row])
        self.save_manifest()

    def save_manifest(self):self.f.write(dashboard.PHYSICAL_RETRY3_DEMOS+'manifest.json',self.manifest)

    def terminal(self,status='success',previous_planning_failed=True):
        self.initialize(previous_planning_failed);f=self.f
        path=f.root/dashboard.PHYSICAL_RETRY3_DEMOS/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'third-round measured states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=22))
        self.row.update(execution_status=status,executed=True,task_success=status=='success',attempts=1)
        self.result=dict(self.row,kind='physical_failure_retry_execution',status='complete',planning_result_path=self.candidate_ref['path'],
            planning_result_sha256=self.candidate_ref['sha256'],original_planning_result=dict(path=self.row['result_path'],sha256=self.row['result_sha256']),
            rendering_enabled=False,video_expected=False,original_task_success=status=='success',final_original_destination_predicate=status=='success',
            source_previous_retry_manifest=self.previous_ref,source_inputs_unchanged=True,execution_sources_unchanged=True,
            render_generation_delta=0,render_state_start={'passed':True},render_state_end={'passed':True},
            physics_evidence=dict(q_path=f.rel(states),q_sha256=sha(states),metadata_path=f.rel(meta),metadata_sha256=sha(meta),complete=True,continuous=True),
            physical_overlay_binding=self.previous.old.result['physical_overlay_binding'])
        f.write(path,self.result);self.row.update(execution_result_path=f.rel(path),execution_result_sha256=sha(path));self.manifest['stage']='complete';self.save_manifest()

    def publication(self,retry):
        self.published=dict(kind='physical_failure_retry_result_snapshot',partial_preview=False,retry_round=3,
            source_retry_manifest=dict(path=dashboard.PHYSICAL_RETRY3_DEMOS+'manifest.json',sha256=sha(self.f.root/dashboard.PHYSICAL_RETRY3_DEMOS/'manifest.json')),
            diagnostic_visual_review=dict(status='complete',conclusions_written=True),entries=[dict(scene_id=self.sid,retry_round3=retry)])
        self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published)

    def visual(self):
        self.terminal('failed');self.diagnostics=self.f.root/dashboard.PHYSICAL_RETRY3_DIAGNOSTICS
        round1.PhysicalRetryDashboardTests.diagnostic(self)
        review=dict(scene_id=self.sid,status='complete',execution_result_sha256=self.row['execution_result_sha256'],
            diagnostic_report_sha256=sha(self.diagnostics/'scenes'/self.sid/'diagnostic.json'),observed_visual_facts_zh=['本轮碗仍在槽外。'],
            interpretation_zh='可能提前失持，根因未确认。',next_adjustment_zh='下轮检查夹持。',root_cause_confirmed=False,interpretation_is_inference=True)
        self.publication(dict(result=dict(path=self.row['execution_result_path'],sha256=self.row['execution_result_sha256']),diagnostic_visual_review=review))

    def test_planning_progress_after_previous_planning_failure_consumes_no_episode(self):
        self.preparation();data=self.progress.physical_execution();third=data['failure_retry_round3'];row=third['entries'][0]
        self.assertFalse(third['manifest_present']);self.assertTrue(third['planning_present'])
        self.assertEqual(third['planning']['counts']['generated'],1);self.assertEqual(third['counts']['stale'],0)
        self.assertEqual(row['physical_attempt_number'],self.previous.row['physical_attempt_number'])
        self.assertEqual(row['attempts'],0);self.assertFalse(row['executed']);self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_repeated_planning_failure_keeps_last_physical_episode_and_counts(self):
        self.initialize(planning_failed=True);data=self.progress.physical_execution();third=data['failure_retry_round3'];row=third['entries'][0]
        self.assertEqual(third['counts']['planning_failed'],1);self.assertEqual(third['counts']['failed'],0);self.assertEqual(third['counts']['stale'],0)
        self.assertEqual(row['attempts'],0);self.assertFalse(row['executed']);self.assertFalse(row['result_valid']);self.assertIsNone(row['result_url'])
        self.assertEqual(self.row['previous_retry_result']['path'],self.previous.old.row['execution_result_path'])
        self.assertEqual(len(self.row['prior_physical_attempts']),3);self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_success_after_skipped_episode_keeps_complete_history_and_promotes_once(self):
        self.terminal();data=self.progress.physical_execution();row=data['failure_retry_round3']['entries'][0]
        self.assertTrue(row['result_valid']);self.assertEqual(row['physical_attempt_number'],4);self.assertEqual(row['attempts'],1)
        self.assertEqual(len(self.result['prior_physical_attempts']),3);self.assertEqual(data['combined_latest_counts']['success'],3)
        self.assertEqual(data['failure_retry_round2']['counts']['planning_failed'],1);self.assertEqual(data['counts']['success'],2)
        self.assertEqual(data['camera_retakes']['saved'],2)

    def test_success_after_actual_previous_failure_includes_that_fourth_episode(self):
        self.terminal(previous_planning_failed=False);data=self.progress.physical_execution();row=data['failure_retry_round3']['entries'][0]
        self.assertTrue(row['result_valid']);self.assertEqual(row['physical_attempt_number'],5)
        self.assertEqual(len(self.result['prior_physical_attempts']),4)
        self.assertEqual(self.row['previous_retry_result']['path'],self.previous.row['execution_result_path'])
        self.assertEqual(data['failure_retry_round2']['counts']['failed'],1);self.assertEqual(data['combined_latest_counts']['success'],3)

    def test_planning_failure_cannot_be_forged_into_physical_history(self):
        self.initialize();self.row['prior_physical_attempts'].append(dict(result_path=self.f.rel(self.previous.candidate_path),
            result_sha256=sha(self.previous.candidate_path),physical_attempt_number=4,executed=True,execution_status='failed',task_success=False,video_sha256=None))
        self.save_manifest();data=self.progress.physical_execution()
        self.assertEqual(data['failure_retry_round3']['counts']['stale'],1);self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_earlier_physical_history_tampering_withholds_new_success(self):
        self.terminal();(self.f.root/self.previous.old.row['execution_result_path']).write_text('{}')
        data=self.progress.physical_execution()
        self.assertEqual(data['failure_retry_round3']['counts']['success'],0);self.assertEqual(data['failure_retry_round3']['counts']['stale'],1)
        self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_round3_annotations_http_are_independent_and_revision_bound(self):
        self.terminal('failed',previous_planning_failed=False)
        payload=dict(scene_id=self.sid,note='round1 saved',expected_revision=0,config_sha256=self.row['config_sha256'],
            result_sha256=self.previous.old.row['execution_result_sha256'])
        self.progress.save_physical_retry_annotation(payload,1)
        payload.update(note='round2 saved',result_sha256=self.previous.row['execution_result_sha256']);self.progress.save_physical_retry_annotation(payload,2)
        originals={n:self.progress.physical_retry_notes_path(n).read_bytes() for n in (1,2)}
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress));thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            url='http://127.0.0.1:'+str(server.server_port)+'/api/physical-retry-round3-annotations'
            payload.update(note='round3 independent',result_sha256=self.row['execution_result_sha256'])
            request=urllib.request.Request(url,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
            with urllib.request.urlopen(request) as response:self.assertEqual(json.load(response)['entry']['revision'],1)
            with urllib.request.urlopen(url) as response:self.assertEqual(json.load(response)['entries'][0]['note'],'round3 independent')
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(request)
            self.assertEqual(caught.exception.code,409);caught.exception.close()
            for n,content in originals.items():self.assertEqual(self.progress.physical_retry_notes_path(n).read_bytes(),content)
        finally:server.shutdown();server.server_close();thread.join()

    def test_round3_supplements_accept_own_round_and_reject_round2(self):
        self.visual();review=self.published['entries'][0]['retry_round3']['diagnostic_visual_review']
        for prefix,accepted in [(dashboard.PHYSICAL_RETRY3_DETAIL_DIAGNOSTICS,True),(dashboard.PHYSICAL_RETRY3_DARK_DIAGNOSTICS+'attempt_01/',True),
                                (dashboard.PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS,True),(dashboard.PHYSICAL_RETRY3_TOP_DIAGNOSTICS,True),
                                (dashboard.PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,False)]:
            directory=Path(prefix)/'scenes'/self.sid;image=self.f.write(directory/'frame_00.jpg',b'third-round supplementary')
            extra=deepcopy(self.report);extra.update(status='ready',timeline_playing=False);extra['frames'][0].update(image_path=self.f.rel(image),image_sha256=sha(image))
            report=self.f.write(directory/'diagnostic.json',extra);review['supplementary_diagnostic_reports']=[dict(path=self.f.rel(report),sha256=sha(report))]
            self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published);row=self.progress.physical_execution()['failure_retry_round3']['entries'][0]
            with self.subTest(prefix=prefix):
                self.assertEqual(len(row['diagnostics']),2 if accepted else 1);self.assertEqual('diagnostic_visual_review' in row,accepted)

    def test_round3_overview_top_images_are_labeled_and_hash_bound_over_http(self):
        self.visual();review=self.published['entries'][0]['retry_round3']['diagnostic_visual_review']
        records=[]
        for prefix,top in ((dashboard.PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS,False),(dashboard.PHYSICAL_RETRY3_TOP_DIAGNOSTICS,True)):
            directory=Path(prefix)/'scenes'/self.sid
            image=self.f.write(directory/'frame_00.jpg',b'bound overview/top actual-state image')
            extra=deepcopy(self.report);extra.update(status='ready',timeline_playing=False,
                observation_policy=dict(detail_view=False,top_view=top,exposure_ev=-1 if top else 0))
            extra['frames'][0].update(image_path=self.f.rel(image),image_sha256=sha(image))
            path=self.f.write(directory/'diagnostic.json',extra)
            records.append((dict(path=self.f.rel(path),sha256=sha(path)),image,extra))
        review['supplementary_diagnostic_reports']=[record[0] for record in records]
        self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published)
        row=self.progress.physical_execution()['failure_retry_round3']['entries'][0]
        self.assertIn('diagnostic_visual_review',row);self.assertEqual(len(row['diagnostics']),3)
        self.assertEqual([f['observation_label'] for f in row['diagnostics'][1:]],['总览补图 · 请求曝光 0EV','俯视补图 · 请求曝光 -1EV'])
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            base='http://127.0.0.1:'+str(server.server_port)+'/artifact/'
            for reference,image,extra in records:
                with self.subTest(image=self.f.rel(image)):
                    self.assertTrue(dashboard.allowed_artifact(self.f.rel(image)))
                    with urllib.request.urlopen(base+self.f.rel(image)) as response:self.assertEqual(response.status,200)
                    for tail in ('secrets.jpg','scenes/'+self.sid+'/physics_states.npz','scenes/'+self.sid+'/diagnostic.json'):
                        self.assertFalse(dashboard.allowed_artifact(image.parents[2].relative_to(self.f.root).as_posix()+'/'+tail))
                    unbound=self.f.write(image.with_name('frame_01.jpg'),b'unbound frame')
                    with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+self.f.rel(unbound))
                    self.assertEqual(caught.exception.code,404);caught.exception.close()
                    original=image.read_bytes();image.write_bytes(b'changed image')
                    with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+self.f.rel(image))
                    self.assertEqual(caught.exception.code,404);caught.exception.close()
                    self.assertNotIn('diagnostic_visual_review',self.progress.physical_execution()['failure_retry_round3']['entries'][0])
                    image.write_bytes(original)
                    # Rehashing a changed report cannot make a foreign episode
                    # or different recorded state into a valid supplemental view.
                    for field in ('binding','frame'):
                        changed=deepcopy(extra)
                        if field=='binding':changed['binding']['execution_result_sha256']='0'*64
                        else:changed['frames'][0]['state_index']+=1
                        path=self.f.write(reference['path'],changed);reference['sha256']=sha(path)
                        self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published)
                        with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+self.f.rel(image))
                        self.assertEqual(caught.exception.code,404);caught.exception.close()
                    path=self.f.write(reference['path'],extra);reference['sha256']=sha(path)
                    self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published)
                    with urllib.request.urlopen(base+self.f.rel(image)) as response:self.assertEqual(response.status,200)
        finally:server.shutdown();server.server_close();thread.join()

    def test_round3_new_view_directories_are_rejected_by_both_earlier_rounds(self):
        self.previous.publish_visual();old=self.previous.old;old.publish_visual_review()
        for fixture,number,publication,path,key in (
                (old,1,old.published,dashboard.PHYSICAL_RETRY_RESULTS,'retry'),
                (self.previous,2,self.previous.published,dashboard.PHYSICAL_RETRY2_RESULTS,'retry_round2')):
            for prefix in (dashboard.PHYSICAL_RETRY3_OVERVIEW_DIAGNOSTICS,dashboard.PHYSICAL_RETRY3_TOP_DIAGNOSTICS):
                reference,_=self.previous.supplementary(fixture,prefix)
                publication['entries'][0][key]['diagnostic_visual_review']['supplementary_diagnostic_reports']=[reference]
                self.f.write(path,publication)
                field='failure_retry' if number==1 else 'failure_retry_round2'
                row=self.progress.physical_execution()[field]['entries'][0]
                with self.subTest(retry_round=number,prefix=prefix):
                    self.assertEqual(len(row['diagnostics']),1);self.assertNotIn('diagnostic_visual_review',row)

    def test_round3_http_artifacts_are_bound_and_do_not_expose_native_state(self):
        self.visual();row=self.progress.physical_execution()['failure_retry_round3']['entries'][0]
        urls=[row['result_url'],row['planning_result_url'],row['diagnostic_report_url'],row['diagnostics'][0]['url']]
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress));thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            base='http://127.0.0.1:'+str(server.server_port)
            for url in urls:
                with urllib.request.urlopen(base+url) as response:self.assertEqual(response.status,200)
            self.assertFalse(dashboard.allowed_artifact(dashboard.PHYSICAL_RETRY3_DEMOS+'executions/'+self.sid+'/physics_states.npz'))
            self.assertFalse(dashboard.allowed_artifact(dashboard.PHYSICAL_RETRY3_DIAGNOSTICS+'secrets.jpg'))
            self.candidate_path.write_text('{}')
            for url in urls:
                with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+url)
                self.assertEqual(caught.exception.code,404);caught.exception.close()
        finally:server.shutdown();server.server_close();thread.join()

    def test_round3_cpu_diagnostic_binds_current_candidate_and_round(self):
        self.initialize(planning_failed=True)
        diagnostic=dict(scene_id=self.sid,status='diagnosed',candidate_planning_result=self.candidate_ref,
            root_cause_confirmed=False,scene_infeasibility_established=False,failed_phase='lift',solver_status='IK_FAIL',
            observed_cpu_facts_zh=['第三轮固定端点存在球模型近邻。'],interpretation_zh='尚未确认完整任务不可行。',next_adjustment_zh='下一轮检查腕部姿态。')
        source=dict(kind='physical_retry_planning_failure_cpu_diagnostics',retry_round=3,complete=True,
            physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,
            layout_changed=False,source_plan_unchanged=True,source_plan=dict(path=self.manifest['retry_plan_path'],sha256=self.manifest['retry_plan_sha256']),
            entries=[deepcopy(diagnostic)])
        source_path=self.f.write('reports/replicator_physical_retry_round3_20261029/planning_failure_diagnostics.json',source)
        evidence=dict(path=self.f.rel(source_path),sha256=sha(source_path));diagnostic['evidence']=evidence
        self.publication(dict(execution_status='planning_failed',planning_status='failed',attempts=0,executed=False,
            candidate_planning_result=self.candidate_ref,planning_failure_diagnostic=diagnostic))
        self.published['source_planning_failure_diagnostics']=evidence
        self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published)
        row=self.progress.physical_execution()['failure_retry_round3']['entries'][0]
        self.assertIn('planning_failure_diagnostic',row);self.assertNotIn('diagnostic_visual_review',row)
        self.assertEqual(row['attempts'],0);self.assertFalse(row['result_valid'])
        # A report explicitly from round 2 is rejected, even when rehashed and
        # copied into the same publication, rather than silently mislabelled.
        source['retry_round']=2;self.f.write(source_path,source);evidence['sha256']=sha(source_path)
        self.f.write(dashboard.PHYSICAL_RETRY3_RESULTS,self.published)
        self.assertNotIn('planning_failure_diagnostic',self.progress.physical_execution()['failure_retry_round3']['entries'][0])


if __name__=='__main__':unittest.main()
