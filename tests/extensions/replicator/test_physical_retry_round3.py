"""A planning failure must not become an extra physical episode next round."""
import copy
import unittest
from kitchen_safety.extensions.replicator import retry_physical_execution as run
from tests.extensions.replicator.test_grasp_retry_execution import GraspRetryTests


class Round3HistoryTests(unittest.TestCase):
    def setUp(self):
        self.f=GraspRetryTests('test_new_grasp_and_closure_accept_without_changing_start_or_history')
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        f=self.f
        for entry in f.plan['entries'][10:29]:
            candidate=run.bound_json(entry['candidate_planning_result'])
            candidate.update(status='failed',trajectory_complete=False,artifacts={})
            entry['candidate_planning_result']=f.write(entry['candidate_planning_result']['path'],candidate)
            entry.update(planning_status='failed',failure_reason='IK_FAIL',trajectory_path=None,trajectory_sha256=None)
        f.save_plan();_,rows=run.validate_plan(f.planpath)
        previous=dict(kind='physical_failure_retry',schema_version=2,retry_round=2,stage='complete',
            retry_plan_path=str(f.planpath.relative_to(f.root)),retry_plan_sha256=run.sha(f.planpath),
            source_first_pass_manifest=f.plan['source_first_pass_manifest'],
            source_camera_retake_manifest=f.plan['source_camera_retake_manifest'],entries=rows)
        for i,row in enumerate(rows):
            if 10<=i<29:continue
            row.update(executed=True,attempts=1,execution_status='failed' if i<10 else 'success',task_success=i>=29)
            result=dict(row,kind='physical_failure_retry_execution',status='complete',
                **run.planning_result_bindings(row),source_inputs_unchanged=True,execution_sources_unchanged=True,
                rendering_enabled=False,render_generation_delta=0,render_state_start={'passed':True},
                render_state_end={'passed':True},runtime_state_setters_used=False,physical_weld_used=False,
                payload_attachment_used=False,execution_source_sha256_start={'predicate.py':f.predicate['sha256']})
            binding=f.write(f'round2/executions/{row["scene_id"]}/result.json',result)
            row.update(execution_result_path=binding['path'],execution_result_sha256=binding['sha256'])
        self.plan=copy.deepcopy(f.plan)
        self.plan.update(retry_round=3,expected_scene_count=29,
            source_previous_retry_manifest=f.write('round2/manifest.json',previous),entries=[])
        for old in rows[:29]:
            entry=copy.deepcopy(next(r for r in f.plan['entries'] if r['scene_id']==old['scene_id']))
            last=old if old['executed'] else dict(execution_result_path=old['prior_physical_attempts'][-1]['result_path'],
                execution_result_sha256=old['prior_physical_attempts'][-1]['result_sha256'])
            entry.update(previous_retry_result=dict(path=last['execution_result_path'],sha256=last['execution_result_sha256']),
                physical_attempt_number=old['physical_attempt_number']+int(old['executed']))
            self.plan['entries'].append(entry)
        self.path=f.root/'round3_plan.json';self.save()

    def save(self):self.f.write(str(self.path.relative_to(self.f.root)),self.plan)

    def test_physical_history_increments_only_for_executed_rows(self):
        _,rows=run.validate_plan(self.path)
        self.assertEqual(len(rows),29)
        self.assertEqual(rows[0]['physical_attempt_number'],5)  # Includes the retained camera episode.
        self.assertEqual(rows[10]['physical_attempt_number'],3) # Round2 planning did not execute.
        self.assertEqual(len(rows[10]['prior_physical_attempts']),2)
        self.assertEqual(rows[10]['attempts'],0)

    def test_invented_episode_or_missing_scene_is_rejected(self):
        self.plan['entries'][10]['physical_attempt_number']+=1;self.save()
        with self.assertRaisesRegex(ValueError,'must not increment'):run.validate_plan(self.path)
        self.plan['entries'].pop();self.save()
        with self.assertRaisesRegex(ValueError,'all 29'):run.validate_plan(self.path)

    def test_wrong_latest_actual_result_is_rejected(self):
        self.plan['entries'][0]['previous_retry_result']=self.plan['entries'][10]['previous_retry_result'];self.save()
        with self.assertRaisesRegex(ValueError,'Latest actual failure'):run.validate_plan(self.path)
