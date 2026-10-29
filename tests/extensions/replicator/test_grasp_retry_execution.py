"""CPU-only round-2 lineage, failed-planning and command-policy regressions."""
import copy
import json
from contextlib import redirect_stdout
from io import StringIO
import unittest
from unittest.mock import patch

import numpy as np

from kitchen_safety.extensions.replicator import retry_physical_execution as run
from kitchen_safety.extensions.replicator.physical_retry_plan import retime_candidate
from scripts.extensions.replicator import execute_failure_retry as cli
from scripts.extensions.replicator import run_physical_retry_batches as batches
from tests.extensions.replicator import test_retry_physical_execution as round_one_tests


class GraspRetryTests(unittest.TestCase):
    write = round_one_tests.RetryTests.write

    def setUp(self):
        round_one_tests.RetryTests.setUp(self)
        p = patch.object(batches, 'ROOT', self.root); p.start(); self.addCleanup(p.stop)
        self.predicate = self.write('predicate.py', {'original_predicate': True})
        p = patch.object(run, 'PREDICATE_SOURCES', ('predicate.py',)); p.start(); self.addCleanup(p.stop)
        first_plan, old_rows = run.validate_plan(self.planpath)
        previous = dict(kind='physical_failure_retry', schema_version=1, stage='complete',
            retry_plan_path=str(self.planpath.relative_to(self.root)), retry_plan_sha256=run.sha(self.planpath),
            source_first_pass_manifest=first_plan['source_first_pass_manifest'],
            source_camera_retake_manifest=first_plan['source_camera_retake_manifest'], entries=[])
        for i, row in enumerate(old_rows):
            native = dict(row, kind='physical_failure_retry_execution', status='complete',
                executed=True, attempts=1, task_success=i>=48, execution_status='failed' if i<48 else 'success',
                planning_result_path=row['result_path'], planning_result_sha256=row['result_sha256'],
                source_inputs_unchanged=True, execution_sources_unchanged=True, rendering_enabled=False,
                render_generation_delta=0, render_state_start={'passed':True}, render_state_end={'passed':True},
                runtime_state_setters_used=False, physical_weld_used=False, payload_attachment_used=False,
                execution_source_sha256_start={'predicate.py':self.predicate['sha256']})
            binding = self.write(f'round1/executions/{row["scene_id"]}/result.json', native)
            previous['entries'].append(dict(row, attempts=1, executed=True, task_success=i>=48,
                execution_status=native['execution_status'], execution_result_path=binding['path'],
                execution_result_sha256=binding['sha256']))
        self.previous = previous
        entries = []
        for row in previous['entries'][:48]:
            entry = copy.deepcopy(first_plan['entries'][len(entries)])
            sid = row['scene_id']
            raw = dict(np.load(self.root/entry['source_trajectory_path']))
            # New geometry and closure are permitted, but initialization is not.
            raw['q'] = raw['q'].copy(); raw['q'][:,0] = np.arange(6)*.01
            raw['gripper'][2:5] = .5
            raw_path = self.root/f'round2/planning/{sid}/trajectory.npz'
            raw_path.parent.mkdir(parents=True); np.savez(raw_path, **raw)
            data, _ = retime_candidate(raw)
            data['tcp_world'] = np.repeat(np.eye(4)[None],len(data['q']),axis=0)
            path = self.root/f'round2/trajectories/{sid}.npz'; path.parent.mkdir(exist_ok=True)
            np.savez(path, **data)
            candidate = dict(scene_id=sid,task_id=entry['task_id'],variant_id=entry['variant_id'],
                config_path=entry['config_path'],config_sha256=entry['config_sha256'],
                status='generated',trajectory_complete=True,planning_only=True,executed=False,
                attempts=1,per_stage_max_attempts=1,start_q=[0.]*7,nominal_gripper_joint_rad=.5,
                composition={'source_paths_sha256':{}},
                artifacts={'trajectory':{'path':str(raw_path.relative_to(self.root)), 'sha256':run.sha(raw_path)}})
            entry.update(trajectory_path=str(path.relative_to(self.root)),trajectory_sha256=run.sha(path),
                candidate_planning_result=self.write(f'round2/planning/{sid}/result.json',candidate),
                previous_retry_result=dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']),
                physical_attempt_number=row['physical_attempt_number']+1,planning_status='generated',
                execution_overrides={'close_seconds':3.,'release_seconds':4.})
            entry['adjustment']['candidate_count'] = 1
            entries.append(entry)
        self.plan = dict(kind='physical_failure_retry_plan',schema_version=2,retry_round=2,
            source_first_pass_manifest=first_plan['source_first_pass_manifest'],
            source_camera_retake_manifest=first_plan['source_camera_retake_manifest'],
            source_previous_retry_manifest=self.write('round1/manifest.json',previous),
            expected_scene_count=48,candidate_count_per_scene=1,physical_retries_per_scene=1,
            layout_changed=False,success_predicate_changed=False,entries=entries,
            execution_settings=dict(episode_seconds=200.,max_actions=6000))
        self.planpath = self.root/'round2_plan.json'; self.save_plan()
        self.args = cli.parser().parse_args(['--plan',str(self.planpath),'--output',str(self.root/'round2_execution')])
        cli.apply_settings(self.args,self.plan)

    def save_plan(self):
        self.write(str(self.planpath.relative_to(self.root)),self.plan)

    def fail_last(self):
        row = self.plan['entries'][-1]
        candidate = run.bound_json(row['candidate_planning_result'])
        candidate.update(status='failed',trajectory_complete=False,artifacts={})
        row['candidate_planning_result'] = self.write(row['candidate_planning_result']['path'],candidate)
        row.update(planning_status='failed',failure_reason='one candidate rejected by cuRobo',
                   trajectory_path=None,trajectory_sha256=None)
        self.save_plan()

    def test_new_grasp_and_closure_accept_without_changing_start_or_history(self):
        _, rows = run.validate_plan(self.planpath)
        self.assertEqual(len(rows),48)
        self.assertEqual([r['physical_attempt_number'] for r in rows[:5]],[4,4,4,4,3])
        self.assertEqual(len(rows[0]['prior_physical_attempts']),3)
        candidate,_,data = run.load_retry_inputs(rows[0])
        self.assertEqual(candidate['nominal_gripper_joint_rad'],.5)
        self.assertAlmostEqual(data['q'][-1,0],.05)
        run.validate_entry_lineage(rows[0],self.args)
        effective = run.entry_settings(rows[0],self.args)
        self.assertEqual(effective.close_seconds,3.)
        self.assertEqual(self.args.close_seconds,2.)
        self.assertEqual(run.validate_budget(data,effective)-run.validate_budget(data,self.args),90)
        binding=run.planning_result_bindings(rows[0])
        self.assertEqual(binding['planning_result_path'],rows[0]['candidate_planning_result']['path'])
        self.assertEqual(binding['planning_result_sha256'],rows[0]['candidate_planning_result']['sha256'])
        self.assertEqual(binding['original_planning_result'],dict(path=rows[0]['result_path'],sha256=rows[0]['result_sha256']))

    def test_resume_preserves_real_failure_reason_as_execution_output(self):
        _,rows=run.validate_plan(self.planpath)
        manifest=dict(kind='physical_failure_retry',retry_plan_path=str(self.planpath),retry_plan_sha256=run.sha(self.planpath),
            entries=copy.deepcopy(rows),runtime_revisions=[dict(id=0,source_sha256=run.runtime_hashes(),settings=run.runtime_settings(self.args))])
        manifest['entries'][0].update(execution_status='failed',attempts=1,executed=True,failure_reason='Actual original predicate failed')
        cli.check_existing(manifest,rows,self.args)
        self.assertNotIn(manifest['entries'][0],cli.select_pending(manifest,self.args))

    def test_failed_planning_is_terminal_without_physical_attempt(self):
        self.fail_last(); _, rows = run.validate_plan(self.planpath)
        self.assertEqual(rows[-1]['execution_status'],'planning_failed')
        self.assertEqual(rows[-1]['attempts'],0); self.assertFalse(rows[-1]['executed'])
        self.assertEqual(len(cli.select_pending({'entries':rows},self.args)),47)
        with self.assertRaisesRegex(ValueError,'failed planning'):
            run.execute_one(rows[-1],self.root/'unused',self.args)
        with redirect_stdout(StringIO()) as output:
            cli.main(['--plan',str(self.planpath),'--output',str(self.args.output),'--dry-run'])
        result = json.loads(output.getvalue())
        self.assertEqual(result['planning_failed'],1);self.assertEqual(result['selected_pending'],47)
        self.assertFalse(result['physics_launched']);self.assertFalse(self.args.output.exists())

    def test_fixed_48_cohort_and_previous_result_binding(self):
        original = copy.deepcopy(self.plan)
        for mutate in (lambda p:p['entries'].pop(),
                       lambda p:p['entries'].append(p['entries'][0]),
                       lambda p:p['entries'][0].update(previous_retry_result=self.predicate),
                       lambda p:p.update(layout_changed=True),
                       lambda p:p.update(success_predicate_changed=True)):
            self.plan=copy.deepcopy(original);mutate(self.plan);self.save_plan()
            with self.assertRaises(ValueError):run.validate_plan(self.planpath)

    def test_previous_result_tampering_and_predicate_change_rejected(self):
        (self.root/self.predicate['path']).write_text('changed original predicate')
        with self.assertRaisesRegex(ValueError,'predicate source changed'):run.validate_plan(self.planpath)

    def test_raw_sha_and_retimed_geometry_tampering_rejected(self):
        _,rows=run.validate_plan(self.planpath);row=rows[0]
        path=self.root/row['trajectory_path'];data=dict(np.load(path));data['q'][5,2]+=.01
        np.savez(path,**data);row['trajectory_sha256']=run.sha(path)
        with self.assertRaisesRegex(ValueError,'Geometric'):run.load_retry_inputs(row)
        raw=run.bound_json(row['candidate_planning_result'])['artifacts']['trajectory']
        (self.root/raw['path']).write_bytes(b'changed raw trajectory')
        with self.assertRaisesRegex(ValueError,'trajectory changed'):run.load_retry_inputs(row)

    def test_new_start_and_open_gripper_changes_rejected(self):
        _,rows=run.validate_plan(self.planpath);row=rows[0]
        candidate=run.bound_json(row['candidate_planning_result']);candidate['start_q'][0]=.1
        row['candidate_planning_result']=self.write(row['candidate_planning_result']['path'],candidate)
        with self.assertRaisesRegex(ValueError,'frozen robot start'):run.load_retry_inputs(row)
        candidate['start_q'][0]=0
        raw=candidate['artifacts']['trajectory'];path=self.root/raw['path'];data=dict(np.load(path))
        data['gripper'][0]=.1;np.savez(path,**data);raw['sha256']=run.sha(path)
        row['candidate_planning_result']=self.write(row['candidate_planning_result']['path'],candidate)
        with self.assertRaisesRegex(ValueError,'open-gripper'):run.load_retry_inputs(row)

    def test_invalid_or_unsupported_overrides_rejected(self):
        for override in ({'initial_settle_seconds':1.},{'release_seconds':0.},{'close_seconds':float('nan')},
                         {'close_seconds':True},{'release_seconds':10.1}):
            with self.subTest(override=override),self.assertRaises(ValueError):
                run.entry_settings(dict(retry_schema_version=2,execution_overrides=override),self.args)

    def test_batch_scheduler_preserves_planning_failure_and_single_attempt(self):
        self.fail_last();_,rows=run.validate_plan(self.planpath)
        manifest=dict(schema_version=2,retry_round=2,entries=rows)
        chosen=batches.pending_task(manifest);self.assertEqual(len(chosen),11)
        after=copy.deepcopy(manifest)
        for row in after['entries'][:11]:row.update(execution_status='failed',attempts=1,executed=True)
        batches.verify(manifest,after,chosen)
        after['entries'][-1]['attempts']=1
        with self.assertRaisesRegex(ValueError,'Unselected'):batches.verify(manifest,after,chosen)
        with self.assertRaisesRegex(ValueError,'Planning failure'):batches.pending_task(after)


if __name__=='__main__':unittest.main()
