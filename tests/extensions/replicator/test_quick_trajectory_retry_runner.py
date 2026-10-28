"""The retry queue never repeats a terminal or interrupted scene."""
from contextlib import contextmanager
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('quick_retry_runner', ROOT/'scripts/extensions/replicator/quick_trajectory_retry.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RetryQueueTests(unittest.TestCase):
    def run_queue(self, interrupted=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root/'run'; output.mkdir()
            recipe = root/'recipe.json'; recipe.write_text('{}')
            prior = root/'prior.json'; prior.write_text(json.dumps(dict(start_q=[0]*7, composition=dict(source_paths_sha256={}))))
            rows = []
            for index, status in enumerate(('generated','failed','running' if interrupted else 'pending','pending')):
                rows.append(dict(scene_id=str(index), status=status, retry_round=1,
                    attempts_this_round=0 if status=='pending' else 1,
                    prior_result_path=str(prior), prior_result_sha256=module.sha(prior), recipe={}))
            progress = dict(entries=rows, total=4)
            module.atomic_json(output/'progress.json', progress)
            module.atomic_json(output/'frozen_inputs.json', dict(files={}, recipes_path=str(recipe), recipes_sha256=module.sha(recipe)))
            calls = []
            def fail(row, destination, timeout, retry_recipe):
                calls.append(row['scene_id'])
                return dict(status='failed', stage='approach', start_q=[0]*7, composition=dict(source_paths_sha256={}),
                    result_path=str(output/'scenes'/row['scene_id']/'result.json'), failure_code='IK_FAIL', artifacts={})
            with patch.object(module, 'ROOT', root), patch.object(module.generation, 'run_scene', side_effect=fail), \
                    patch('sys.argv', ['retry', '--output', str(output), '--resume']):
                module.main()
                module.main()
            saved = json.loads((output/'progress.json').read_text())
            return calls, saved

    def test_failure_advances_once_and_resume_does_not_retry(self):
        calls, progress = self.run_queue()
        self.assertEqual(calls, ['2','3'])
        self.assertEqual(progress['stage'], 'complete')
        self.assertTrue(progress['fixed_layout_integrity']['exactly_one_attempt_per_failed_scene'])

    def test_interrupted_attempt_is_failed_without_rerun(self):
        calls, progress = self.run_queue(interrupted=True)
        self.assertEqual(calls, ['3'])
        self.assertEqual(progress['entries'][2]['failure_code'], 'INTERRUPTED_ONCE')
        self.assertEqual(progress['entries'][2]['attempts_this_round'], 1)


class AnnotationOnlyQueueTests(unittest.TestCase):
    @contextmanager
    def queue(self, *, following_plan=False, interrupted=False):
        """Use saved previous output, so the queue must preserve its provenance."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root/'run'; output.mkdir()
            old_scene = root/'previous'/'scenes'/'blocked'; old_scene.mkdir(parents=True)
            evidence = root/'diagnosis.json'
            evidence.write_text(json.dumps({'fixed_initial_mesh_intersection': True}))
            partial = old_scene/'partial_trajectory.npz'
            partial.write_bytes(b'original partial trajectory bytes\x00\x01')
            sidecar = old_scene/'partial_trajectory.json'
            sidecar.write_text(json.dumps({'samples': 78, 'complete': False}))
            source_asset = root/'source.usda'; source_asset.write_text('#usda 1.0\n')
            identity = [[1.,0.,0.,0.], [0.,1.,0.,0.], [0.,0.,1.,0.], [0.,0.,0.,1.]]
            previous = dict(scene_id='blocked', status='failed', stage='lift', failed_phase='lift',
                failure_code='INVALID_START_STATE_WORLD_COLLISION', failure_reason='previous rejection',
                start_q=[.1,.2,.3,-1.4,.5,1.6,.7], start_tcp_world=identity,
                composition=dict(source_paths_sha256={str(source_asset): module.sha(source_asset)}),
                targets={'approach': identity, 'lower': identity}, nominal_attachment=identity,
                nominal_gripper_joint_rad=.5, payload_name='banana',
                planning_only=True, executed=False, task_success_verified=False,
                trajectory_complete=False, trajectory_samples=78, partial_trajectory=True,
                attempts=1, per_stage_max_attempts=1,
                phases=[{'phase': 'pregrasp', 'status': 'generated'},
                        {'phase': 'approach', 'status': 'generated'},
                        {'phase': 'lift', 'status': 'failed'}],
                artifacts={name: dict(path=str(path), sha256=module.sha(path)) for name, path in
                    [('partial_trajectory', partial), ('partial_trajectory_json', sidecar)]})
            prior = old_scene/'result.json'; module.atomic_json(prior, previous)
            recipe = dict(action='annotate_only', annotation_evidence=dict(path=str(evidence),
                sha256=module.sha(evidence)), review_note='布局固定，已有真实初态相交证据，本轮仅标注。')
            rows = [dict(scene_id='blocked', status='running' if interrupted else 'pending',
                stage='annotating' if interrupted else 'queued', retry_round=3,
                attempts_this_round=0, actions_this_round=int(interrupted),
                annotation_only=interrupted, prior_result_path=str(prior),
                prior_result_sha256=module.sha(prior), recipe=recipe)]
            if following_plan:
                rows.append(dict(scene_id='next', status='pending', stage='queued', retry_round=3,
                    attempts_this_round=0, actions_this_round=0, annotation_only=False,
                    prior_result_path=str(prior), prior_result_sha256=module.sha(prior), recipe={'action': 'plan'}))
            recipe_path = root/'recipe.json'; module.atomic_json(recipe_path, {'recipes': rows})
            module.atomic_json(output/'progress.json', dict(entries=rows, total=len(rows), retry_round=3))
            module.atomic_json(output/'frozen_inputs.json', dict(files={str(prior): module.sha(prior)},
                recipes_path=str(recipe_path), recipes_sha256=module.sha(recipe_path)))

            def plan(row, destination, timeout, retry_recipe):
                result = dict(status='failed', stage='approach', start_q=previous['start_q'],
                    composition=previous['composition'], attempts=1, per_stage_max_attempts=1,
                    result_path=str(output/'scenes'/row['scene_id']/'result.json'),
                    failure_code='IK_FAIL', artifacts={})
                module.atomic_json(Path(result['result_path']), result)
                return result

            with patch.object(module, 'ROOT', root), \
                    patch.object(module.generation, 'run_scene', side_effect=plan) as solver, \
                    patch.object(module, 'annotate_only', wraps=module.annotate_only) as annotator, \
                    patch('sys.argv', ['retry', '--output', str(output), '--resume']):
                yield dict(root=root, output=output, prior=prior, previous=previous,
                    partial=partial, sidecar=sidecar, evidence=evidence,
                    solver=solver, annotator=annotator)

    def test_annotation_is_one_action_without_planning_and_queue_advances(self):
        with self.queue(following_plan=True) as state:
            prior_hash = module.sha(state['prior'])
            source_hashes = {path: module.sha(path) for path in (state['partial'], state['sidecar'])}
            module.main()
            module.main()  # A resumed completed queue must not repeat either action.
            progress = json.loads((state['output']/'progress.json').read_text())
            row = progress['entries'][0]
            result = json.loads((state['output']/'scenes'/'blocked'/'result.json').read_text())
            self.assertEqual(state['annotator'].call_count, 1)
            self.assertEqual([call.args[0]['scene_id'] for call in state['solver'].call_args_list], ['next'])
            self.assertEqual(progress['round_actions'], {'planned': 1, 'annotated': 1})
            self.assertEqual(row['actions_this_round'], 1)
            self.assertEqual(row['attempts_this_round'], 0)
            self.assertTrue(progress['fixed_layout_integrity']['passed'])
            self.assertTrue(progress['fixed_layout_integrity']['exactly_one_action_per_failed_scene'])
            self.assertTrue(progress['fixed_layout_integrity']['action_attempt_counts_valid'])
            self.assertFalse(progress['fixed_layout_integrity']['exactly_one_attempt_per_failed_scene'])
            self.assertTrue(result['annotation_only'])
            self.assertFalse(result['planning_attempted'])
            self.assertEqual(result['attempts'], 0)
            self.assertEqual(result['per_stage_max_attempts'], 0)
            self.assertEqual(result['phases'], [])
            self.assertEqual(result['previous_phases'], state['previous']['phases'])
            self.assertEqual(result['failure_code'], state['previous']['failure_code'])
            self.assertEqual(result['trajectory_samples'], 78)
            self.assertFalse(result['trajectory_complete'])
            self.assertFalse(result['executed'])
            self.assertFalse(result['task_success_verified'])
            self.assertEqual(module.sha(state['prior']), prior_hash)
            for name, original in state['previous']['artifacts'].items():
                copied = result['artifacts'][name]
                target = Path(copied['path'])
                self.assertEqual(target.parent, state['output']/'scenes'/'blocked')
                self.assertNotEqual(copied['path'], original['path'])
                self.assertEqual(copied['sha256'], original['sha256'])
                self.assertEqual(module.sha(target), original['sha256'])
                self.assertEqual(target.read_bytes(), Path(original['path']).read_bytes())
            self.assertEqual({path: module.sha(path) for path in source_hashes}, source_hashes)

    def test_interrupted_annotation_is_terminal_without_solver_or_second_action(self):
        with self.queue(interrupted=True) as state:
            module.main()
            module.main()
            progress = json.loads((state['output']/'progress.json').read_text())
            result = json.loads((state['output']/'scenes'/'blocked'/'result.json').read_text())
            state['solver'].assert_not_called()
            state['annotator'].assert_not_called()
            self.assertEqual(result['failure_code'], 'INTERRUPTED_ONCE')
            for key in ('start_q', 'start_tcp_world', 'composition', 'targets', 'nominal_attachment',
                        'nominal_gripper_joint_rad', 'payload_name'):
                self.assertEqual(result[key], state['previous'][key])
            self.assertEqual(result['prior_result_sha256'], module.sha(state['prior']))
            self.assertEqual(result['attempts'], 0)
            self.assertEqual(result['per_stage_max_attempts'], 0)
            self.assertFalse(result['planning_attempted'])
            self.assertEqual(progress['entries'][0]['actions_this_round'], 1)
            self.assertEqual(progress['entries'][0]['attempts_this_round'], 0)
            self.assertTrue(progress['fixed_layout_integrity']['passed'])

    def test_annotation_recovers_hash_bound_solver_history_after_worker_error(self):
        with self.queue() as state:
            progress_path = state['output']/'progress.json'
            progress = json.loads(progress_path.read_text())
            original_row = progress['entries'][0]
            worker = module.error_result(original_row, state['root']/'worker', 'ValueError', 'missing evidence')
            worker_path = Path(worker['result_path'])
            original_row.update(prior_result_path=str(worker_path), prior_result_sha256=module.sha(worker_path))
            module.atomic_json(progress_path, progress)
            module.main()
            result = json.loads((state['output']/'scenes'/'blocked'/'result.json').read_text())
            state['solver'].assert_not_called()
            self.assertTrue(result['annotation_recorded'])
            self.assertEqual(result['stage'], 'lift')
            self.assertEqual(result['failure_code'], 'INVALID_START_STATE_WORLD_COLLISION')
            self.assertEqual(result['prior_result_path'], str(worker_path))
            self.assertEqual(result['historical_planning_result']['path'], str(state['prior']))
            self.assertEqual(len(result['historical_result_chain']), 2)
            for name, item in state['previous']['artifacts'].items():
                self.assertEqual(result['artifacts'][name]['sha256'], item['sha256'])
                self.assertEqual(Path(result['artifacts'][name]['path']).read_bytes(), Path(item['path']).read_bytes())
            self.assertEqual(result['previous_phases'], state['previous']['phases'])

    def test_annotation_refuses_changed_ancestor_after_worker_error(self):
        with self.queue() as state:
            progress = json.loads((state['output']/'progress.json').read_text())
            row = progress['entries'][0]
            worker = module.error_result(row, state['root']/'worker', 'ValueError', 'missing evidence')
            worker_path = Path(worker['result_path'])
            row.update(prior_result_path=str(worker_path), prior_result_sha256=module.sha(worker_path))
            state['prior'].write_text('{}')
            with self.assertRaisesRegex(ValueError, 'Historical planning result changed'):
                module.annotate_only(row, state['output'])
            state['solver'].assert_not_called()

    def test_resume_after_annotation_result_written_preserves_saved_artifacts(self):
        with self.queue() as state:
            module.main()
            result_path = state['output']/'scenes'/'blocked'/'result.json'
            result_hash = module.sha(result_path)
            progress_path = state['output']/'progress.json'
            progress = json.loads(progress_path.read_text())
            progress.update(stage='running')
            progress['entries'][0].update(status='running', stage='annotating')
            module.atomic_json(progress_path, progress)  # Simulate the final progress write being lost.
            module.main()
            self.assertEqual(state['annotator'].call_count, 1)
            state['solver'].assert_not_called()
            self.assertEqual(module.sha(result_path), result_hash)
            resumed = json.loads(progress_path.read_text())
            self.assertEqual(resumed['entries'][0]['attempts_this_round'], 0)
            self.assertEqual(resumed['entries'][0]['actions_this_round'], 1)
            self.assertEqual(resumed['entries'][0]['status'], 'failed')
            for name, artifact in resumed['entries'][0]['artifacts'].items():
                self.assertEqual(module.sha(artifact['path']), state['previous']['artifacts'][name]['sha256'])

    def test_changed_partial_is_rejected_and_never_replanned(self):
        with self.queue() as state:
            state['partial'].write_bytes(b'changed after the original recorded hash')
            module.main()
            module.main()
            result = json.loads((state['output']/'scenes'/'blocked'/'result.json').read_text())
            state['solver'].assert_not_called()
            self.assertEqual(state['annotator'].call_count, 1)
            self.assertEqual(result['failure_code'], 'ValueError')
            self.assertIn('Previous partial artifact changed', result['failure_reason'])
            for key in ('start_q', 'start_tcp_world', 'composition', 'targets', 'nominal_attachment',
                        'nominal_gripper_joint_rad', 'payload_name'):
                self.assertEqual(result[key], state['previous'][key])
            self.assertEqual(result['prior_result_sha256'], module.sha(state['prior']))
            self.assertEqual(result['attempts'], 0)
            self.assertEqual(result['per_stage_max_attempts'], 0)
            self.assertFalse(result['planning_attempted'])
            self.assertEqual(result['artifacts'], {})
            self.assertFalse((state['output']/'scenes'/'blocked'/'partial_trajectory.npz').exists())

    def test_later_annotation_round_keeps_original_partial_phase_history(self):
        with self.queue() as state:
            module.main()
            previous_path = state['output']/'scenes'/'blocked'/'result.json'
            progress = json.loads((state['output']/'progress.json').read_text())
            next_row = dict(progress['entries'][0], retry_round=4,
                prior_result_path=str(previous_path), prior_result_sha256=module.sha(previous_path))
            next_output = state['root']/'round4'
            result = module.annotate_only(next_row, next_output)
            state['solver'].assert_not_called()
            self.assertEqual(result['attempts'], 0)
            self.assertEqual(result['phases'], [])
            self.assertEqual(result['previous_phases'], state['previous']['phases'])
            self.assertEqual(result['prior_result_sha256'], module.sha(previous_path))
            for name, artifact in result['artifacts'].items():
                self.assertEqual(Path(artifact['path']).parent, next_output/'scenes'/'blocked')
                self.assertEqual(module.sha(artifact['path']), state['previous']['artifacts'][name]['sha256'])

    def test_error_context_is_not_copied_from_untrusted_prior_result(self):
        with self.queue() as state:
            progress = json.loads((state['output']/'progress.json').read_text())
            row = progress['entries'][0]
            tampered = dict(state['previous'], start_q=[9]*7)
            module.atomic_json(state['prior'], tampered)
            result = module.error_result(row, state['output'], 'BROKEN_INPUT', 'Untrusted prior')
            self.assertNotIn('start_q', result)
            self.assertNotIn('composition', result)
            self.assertNotIn('targets', result)
            self.assertEqual(result['prior_result_sha256'], row['prior_result_sha256'])

    def test_changed_evidence_blocks_annotation_before_action(self):
        with self.queue() as state:
            state['evidence'].write_text('{"changed": true}')
            with self.assertRaisesRegex(ValueError, 'Correction evidence missing or changed'):
                module.main()
            state['solver'].assert_not_called()
            state['annotator'].assert_not_called()
            progress = json.loads((state['output']/'progress.json').read_text())
            self.assertEqual(progress['entries'][0]['actions_this_round'], 0)
            self.assertEqual(progress['entries'][0]['attempts_this_round'], 0)
            self.assertEqual(progress['entries'][0]['status'], 'pending')

    def test_resume_rejects_missing_annotation_evidence_before_action(self):
        with self.queue() as state:
            progress_path = state['output']/'progress.json'
            progress = json.loads(progress_path.read_text())
            progress['entries'][0]['recipe'].pop('annotation_evidence')
            module.atomic_json(progress_path, progress)
            frozen_path = state['output']/'frozen_inputs.json'
            frozen_hash = module.sha(frozen_path)
            with self.assertRaisesRegex(ValueError, 'Correction evidence missing or changed'):
                module.main()
            state['solver'].assert_not_called()
            state['annotator'].assert_not_called()
            self.assertEqual(module.sha(frozen_path), frozen_hash)
            saved = json.loads(progress_path.read_text())
            self.assertEqual(saved['entries'][0]['actions_this_round'], 0)
            self.assertEqual(saved['entries'][0]['status'], 'pending')

    def test_completed_old_annotation_without_evidence_is_not_restarted(self):
        with self.queue() as state:
            progress = json.loads((state['output']/'progress.json').read_text())
            progress['entries'][0].update(status='failed', actions_this_round=1, annotation_only=True)
            progress['entries'][0]['recipe'].pop('annotation_evidence')
            frozen_path = state['output']/'frozen_inputs.json'
            frozen = json.loads(frozen_path.read_text())
            frozen_hash = module.sha(frozen_path)
            module.bind_correction_evidence(state['output'], progress, frozen)
            self.assertEqual(module.sha(frozen_path), frozen_hash)
            state['solver'].assert_not_called()
            state['annotator'].assert_not_called()

    def test_annotation_limit_counts_action_and_retains_remaining_plan(self):
        with self.queue(following_plan=True) as state:
            with patch('sys.argv', ['retry', '--output', str(state['output']), '--resume', '--limit', '1']):
                module.main()
            progress = json.loads((state['output']/'progress.json').read_text())
            self.assertEqual([row['status'] for row in progress['entries']], ['failed', 'pending'])
            self.assertEqual(progress['round_actions'], {'planned': 0, 'annotated': 1})
            state['solver'].assert_not_called()
            module.main()
            self.assertEqual(state['annotator'].call_count, 1)
            self.assertEqual(state['solver'].call_count, 1)
            progress = json.loads((state['output']/'progress.json').read_text())
            self.assertEqual(progress['stage'], 'complete')
            self.assertTrue(progress['fixed_layout_integrity']['passed'])


class AnnotationInitializationTests(unittest.TestCase):
    @contextmanager
    def source_batch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'previous'; source.mkdir()
            layout = root/'layout.json'; layout.write_text('{"fixed": true}')
            evidence = root/'evidence.json'; evidence.write_text('{"intersection": true}')
            rows = []
            # Put the failed entry last to verify initialization creates no
            # output even after reading all 153 successful carry-forward rows.
            for index in range(154):
                sid = str(index)
                status = 'failed' if index == 153 else 'generated'
                result_path = source/'scenes'/sid/'result.json'
                result = dict(status=status, stage='pregrasp' if status == 'failed' else 'complete',
                    composition={'source_paths_sha256': {}})
                module.atomic_json(result_path, result)
                rows.append(dict(scene_id=sid, status=status, stage=result['stage'],
                    config_path=str(layout), config_sha256=module.sha(layout)))
            module.atomic_json(source/'progress.json', dict(entries=rows, retry_round=3,
                checks_not_performed=['physics_execution']))
            prior = source/'scenes'/'153'/'result.json'
            recipe = dict(action='annotate_only', config_sha256=module.sha(layout),
                result_sha256=module.sha(prior), review_note='固定初态真实相交，本轮仅标注。',
                annotation_evidence=dict(path=str(evidence), sha256=module.sha(evidence)))
            recipes_path = root/'recipes.json'
            output = root/'new-round'
            with patch.object(module, 'ROOT', root):
                yield root, source, output, recipes_path, recipe

    @staticmethod
    def write_recipes(path, recipe):
        module.atomic_json(path, dict(schema_version=1, layout_fixed=True, recipes={'153': recipe}))

    def test_incomplete_annotation_recipe_never_creates_output(self):
        with self.source_batch() as (root, source, output, recipes_path, valid):
            variants = [
                ('missing note', {key: value for key, value in valid.items() if key != 'review_note'}),
                ('blank note', dict(valid, review_note='   ')),
                ('non-string note', dict(valid, review_note=['not a note'])),
                ('missing evidence', {key: value for key, value in valid.items() if key != 'annotation_evidence'}),
                ('non-dict evidence', dict(valid, annotation_evidence='evidence.json')),
                ('missing path', dict(valid, annotation_evidence={'sha256': valid['annotation_evidence']['sha256']})),
                ('missing digest', dict(valid, annotation_evidence={'path': valid['annotation_evidence']['path']})),
                ('missing file', dict(valid, annotation_evidence=dict(valid['annotation_evidence'], path=str(root/'absent.json')))),
                ('directory path', dict(valid, annotation_evidence=dict(valid['annotation_evidence'], path=str(source)))),
                ('invalid digest', dict(valid, annotation_evidence=dict(valid['annotation_evidence'], sha256='invalid'))),
                ('changed file', dict(valid, annotation_evidence=dict(valid['annotation_evidence'], sha256='0'*64))),
            ]
            source_hashes = {path: module.sha(path) for path in source.rglob('*') if path.is_file()}
            for name, recipe in variants:
                with self.subTest(name=name):
                    self.write_recipes(recipes_path, recipe)
                    with self.assertRaises(ValueError):
                        module.initialize(output, source, recipes_path)
                    self.assertFalse(output.exists())
            self.assertEqual({path: module.sha(path) for path in source_hashes}, source_hashes)

    def test_valid_evidence_initializes_without_starting_annotation(self):
        with self.source_batch() as (root, source, output, recipes_path, recipe):
            self.write_recipes(recipes_path, recipe)
            progress = module.initialize(output, source, recipes_path)
            self.assertEqual(progress['retry_round'], 4)
            self.assertEqual(progress['counts']['generated'], 153)
            self.assertEqual(progress['counts']['pending'], 1)
            pending = progress['entries'][-1]
            self.assertEqual(pending['actions_this_round'], 0)
            self.assertEqual(pending['attempts_this_round'], 0)
            self.assertFalse((output/'scenes'/'153'/'result.json').exists())


if __name__ == '__main__': unittest.main()
