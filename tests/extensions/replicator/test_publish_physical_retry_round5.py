"""Minimal round5 publication regressions with round4-style bound fixtures.

Files are synthetic; no simulator, physics, GPU, or new image review is used.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.extensions.replicator import publish_physical_retry_round5 as pub

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def fixture(monkeypatch, tmp_path):
    previous = json.loads((ROOT/pub.PREVIOUS).read_text())
    monkeypatch.setattr(pub, 'ROOT', tmp_path)

    def write(path, data):
        path = pub.local(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, ensure_ascii=False))
        return pub.ref(path)

    prior = write('prior/manifest.json', {'stage': 'complete'})
    previous['source_retry_manifest'] = prior
    evidence = write('cpu/evidence.json', {'synthetic': True})
    plan = dict(retry_round=5, layout_changed=False, success_predicate_changed=False,
                source_previous_retry_manifest=prior, source_first_pass_manifest=prior,
                source_camera_retake_manifest=prior, execution_settings={}, entries=[])
    manifest = dict(kind='physical_failure_retry', retry_round=5, stage='complete', rendering_enabled=False,
                    source_previous_retry_manifest=prior, entries=[])
    audit = dict(kind='physical_retry_round5_independent_audit', passed=True, errors=[], all5_audited=True,
                 audited_scenes=5, require_complete=True, selected_scene=None, entries=[])
    diagnostics = dict(kind='physical_retry_planning_failure_cpu_diagnostics', retry_round=5,
                       total=3, diagnosed=3, complete=True, source_plan_unchanged=True, errors=0,
                       physical_execution_started=False, kit_started=False, curobo_planning_calls=0,
                       new_target_candidates=0, cpu_ik_seed_attempts_per_scene=0, layout_changed=False,
                       diagnostic_sources={}, method='Retained solver records / source-FK fixtures',
                       limitations=['No new IK or physical result'], entries=[])
    failed = [old for old in previous['entries'] if not old['latest_success']]
    assert len(failed) == 5
    for index, old in enumerate(failed):
        sid = old['scene_id']
        config = write('configs/'+sid+'.json', {'scene_id': sid})
        scene = write('scenes/'+sid+'.usda', {'synthetic': sid})
        old.update(config_path=config['path'], config_sha256=config['sha256'])
        old['latest_physical_result'] = write('prior/'+sid+'.json', {'scene_id': sid})
        layer = next(old[key] for key in ('retry_round4', 'retry_round3', 'retry_round2', 'retry')
                     if old.get(key) and old[key].get('executed'))
        layer['result'] = deepcopy(old['latest_physical_result'])
        generated = index < 2
        identity = dict(scene_id=sid, task_id=old['task_id'], variant_id=old['variant_id'],
                        config_path=config['path'], config_sha256=config['sha256'])
        status = 'TCP_SEGMENT_DEVIATION_LIMIT' if index == 2 else 'MotionGenStatus.FINETUNE_TRAJOPT_FAIL'
        candidate = dict(identity, status='generated' if generated else 'failed', attempts=1,
                         executed=False, planning_only=True, composition={'source_paths_sha256':{}},
                         failure_reason=None if generated else status, artifacts={}, phases=[],
                         failed_phase=None if generated else 'lift')
        if not generated:
            segment = dict(phase='lift', segment_index=0, solver_status=status, status='failed',
                           solver_success=index == 2, samples=39 if index == 2 else 0)
            if index == 2:
                segment['trajectory'] = write('segments/'+sid+'.npz', {'fixture':'independent FK checked by separate audit'})
            candidate['phases'] = [dict(segment, segments=[deepcopy(segment)])]
        planning = write('planning/'+sid+'.json', candidate)
        row = dict(identity, candidate_planning_result=planning, previous_retry_result=old['latest_physical_result'],
                   source_scene_path=scene['path'], source_scene_sha256=scene['sha256'],
                   physical_attempt_number=5, planning_status='generated' if generated else 'failed',
                   trajectory_path=None, trajectory_sha256=None, failure_reason=None if generated else status,
                   adjustment=dict(description='Synthetic frozen grasp adjustment', evidence_path=evidence['path'], evidence_sha256=evidence['sha256']))
        if generated:
            trajectory = write('trajectories/'+sid+'.npz', {'fixture':'trajectory bytes'})
            row.update(trajectory_path=trajectory['path'], trajectory_sha256=trajectory['sha256'])
        plan['entries'].append(deepcopy(row))
        row.update(execution_status='success' if generated else 'planning_failed', attempts=int(generated),
                   executed=generated, task_success=generated)
        checked = {key:row[key] for key in ('scene_id','planning_status','execution_status','attempts','executed','task_success','candidate_planning_result')}
        checked['status'] = 'passed'
        if generated:
            raw = dict(row, kind='physical_failure_retry_execution', planning_result_path=planning['path'],
                       planning_result_sha256=planning['sha256'], prior_physical_attempts=[])
            result = write(Path('exports')/pub.RUN/'executions'/sid/'result.json', raw)
            row.update(execution_result_path=result['path'], execution_result_sha256=result['sha256'])
            checked.update(result=result, no_render_gate_passed=True)
        else:
            item = dict(identity, candidate_planning_result=planning, config=config, source_scene=scene,
                        status='diagnosed', failed_phase='lift', solver_status=status,
                        root_cause_confirmed=False, scene_infeasibility_established=False,
                        observed_cpu_facts_zh=['保留本次规划结果记录，未进行新物理或IK。'],
                        interpretation_zh='仅单候选失败，未证明场景无解。', next_adjustment_zh='下一轮修改预声明种子，保留原5mm限制。')
            if index == 2:
                item.update(diagnostic_method='independent_source_fk_segment_gate',
                            segment_audit=dict(segments=[dict(trajectory=segment['trajectory'],
                                deviation_gate_passed=False, independent_fk_verified=True,
                                solver_status='TCP_SEGMENT_DEVIATION_LIMIT')]))
            else:
                item.update(diagnostic_method='solver_failure_without_failed_segment_path',
                            failed_segment_trajectory_available=False,
                            failed_solver_record=dict(phase='lift',segment_index=0,solver_status=status,
                                                      solver_success=False,samples=0,trajectory_saved=False))
            diagnostics['entries'].append(item)
        manifest['entries'].append(row)
        audit['entries'].append(checked)
    write(pub.PREVIOUS, previous)
    plan_ref = write('plan.json', plan)
    manifest.update(retry_plan_path=plan_ref['path'], retry_plan_sha256=plan_ref['sha256'])
    diagnostics['source_plan'] = plan_ref

    def flush():
        audit['manifest_snapshot'] = write(pub.MANIFEST, manifest)
        write(pub.AUDIT, audit)
        write(pub.PLANNING_DIAGNOSTICS, diagnostics)

    flush()
    return SimpleNamespace(previous=previous, plan=plan, manifest=manifest, audit=audit,
                           diagnostics=diagnostics, write=write, flush=flush, root=tmp_path)


def test_merge_149_plus_two_preserves_history_and_three_zero_episode_failures(fixture):
    data = pub.build()
    assert len(data['entries']) == 154 and len(data['tasks']) == 14
    assert data['counts']['previous_success'] == 149
    assert data['counts']['new_successes'] == 2
    assert data['counts']['latest_success'] == 151
    assert data['counts']['latest_failed'] == 3
    assert data['counts']['physical_attempts'] == 2
    assert data['counts']['planning_failed'] == 3
    for old, new in zip(fixture.previous['entries'], data['entries']):
        for key in ('first_pass','camera_retake','retry','retry_round2','retry_round3','retry_round4'):
            assert old.get(key) == new.get(key)
        current = new['retry_round5']
        if current and current['execution_status'] == 'planning_failed':
            assert current['attempts'] == 0 and current['executed'] is False
            assert current['physical_attempt_number'] is None and 'result' not in current
            assert new['latest_physical_result'] == old['latest_physical_result']
            assert current['diagnostic_visual_review']['status'] == 'prior_round_reference_only'
    assert not (fixture.root/pub.JSON_PATH).exists()
    assert data['safety_accepted'] is data['safety_certified'] is data['long_term_stability_verified'] is False


def test_no_actual_failure_needs_no_visual_artifact_and_both_methods_publish(fixture):
    assert not pub.local(pub.VISUAL).exists()
    data = pub.build()
    assert data['counts']['actual_failed_executions'] == 0
    assert data['diagnostic_visual_review']['review_required'] is False
    assert data['diagnostic_visual_review']['source_visual_review'] is None
    counts = data['planning_failure_diagnostics']
    assert counts['segment_gate_diagnosed'] == 1
    assert counts['solver_failure_without_path_diagnosed'] == 2
    assert counts['fixed_goal_cpu_ik_checked'] == counts['fixed_goal_cpu_ik_solved'] == 0
    text = pub.document(data)
    assert '151' in text and '14 × 11' in text
    assert '本轮实际执行失败为0景' in text
    assert '3景规划失败没有新增物理测试' in text
    assert '](../../../reports/' not in text
    fixture.write(pub.VISUAL, dict(kind='physical_failure_visual_review', entries=[]))
    assert pub.build()['diagnostic_visual_review']['source_visual_review'] == pub.ref(pub.VISUAL)


def test_planning_failure_cannot_invent_an_episode(fixture):
    row = next(r for r in fixture.manifest['entries'] if r['execution_status'] == 'planning_failed')
    row.update(attempts=1, executed=True)
    check = next(a for a in fixture.audit['entries'] if a['scene_id'] == row['scene_id'])
    check.update(attempts=1, executed=True)
    fixture.flush()
    with pytest.raises(ValueError, match='zero physical attempts'):
        pub.build()


@pytest.mark.parametrize('method', ['independent_source_fk_segment_gate','solver_failure_without_failed_segment_path'])
def test_each_diagnostic_method_rejects_a_stale_candidate_binding(fixture, method):
    item = next(d for d in fixture.diagnostics['entries'] if d['diagnostic_method'] == method)
    item['candidate_planning_result'] = dict(path='old.json', sha256='0'*64)
    fixture.flush()
    with pytest.raises(ValueError, match='candidate_planning_result binding differs'):
        pub.build()


def test_solver_failure_diagnosis_cannot_invent_failed_path(fixture):
    item = next(d for d in fixture.diagnostics['entries'] if d['diagnostic_method'] == 'solver_failure_without_failed_segment_path')
    item['failed_solver_record']['trajectory_saved'] = True
    fixture.flush()
    with pytest.raises(ValueError, match='must not fabricate a path or IK'):
        pub.build()


@pytest.mark.parametrize('method', ['unknown_method','fixed_goal_one_seed_cpu_ik'])
def test_unknown_diagnosis_or_missing_solved_is_not_reported_as_ik_failure(fixture, method):
    item = fixture.diagnostics['entries'][-1]
    item['diagnostic_method'] = method
    item['fixed_goal_one_seed_ik'] = dict(seed_attempts=1)  # deliberately no Boolean solved
    fixture.flush()
    with pytest.raises(ValueError, match='Unknown planning diagnostic method|single fixed-goal CPU IK'):
        pub.build()
