"""Mixed-layout no-render history must not replace the original camera source."""
import json
import pytest
from scripts.extensions.replicator import render_physical_retry_failures as replay


@pytest.fixture
def history(monkeypatch, tmp_path):
    monkeypatch.setattr(replay, 'ROOT', tmp_path)
    def write(name, data):
        path = tmp_path / name
        path.write_text(json.dumps(data))
        return dict(result_path=name, result_sha256=replay.sha(path))
    root_config = 'a' * 64
    original = write('original.json', dict(scene_id='scene', config_sha256=root_config,
        overview={'camera_world_pose': [1, 2, 3]}))
    revised = write('round8.json', dict(scene_id='scene', config_sha256='b' * 64,
        overview=None, camera_override={'applied': False}))
    return dict(scene_id='scene', prior_physical_attempts=[original, revised]), write, root_config


def test_bound_no_camera_revision_is_skipped_and_original_binding_retained(history):
    result, _, root_config = history
    observation, binding = replay.select_historical_observation(result, root_config)
    assert observation['config_sha256'] == root_config
    assert binding == dict(observation_reference_path='original.json',
        observation_reference_sha256=result['prior_physical_attempts'][0]['result_sha256'])


@pytest.mark.parametrize('change', ['stale_sha', 'foreign_scene', 'foreign_camera'])
def test_no_camera_skip_never_bypasses_sha_scene_or_camera_config(history, change):
    result, write, root_config = history
    if change == 'stale_sha':
        result['prior_physical_attempts'][-1]['result_sha256'] = '0' * 64
    else:
        data = dict(scene_id='foreign' if change == 'foreign_scene' else 'scene', config_sha256='b' * 64)
        if change == 'foreign_camera':
            data['camera_override'] = {'applied': True}
        result['prior_physical_attempts'][-1] = write('round8.json', data)
    with pytest.raises(ValueError):
        replay.select_historical_observation(result, root_config)


def test_no_camera_history_is_not_invented(history):
    result, _, root_config = history
    result['prior_physical_attempts'] = result['prior_physical_attempts'][1:]
    with pytest.raises(ValueError, match='No recorded camera'):
        replay.select_historical_observation(result, root_config)


@pytest.mark.parametrize('round_number', [9, 10, 11, 12, 13])
def test_retained_load_keeps_native_metadata_binding_and_matches_publisher(monkeypatch, round_number):
    from copy import deepcopy
    from tests.extensions.replicator.test_scene_revision_round8_bindings import Round8RendererBindings
    from scripts.extensions.replicator import publish_physical_retry_round9 as publisher
    if round_number == 10:
        from scripts.extensions.replicator import publish_physical_retry_round10 as publisher
    elif round_number == 11:
        from scripts.extensions.replicator import publish_physical_retry_round11 as publisher
    elif round_number == 12:
        from scripts.extensions.replicator import publish_physical_retry_round12 as publisher
    elif round_number == 13:
        from scripts.extensions.replicator import publish_physical_retry_round13 as publisher
    nested = Round8RendererBindings('test_new_state_uses_round8_layout_and_original_camera_not_round7')
    nested.setUp()
    try:
        prior = nested.fixture
        f = prior.fixture
        monkeypatch.setattr(replay, 'ROOT', f.root)
        monkeypatch.setattr(publisher, 'ROOT', f.root)
        old_result = f.json('round8_actual.json', prior.result)
        row = dict(f.row, config_path=f.newconfig['path'], config_sha256=f.newconfig['sha256'],
            source_scene_path=f.newstage['path'], source_scene_sha256=f.newstage['sha256'],
            scene_revision=prior.revision, layout_changed=True, execution_status='failed',
            executed=True, attempts=1, task_success=False,
            execution_result_path=old_result['path'], execution_result_sha256=old_result['sha256'])
        previous = f.json('manifest8.json', dict(retry_round=8, stage='complete', entries=[row]))
        scope = dict(retained_scene_revision=True, scene_settings_changed_this_round=False, scene_settings_changed_once=True)
        if round_number >= 10:
            planned = dict(row, **scope, execution_status='planning_failed', planning_status='failed',
                attempts=0, executed=False)
            planned.pop('execution_result_path'); planned.pop('execution_result_sha256')
            other = dict(planned, scene_id='other')
            previous = f.json('manifest9.json', dict(retry_round=9, stage='complete',
                source_previous_retry_manifest=previous, entries=[planned, other]))
            if round_number >= 11:
                previous = f.json('manifest10.json', dict(retry_round=10, stage='complete',
                    source_previous_retry_manifest=previous, entries=[planned, other]))
            if round_number >= 12:
                succeeded = dict(other, status='complete', execution_status='success', planning_status='generated',
                    attempts=1, executed=True, task_success=True)
                success_ref = f.json('other_round11_success.json', succeeded)
                succeeded.update(execution_result_path=success_ref['path'], execution_result_sha256=success_ref['sha256'])
                previous = f.json('manifest11.json', dict(retry_round=11, stage='complete',
                    source_previous_retry_manifest=previous, entries=[planned, succeeded]))
            if round_number >= 13:
                previous = f.json('manifest12.json', dict(retry_round=12, stage='complete',
                    source_previous_retry_manifest=previous, entries=[planned]))
        raw = dict(deepcopy(prior.result), **scope, layout_changed=True,
            source_scene_path=f.newstage['path'], source_scene_sha256=f.newstage['sha256'],
            source_previous_retry_manifest=previous,
            original_planning_result=dict(path=f.row['result_path'], sha256=f.row['result_sha256']))
        raw['prior_physical_attempts'].append(dict(result_path=old_result['path'], result_sha256=old_result['sha256']))
        result_ref = f.json('executions/scene/result.json', raw)
        row.update(scope, execution_result_path=result_ref['path'], execution_result_sha256=result_ref['sha256'])
        result, layout, arrays, metadata, binding = replay.load_record(f.root/'manifest.json', row)
        assert binding == publisher.expected_diagnostic_binding(result_ref, raw)
        assert binding['retained_scene_revision'] is True and binding['scene_settings_changed_this_round'] is False
        assert binding['historical_camera_config'] == f.oldconfig and layout == f.new
        assert result['diagnostic_observation_reference']['config_sha256'] == f.oldconfig['sha256']
        assert metadata['samples'] == len(arrays['physics_step']) == 2
        assert binding['physics_metadata_sha256'] == raw['physics_evidence']['metadata_sha256']
        wrong_scope = deepcopy(raw); wrong_scope.pop('retained_scene_revision')
        altered = f.json('executions/scene/result.json', wrong_scope)
        row['execution_result_sha256'] = altered['sha256']
        with pytest.raises(ValueError, match='retained-layout scope differ'):
            replay.load_record(f.root/'manifest.json', row)
        raw['physics_evidence']['metadata_sha256'] = '0' * 64
        altered = f.json('executions/scene/result.json', raw)
        row['execution_result_sha256'] = altered['sha256']
        with pytest.raises(ValueError, match='Source hash mismatch'):
            replay.load_record(f.root/'manifest.json', row)
    finally:
        nested.doCleanups()
