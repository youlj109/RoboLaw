from scripts.extensions.controls.build_benchmark_report import complete_benchmark,paired_binding
from scripts.extensions.controls.summarize_reviewed_completion import EXPECTED_SCENES


def test_nine_positive_cells_are_required_not_one_successful_cell():
    rows=[dict(scene=sid,accepted=True) for sid in EXPECTED_SCENES]
    assert complete_benchmark(rows)
    assert not complete_benchmark(rows[:-1])
    assert not complete_benchmark(rows+[rows[0]])
    rows[-1]['accepted']=False
    assert not complete_benchmark(rows)


def test_empty_pair_files_and_empty_configs_cannot_bind(tmp_path):
    gates=paired_binding({}, {}, {}, {}, tmp_path, 'human', tmp_path/'missing.json')
    assert not any(gates.values())


def test_missing_config_hash_is_not_equal_missing_file_hash(tmp_path):
    config=dict(task_id='left',variants=dict(robot=dict(asset='fixture.usda')))
    gates=paired_binding(dict(config_sha256=None),config,config,config,tmp_path,'robot',tmp_path/'missing.json')
    assert not gates['paired_config_bound']
