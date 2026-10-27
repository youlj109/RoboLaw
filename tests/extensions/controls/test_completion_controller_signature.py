"""Controller matching must not confuse obstacle layout with task/controller changes."""
import importlib.util
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[3]/'scripts/extensions/controls/complete_reviewed_scenes.py'
spec = importlib.util.spec_from_file_location('complete_reviewed_scenes', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_layout_changes_do_not_change_obstacle_free_controller_signature():
    config = dict(task_id='temperature', source_sha256='source', robot_pose_xyzw=[1, 2, 3],
                  initial_roll_deg=-90., variants={'human': {'position': [1, 2, 3]}})
    changed = dict(config, variants={'human': {'position': [3, 2, 1]}}, execution_revision={'round': 2})
    assert module.controller_signature(config) == module.controller_signature(changed)


def test_auxiliary_pose_and_ik_seed_require_separately_matched_baselines():
    config = dict(task_id='temperature', pregrasp_tcp_world_m=[5.17, -3.2, 1.26],
                  initial_roll_deg=-90., prepress_planning={'ik_branch_seed': [0.]*7})
    for changed in (dict(config, pregrasp_roll_deg=-165.),
                    dict(config, pregrasp_tcp_world_m=[5.17, -3.2, 1.32]),
                    dict(config, prepress_planning={'ik_branch_seed': [1.]*7})):
        assert module.controller_signature(config) != module.controller_signature(changed)


def test_host_repo_paths_are_translated_to_container_working_directory():
    assert module.runtime_argument(module.ROOT/'configs/extensions/controls/trial.json') == 'configs/extensions/controls/trial.json'
    assert module.runtime_argument(Path('datasets/episode')) == 'datasets/episode'
    assert module.runtime_argument(42) == '42'


def test_policy_provenance_is_not_a_physical_controller_parameter():
    config={'prepress_planning':{'method':'curobo_joint_goal','ik_branch_seed':[0.]*7,'provenance':'old'}}
    newer={'prepress_planning':dict(config['prepress_planning'],provenance='new')}
    assert module.controller_signature(config)==module.controller_signature(newer)
