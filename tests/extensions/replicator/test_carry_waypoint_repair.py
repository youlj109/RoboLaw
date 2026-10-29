"""Reject incomplete or mislabelled carrying waypoint contracts."""
from copy import deepcopy
import json
from pathlib import Path
import pytest
from kitchen_safety.extensions.replicator.planner_refinement_round12 import validate_round12_policy
from kitchen_safety.extensions.replicator.planner_refinement_round13 import validate_round13_policy


def valid_recipe():
    root = Path(__file__).resolve().parents[3]
    document = json.loads((root / 'configs/extensions/replicator/physical_retry_round11_human_recipes_20261029.json').read_text())
    recipe = deepcopy(next(iter(document['recipes'].values())))
    recipe['retry_round'] = 12
    recipe['planning_path_policy']['kind'] = 'collision_checked_graph_joint_goals_with_carry_waypoints'
    recipe['phase_waypoints_world']['lower'] = [recipe['target_poses_world']['transfer']]
    recipe['joint_space_target_q_by_phase']['lower'].insert(0, recipe['joint_space_target_q_by_phase']['retreat'][-1])
    return recipe


@pytest.mark.parametrize('change', ['missing_joint_goal', 'missing_waypoint', 'undeclared_phase', 'soft_cost', 'other_scene', 'graph_off'])
@pytest.mark.parametrize('retry_round', [12, 13])
def test_carry_waypoints_cannot_omit_corresponding_goals_or_change_policy(change, retry_round):
    recipe = valid_recipe()
    recipe['retry_round'] = retry_round
    validate = validate_round12_policy if retry_round == 12 else validate_round13_policy
    if retry_round == 13:
        recipe['planning_path_policy']['kind'] = 'collision_checked_graph_joint_goals_raised_release'
    assert validate(recipe) is None
    if change == 'missing_joint_goal': recipe['joint_space_target_q_by_phase']['lower'].pop(0)
    elif change == 'missing_waypoint': recipe['phase_waypoints_world']['lower'] = []
    elif change == 'undeclared_phase': recipe['phase_waypoints_world']['lift'] = []
    elif change == 'soft_cost': recipe['hold_orientation_phases'] = ['lower']
    elif change == 'other_scene': recipe['scene_id'] = 'other'
    elif change == 'graph_off': recipe['planning_options_by_phase']['lower']['enable_graph'] = False
    with pytest.raises(ValueError): validate(recipe)
