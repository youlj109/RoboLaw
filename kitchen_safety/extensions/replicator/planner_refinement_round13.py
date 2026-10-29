"""Explicit carrying waypoints for the one unresolved bowl scene."""
from copy import deepcopy
from .joint_goal_repair import validate_round10_policy


def validate_round13_policy(recipe):
    if (recipe.get('retry_round') != 13
            or recipe.get('scene_id') != 'u_shape_bowl_sink__human_default_mpfb'
            or recipe.get('payload_name') != 'bowl'):
        raise ValueError('Round 13 only repairs the remaining human/bowl scene')
    if recipe.get('planning_path_policy', {}).get('kind') != 'collision_checked_graph_joint_goals_raised_release':
        raise ValueError('Round 13 must declare its carrying waypoint policy')
    waypoints = recipe.get('phase_waypoints_world', {})
    if not 1 <= len(waypoints.get('lower', [])) <= 12 or set(waypoints) != {'lower', 'retreat'}:
        raise ValueError('Round 13 requires bounded explicit lower waypoints and retained retreat')
    if recipe.get('planning_options_by_phase') != {p: {'enable_graph': True} for p in ('lower', 'retreat')}:
        raise ValueError('Round 13 must retain graph-seeded joint-goal planning')
    previous = deepcopy(recipe)
    previous['retry_round'] = 10
    previous['planning_path_policy']['kind'] = 'collision_checked_joint_goal_endpoint'
    previous['planning_options_by_phase'] = {p: {'enable_graph': False} for p in ('lower', 'retreat')}
    return validate_round10_policy(previous)
