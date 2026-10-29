"""Change only the solver strategy while retaining physical and FK requirements."""
from copy import deepcopy
from .lift_path_validation import validate_round9_policy
from .joint_goal_repair import validate_round10_policy


def validate_round11_policy(recipe):
    if recipe.get('retry_round')!=11:raise ValueError('Expected round 11 solver refinement')
    previous=deepcopy(recipe)
    if recipe.get('payload_name')=='mustard_bottle':
        if recipe.get('planning_options_by_phase')!={'lift':{'constraint_weight':1.,'enable_graph':True}}:
            raise ValueError('Round 11 mustard changes only to the declared unit pose weight and graph seed')
        if recipe.get('hold_orientation_phases')!=['lift'] or recipe.get('linear_motion_phases')!=['lift']:
            raise ValueError('Round 11 mustard retains explicit vertical lift soft cost')
        previous['retry_round']=9
        limits=validate_round9_policy(previous)
        if limits!={'reference_phase':'approach','max_xy_deviation_m':.008,'max_orientation_deviation_deg':4.,'min_z_offset_m':-.002}:
            raise ValueError('Round 11 cannot relax the validated lift gate')
    else:
        if recipe.get('planning_path_policy',{}).get('kind')!='collision_checked_graph_joint_goals':
            raise ValueError('Round 11 bowl explicitly uses graph-seeded joint goals')
        if recipe.get('planning_options_by_phase')!={p:{'enable_graph':True} for p in ('lower','retreat')}:
            raise ValueError('Round 11 bowl must explicitly enable the graph for both joint-goal phases')
        # The prior strict joint-goal contract is unchanged except for this
        # explicitly declared graph option, not an implicit fallback relaxation.
        previous['retry_round']=10
        previous['planning_path_policy']['kind']='collision_checked_joint_goal_endpoint'
        previous['planning_options_by_phase']={p:{'enable_graph':False} for p in ('lower','retreat')}
        limits=validate_round10_policy(previous)
    return limits
