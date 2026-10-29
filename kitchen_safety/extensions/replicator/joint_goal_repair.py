"""Frozen feasible joint goals; cuRobo still generates every executed path."""
import numpy as np
from .lift_path_validation import checked_limits


def solver_diagnostics(output):
    """Small CPU summaries; never retain solver GPU tensors or a failed path as executable."""
    def plain(value):
        if value is None or isinstance(value,(str,bool,int,float)): return value
        if hasattr(value,'detach'): value=value.detach().cpu().numpy()
        if isinstance(value,np.ndarray):
            if value.size<=64:return value.tolist()
            return dict(shape=list(value.shape),minimum=float(value.min()),maximum=float(value.max()))
        return str(type(value).__name__)
    keys=('success','valid_query','optimized_dt','position_error','rotation_error','cspace_error',
        'attempts','trajopt_attempts','used_graph','ik_time','graph_time','trajopt_time','finetune_time')
    result={k:plain(getattr(output,k,None)) for k in keys}
    debug=getattr(output,'debug_info',None)
    result['debug_stages']={}
    if isinstance(debug,dict):
        for name,value in debug.items():
            row={k:plain(getattr(value,k,None)) for k in ('success','position_error','rotation_error','cspace_error','optimized_dt','solve_time')}
            metrics=getattr(value,'metrics',None)
            if metrics is not None:
                row['metrics']={k:plain(getattr(metrics,k,None)) for k in ('feasible','constraint','position_error','rotation_error','cspace_error')}
            result['debug_stages'][name]=row
    return result


def validate_round10_policy(recipe):
    if (recipe.get('retry_round') != 10 or recipe.get('retained_scene_revision') is not True
            or recipe.get('scene_settings_changed_this_round') is not False):
        raise ValueError('Round 10 must retain its exact prior layout')
    mustard=recipe.get('payload_name')=='mustard_bottle'
    if not mustard and recipe.get('payload_name')!='bowl': raise ValueError('Unknown joint goal repair payload')
    expected={'lift'} if mustard else {'lower','retreat'}
    goals=recipe.get('joint_space_target_q_by_phase',{})
    if set(goals)!=expected: raise ValueError('Joint goal phases differ from declared repair')
    policy=recipe.get('planning_path_policy',{})
    kind='collision_checked_joint_goal_lift_corridor' if mustard else 'collision_checked_joint_goal_endpoint'
    if (policy.get('kind')!=kind or policy.get('previous_straight_line_gate_removed') is not True
            or recipe.get('segment_deviation_limits_m')):
        raise ValueError('Round 10 must retain its explicit joint-goal/FK policy')
    for phase,qs in goals.items():
        if (phase in recipe.get('hold_orientation_phases',[])
                or phase in recipe.get('linear_motion_phases',[])
                or recipe.get('planning_options_by_phase',{}).get(phase)!={'enable_graph':False}):
            raise ValueError('Joint goals do not imply Cartesian soft constraints or graph fallback')
        q=np.asarray(qs,dtype=float)
        if q.shape!=(len(recipe.get('phase_waypoints_world',{}).get(phase,[]))+1,7) or not np.isfinite(q).all():
            raise ValueError('Every joint goal must correspond to exactly one frozen endpoint')
    limits=checked_limits(recipe)
    if mustard:
        if limits!={'reference_phase':'approach','max_xy_deviation_m':.008,'max_orientation_deviation_deg':4.,'min_z_offset_m':-.002}:
            raise ValueError('Joint-goal repair cannot relax the prior lift FK limits')
        reference=np.asarray(recipe['target_poses_world']['approach'])
        points=recipe.get('phase_waypoints_world',{}).get('lift',[])+[recipe['target_poses_world']['lift']]
        heights=[]
        for p in points:
            p=np.asarray(p)
            if not np.allclose(p[:3,:3],reference[:3,:3],atol=1e-7,rtol=0) or not np.allclose(p[:2,3],reference[:2,3],atol=1e-7,rtol=0):
                raise ValueError('Lift goals must remain vertical without wrist rotation')
            heights.append(float(p[2,3]-reference[2,3]))
        if heights[0]<=0 or any(b<=a for a,b in zip(heights,heights[1:])):
            raise ValueError('Lift waypoint heights must increase')
    elif limits is not None:
        raise ValueError('Bowl branch repair does not add a lift corridor')
    return limits
