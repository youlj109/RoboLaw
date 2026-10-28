"""Contact-task waypoint progression from measured TCP, never risk grading.

Joint-perfect settling can be prevented by the intended handle contact. This
wrapper accepts a bounded near-goal TCP for contact phases, then continues the
real robot command sequence. It never claims a grasp/contact/task success and
never writes object poses or external articulation forces.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.task_cartesian_policy import pose_matrix
from kitchen_safety.task_contract import digest
from kitchen_safety.task_review_policy import create as create_base
VERSION='robolaw-contact-task-review-1'


def waypoint_progression(phase,target,actual,q,goal,dq,settings):
    result=dict(accepted=False,phase=phase,criterion='measured TCP contact-task waypoint; not contact/grasp truth')
    if phase not in ('approach','operate'):return dict(result,reason='non-contact control phase')
    try:
        target=np.asarray(target,float);actual=np.asarray(actual,float)
        q=np.asarray(q,float);goal=np.asarray(goal,float);dq=np.asarray(dq,float)
        if target.shape!=(4,4) or actual.shape!=(4,4) or q.shape!=(7,) or goal.shape!=(7,) or dq.shape!=(7,):raise ValueError('shape')
        if not all(np.isfinite(x).all() for x in (target,actual,q,goal,dq)):raise ValueError('nonfinite')
        position=float(np.linalg.norm(target[:3,3]-actual[:3,3]));angle=float(np.linalg.norm(Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).as_rotvec()))
        tracking=float(np.max(np.abs(q-goal)));speed=float(np.max(np.abs(dq)))
        accepted=(position<=settings['position_m'] and angle<=settings['orientation_rad'] and tracking<=settings['joint_tracking_rad'] and speed<=settings['joint_speed_rad_s'])
        return dict(result,accepted=bool(accepted),position_error_m=position,orientation_error_rad=angle,joint_tracking_error_rad=tracking,measured_joint_speed_rad_s=speed,tolerances=dict(settings),reason='bounded measured waypoint' if accepted else 'outside measured waypoint tolerance')
    except (ValueError,TypeError,KeyError) as exc:return dict(result,reason='invalid measured waypoint: '+repr(exc))


def create(*,env,contract,bindings,config):
    if set(config)!={'version','task_sha256','base_config','settle'} or config['version']!=VERSION or config['task_sha256']!=digest(contract):raise ValueError('Explicit task-bound contact-review policy required')
    if contract['family']!='articulated_object':raise ValueError('Contact wrapper is for declared articulated tasks')
    limits=config['settle'];expected={'position_m':.05,'orientation_rad':.25,'joint_tracking_rad':.3,'joint_speed_rad_s':.3}
    if set(limits)!=set(expected) or any(type(limits[k]) not in (int,float) or not math.isfinite(limits[k]) or not 0<limits[k]<=bound for k,bound in expected.items()):raise ValueError('Invalid bounded control-waypoint tolerances')
    base=create_base(env=env,contract=contract,bindings=bindings,config=config['base_config'])
    ids=[env.unwrapped.scene['robot'].joint_names.index('panda_joint'+str(i)) for i in range(1,8)]
    hand=bindings['robot_model']['hand_actor_path'];root_sha=digest(config)
    class Policy:
        def act(self,state,last_result):
            progression=None
            if base.goal is not None and not base.ending and base.tick>=base.steps and base.evidence.get('target_world_matrix') is not None:
                actual=pose_matrix(state['actor_states_world'][hand]);actual[:3,3]+=actual[:3,:3]@np.array([.131,0,0])
                progression=waypoint_progression(base.frame['phase'],base.evidence['target_world_matrix'],actual,np.asarray(state['robot']['q'])[ids],base.goal,np.asarray(state['robot']['dq'])[ids],limits)
                if progression['accepted']:base.goal=None
            command=base.act(state,last_result)
            if command is None:return None
            evidence=command['evidence'];evidence['base_policy_config_sha256']=evidence['config_sha256'];evidence['config_sha256']=root_sha;evidence['policy_version']=VERSION
            if progression is not None:evidence['waypoint_progression']=progression
            evidence['waypoint_progression_is_risk_evaluation']=False
            return command
    return Policy()
