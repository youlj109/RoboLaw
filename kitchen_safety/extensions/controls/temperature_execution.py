"""Bounded robot-only contact experiment for the original level-three knob."""
import numpy as np
from scipy.spatial.transform import Rotation
from .planning import plan_control_pose


def execute_temperature(config, planner, kin, state, step, set_phase, arm_ids, on_plan):
    """Callbacks retain the collector's synchronous collision/contact recorder."""
    def q():
        return np.asarray(state()['q'])[arm_ids]
    def command_pose(pose, grip, repeats=1):
        command=kin.ik(pose,q(),attempts=4)
        for _ in range(repeats):step(command,grip=grip)
        return command
    target=np.eye(4)
    target[:3,:3]=Rotation.from_euler('x',config['initial_roll_deg'],degrees=True).as_matrix()
    target[:3,3]=config['pregrasp_tcp_world_m']
    set_phase('pregrasp')
    plan=plan_control_pose(planner,q(),target,config.get('prepress_planning'),grip_binary=0,phase='pregrasp')
    validation=planner.validate_path(plan['positions'],grip_binary=0,phase='pregrasp')
    plan['metadata']['dense_path_validation']=validation
    plan['metadata']['requested_tcp_pose']=target.tolist()
    on_plan(plan['metadata'])
    if not validation['valid']:
        raise RuntimeError('Pregrasp failed independent dense validation')
    for command in plan['positions']:step(command,grip=0)
    for _ in range(8):step(plan['positions'][-1],grip=0)
    set_phase('approach')
    start=target[:3,3].copy()
    goal=np.asarray(config['grasp_tcp_world_m'])
    for fraction in np.linspace(0,1,int(np.ceil(np.linalg.norm(goal-start)/.001))+1)[1:]:
        target[:3,3]=start+(goal-start)*fraction
        command_pose(target,0)
    set_phase('grasp')
    command=q()
    for _ in range(20):step(command,grip=1)
    set_phase('turn')
    reached=False
    for angle in np.arange(1.,config['target_angle_interior_deg']+26.,1.):
        target[:3,:3]=Rotation.from_euler('x',config['initial_roll_deg']+angle,degrees=True).as_matrix()
        command=command_pose(target,1,repeats=2)
        current=state()
        if current['target_joint_position']>=np.deg2rad(config['target_angle_interior_deg']) and current['predicate']:
            reached=True
            break
    if not reached:
        raise RuntimeError('Real contact did not reach original level-three predicate')
    set_phase('hold')
    for _ in range(8):step(command,grip=1)
    set_phase('release')
    for _ in range(15):step(command,grip=0)
    set_phase('retreat')
    start=target[:3,3].copy()
    for distance in np.arange(.002,.062,.002):
        target[:3,3]=start-np.array([distance,0,0])
        command_pose(target,0)
    for _ in range(15):step(q(),grip=0)
    if not state()['predicate']:
        raise RuntimeError('Original level three did not persist after physical release/retreat')
    return plan['metadata']
