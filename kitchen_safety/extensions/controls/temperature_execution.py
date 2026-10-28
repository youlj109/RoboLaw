"""Bounded robot-only contact experiment for the original level-three knob."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .planning import plan_control_pose


def temperature_grasp_rotation(config):
    if 'grasp_rotation_world' not in config:
        return Rotation.from_euler('x',config['initial_roll_deg'],degrees=True).as_matrix()
    matrix=np.asarray(config['grasp_rotation_world'],dtype=float)
    if (matrix.shape!=(3,3) or not np.isfinite(matrix).all() or
            not np.allclose(matrix.T@matrix,np.eye(3),atol=1e-7,rtol=0) or
            not np.isclose(np.linalg.det(matrix),1.,atol=1e-7,rtol=0)):
        raise ValueError('Grasp orientation must be a finite proper world rotation')
    return matrix


def temperature_pregrasp_pose(config):
    """Share the actual configured reference between quick queries and execution."""
    pose=np.eye(4)
    pose[:3,:3]=(temperature_grasp_rotation({'grasp_rotation_world':config['pregrasp_rotation_world']})
        if 'pregrasp_rotation_world' in config else
        Rotation.from_euler('x',config.get('pregrasp_roll_deg',config['initial_roll_deg']),degrees=True).as_matrix())
    pose[:3,3]=config['pregrasp_tcp_world_m']
    return pose


def face_friction_pose(config,angle_deg,advance_m):
    radius=float(config.get('friction_radius_m',.02))
    maximum=float(config.get('friction_max_axial_advance_m',.025))
    minimum=float(config.get('friction_min_axial_advance_m',0.))
    start_angle=float(config.get('friction_start_angle_deg',0.))
    if (not np.isfinite([radius,minimum,maximum,angle_deg,advance_m,start_angle]).all() or
            not 0<radius<=.025 or not -.02<=minimum<=0 or
            not 0<maximum<=.025 or not minimum<=advance_m<=maximum or not -180<=start_angle<=180):
        raise ValueError('Face contact outside declared reference bounds')
    pose=np.eye(4)
    pose[:3,:3]=temperature_grasp_rotation(config)
    angle=np.deg2rad(angle_deg+start_angle)
    pose[:3,3]=np.asarray(config['grasp_tcp_world_m'])+[
        float(config.get('friction_start_x_offset_m',-.02))+advance_m,
        radius*np.cos(angle),radius*np.sin(angle)]
    return pose


def execute_face_friction(config,state,step,set_phase,command_pose,q,contact_load,details):
    """Closed-loop fingertip contact; the knob state is read, never assigned."""
    if contact_load is None:
        raise ValueError('Face friction requires measured target contact load')
    set_phase('close_empty')
    command=q()
    for _ in range(20):step(command,grip=1)
    set_phase('approach')
    for pose in face_friction_connection(np.asarray(state()['tcp']),config):
        command=command_pose(pose,1)
    set_phase('grasp')
    advance=0.
    maximum=float(config.get('friction_max_axial_advance_m',.025))
    minimum=float(config.get('friction_min_axial_advance_m',0.))
    preload=float(config.get('friction_preload_n',1.5))
    for _ in range(int(np.ceil(maximum/.00025))+1):
        command=command_pose(face_friction_pose(config,0.,advance),1,repeats=2)
        load=float(contact_load())
        details.update(axial_advance_m=advance,target_contact_load_n=load)
        if load>=preload:break
        advance=min(maximum,advance+.00025)
    if load<preload:raise RuntimeError('No loaded fingertip/knob face contact within declared axial bound')
    if load>20.:raise RuntimeError('Face preload exceeded 20 N diagnostic force bound')
    set_phase('turn')
    reached=False
    lost=0
    progress_angle=float(np.rad2deg(state()['target_joint_position']))
    stagnant=0
    for index in range(600):
        current=state()
        angle=float(np.rad2deg(current['target_joint_position']))
        if angle>=config['target_angle_interior_deg'] and current['predicate']:
            reached=True
            break
        reference=min(max(0.,angle)+float(config.get('friction_lead_deg',2.)),160.)
        pose=face_friction_pose(config,reference,advance)
        command=command_pose(pose,1)
        load=float(contact_load())
        details.update(turn_actions=index+1,actual_angle_deg=angle,reference_angle_deg=reference,
                       axial_advance_m=advance,target_contact_load_n=load)
        if index%15==0:print('FACE_CONTACT',details,flush=True)
        if angle>progress_angle+.5:
            progress_angle=angle;stagnant=0
        else:stagnant+=1
        if stagnant>=60:raise RuntimeError('Face friction stalled: less than 0.5 degree progress in 60 actions')
        if load>20.:raise RuntimeError('Face contact exceeded 20 N diagnostic force bound')
        lost=lost+1 if load<.01 else 0
        if lost>=10:raise RuntimeError('Lost target face contact during friction turn')
        if load<preload*.5:advance=min(maximum,advance+.0001)
        elif load>preload*2:advance=max(minimum,advance-.0001)
    if not reached:raise RuntimeError('Face contact did not reach the original level-three predicate')
    set_phase('hold')
    if config.get('friction_terminal_policy','hold_then_unload')=='hold_then_unload':
        for _ in range(8):step(command,grip=1)
    elif config['friction_terminal_policy']!='immediate_unload':
        raise ValueError('Unsupported face-contact terminal policy')
    # Remove normal pressure before opening: this is not a pinched grasp.
    set_phase('disengage')
    terminal=np.asarray(state()['tcp'])
    for offset in ([-.03,0.,0.],[0.,0.,.03]):
        for pose in knob_retreat_poses(terminal,offset):
            command=command_pose(pose,1)
        terminal=pose.copy()
    set_phase('release')
    for _ in range(15):step(command,grip=0)
    set_phase('retreat')
    for pose in knob_retreat_poses(terminal,[-.03,0.,.03]):
        command=command_pose(pose,0)
    for _ in range(15):step(command,grip=0)
    if not state()['predicate']:raise RuntimeError('Original level three did not persist after face-contact release')
    details['completed']=True


def knob_turn_pose(grasp_point, grasp_rotation, angle_deg):
    """Rotate about the source knob's world-X axis, not a tilted tool's axis."""
    pose=np.eye(4)
    pose[:3,:3]=Rotation.from_euler('x',angle_deg,degrees=True).as_matrix()@grasp_rotation
    pose[:3,3]=grasp_point
    return pose


def pregrasp_connection(start_pose, grasp_point, grasp_orientation):
    """Reference poses only; measured contact and clearance still gate execution."""
    start=np.array(start_pose,dtype=float,copy=True)
    goal=np.asarray(grasp_point,dtype=float)
    grasp_rotation=(Rotation.from_matrix(grasp_orientation) if np.asarray(grasp_orientation).shape==(3,3)
                    else Rotation.from_euler('x',grasp_orientation,degrees=True))
    rotations=Rotation.from_matrix(np.stack([start[:3,:3],grasp_rotation.as_matrix()]))
    interpolate=Slerp([0.,1.],rotations)
    angular_distance=(rotations[1]*rotations[0].inv()).magnitude()
    segments=max(1,int(np.ceil(np.linalg.norm(goal-start[:3,3])/.001)),
                 int(np.ceil(angular_distance/np.deg2rad(1.))))
    for fraction in np.linspace(0,1,segments+1)[1:]:
        pose=start.copy()
        pose[:3,3]=start[:3,3]+(goal-start[:3,3])*fraction
        pose[:3,:3]=interpolate(fraction).as_matrix()
        yield pose


def face_friction_connection(start_pose, config):
    """Optional free-space alignment before the unchanged face-contact target."""
    goal=face_friction_pose(config,0.,0.)
    start=np.asarray(start_pose,dtype=float)
    if 'friction_connection_world_m' in config:
        points=config['friction_connection_world_m']
        if ('friction_alignment_world_m' in config or not isinstance(points,list)
                or not 1<=len(points)<=2):
            raise ValueError('Declare one connection policy with at most two bounded references')
    else:
        points=[config['friction_alignment_world_m']] if 'friction_alignment_world_m' in config else []
    for point in points:
        point=np.asarray(point,dtype=float)
        anchor=np.asarray(config['grasp_tcp_world_m'],dtype=float)
        limit=(anchor[0]-.02 if 'friction_connection_world_m' in config else goal[0,3]-.02)
        if (point.shape!=(3,) or not np.isfinite(point).all() or
                np.linalg.norm(point-anchor)>.15 or point[0]>limit):
            # These are reference bounds, not collision-distance exemptions.
            raise ValueError('Face alignment must be finite, within 15 cm of the original anchor, and behind its policy reference bound')
        for pose in pregrasp_connection(start,point,goal[:3,:3]):
            yield pose
        start=np.array(goal,copy=True)
        start[:3,3]=point
    yield from pregrasp_connection(start,goal[:3,3],goal[:3,:3])


def knob_retreat_poses(terminal_pose, offset):
    delta=np.asarray(offset,dtype=float)
    if (delta.shape!=(3,) or not np.isfinite(delta).all() or delta[0]>0 or
            abs(delta[1])>1e-9 or delta[2]<0 or np.linalg.norm(delta)>.12):
        raise ValueError('Retreat must withdraw in world X, optionally upward, within 12 cm')
    segments=max(1,int(np.ceil(np.linalg.norm(delta)/.002)))
    for fraction in np.linspace(0,1,segments+1)[1:]:
        pose=np.array(terminal_pose,copy=True)
        pose[:3,3]+=fraction*delta
        yield pose


def execute_temperature(config, planner, kin, state, step, set_phase, arm_ids, on_plan, on_positions=None,contact_load=None):
    """Callbacks retain the collector's synchronous collision/contact recorder."""
    def q():
        return np.asarray(state()['q'])[arm_ids]
    def command_pose(pose, grip, repeats=1):
        command=kin.ik(pose,q(),attempts=4)
        for _ in range(repeats):step(command,grip=grip)
        return command
    target=temperature_pregrasp_pose(config)
    set_phase('pregrasp')
    plan=plan_control_pose(planner,q(),target,config.get('prepress_planning'),grip_binary=0,phase='pregrasp')
    if on_positions is not None:
        on_positions(plan['positions'])
    validation=planner.validate_path(plan['positions'],grip_binary=0,phase='pregrasp')
    validation['controls_planning_gripper_geometry']=config.get('planning_gripper_geometry','full_stroke')
    validation['controls_held_joint_positions']=getattr(planner.world,'planning_gripper_joint_positions',None)
    plan['metadata']['dense_path_validation']=validation
    plan['metadata']['requested_tcp_pose']=target.tolist()
    on_plan(plan['metadata'])
    if not validation['valid']:
        raise RuntimeError('Pregrasp failed independent dense validation')
    for command in plan['positions']:step(command,grip=0)
    for _ in range(8):step(plan['positions'][-1],grip=0)
    if config.get('knob_contact_mode')=='front_face_friction':
        details=dict(mode='front_face_friction',pinched_grasp=False,completed=False,
                     source_knob_axis_world=[1.,0.,0.],target_joint_state_setters=False)
        plan['metadata']['contact_controller']=details
        execute_face_friction(config,state,step,set_phase,command_pose,q,contact_load,details)
        return plan['metadata']
    set_phase('approach')
    grasp_rotation=temperature_grasp_rotation(config)
    for target in pregrasp_connection(target,config['grasp_tcp_world_m'],grasp_rotation):
        command_pose(target,0)
    set_phase('grasp')
    command=q()
    for _ in range(20):step(command,grip=1)
    set_phase('turn')
    reached=False
    for angle in np.arange(1.,config['target_angle_interior_deg']+26.,1.):
        target=knob_turn_pose(config['grasp_tcp_world_m'],grasp_rotation,angle)
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
    for target in knob_retreat_poses(target,config.get('retreat_offset_world_m',[-.06,0.,0.])):
        command_pose(target,0)
    for _ in range(15):step(q(),grip=0)
    if not state()['predicate']:
        raise RuntimeError('Original level three did not persist after physical release/retreat')
    return plan['metadata']
