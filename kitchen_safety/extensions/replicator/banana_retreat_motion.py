"""Explicit free-gripper retreat targets away from the source upper cabinet.

No task object/placement goal moves. Every candidate preserves the requested
vertical rise and current orientation; full CUDA and source FCL remain required.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def retreat_candidates(kin, start_q, requested_goal):
    initial=kin.fk(start_q)
    outward=kin.base[:2,3]-initial[:2,3]
    norm=np.linalg.norm(outward)
    if norm<.05:raise ValueError('Robot-facing retreat direction is ambiguous')
    outward/=norm
    rise=float(requested_goal[2,3]-initial[2,3])
    if rise<.1:raise ValueError('Expected original upward free retreat')
    rows=[]
    # Ordered by extra travel; two small initial lifts avoid sliding the open
    # fingers through the released object before leaving the cabinet footprint.
    for distance in (.08,.12,.16):
        for lift in (.04,.06):
            clear=initial.copy();clear[2,3]+=lift
            outside=clear.copy();outside[:2,3]+=outward*distance
            goal=outside.copy();goal[2,3]=requested_goal[2,3]
            rows.append(dict(id=f'out{round(distance*100):02d}_lift{round(lift*100):02d}',
                outward_direction_xy=outward.tolist(),outward_distance_m=distance,initial_lift_m=lift,
                requested_goal_pose=np.asarray(requested_goal).tolist(),effective_goal_pose=goal,
                waypoints=[clear,outside,goal],retained_vertical_rise_m=rise))
    return rows


def cartesian_retreat_candidate(kin,start_q,description,dt,max_joint_speed):
    initial=kin.fk(start_q)
    path=[np.asarray(start_q).copy()];leg_ends=[]
    for goal in description['waypoints']:
        previous=kin.fk(path[-1]);goal=np.asarray(goal)
        if np.max(np.abs(goal[:3,:3]-initial[:3,:3]))>1e-8:raise ValueError('Retreat orientation must remain fixed')
        count=max(2,int(np.ceil(np.linalg.norm(goal[:3,3]-previous[:3,3])/.002))+1)
        for fraction in np.linspace(0.,1.,count)[1:]:
            pose=previous.copy();pose[:3,3]=previous[:3,3]+fraction*(goal[:3,3]-previous[:3,3])
            q=kin.ik(pose,path[-1],attempts=1)
            if np.max(np.abs(q-path[-1]))>.15:raise RuntimeError('Retreat candidate switched IK branch')
            actual=kin.fk(q)
            if np.linalg.norm(actual[:3,3]-pose[:3,3])>.002:raise RuntimeError('Retreat IK position error')
            if Rotation.from_matrix(actual[:3,:3]@initial[:3,:3].T).magnitude()>np.deg2rad(2):raise RuntimeError('Retreat IK rotation error')
            path.append(q)
        leg_ends.append(len(path)-1)
    path=np.asarray(path)
    segment=np.maximum(3./(len(path)-1),np.max(np.abs(np.diff(path,axis=0)),axis=1)/max_joint_speed)
    knots=np.r_[0.,np.cumsum(segment)]
    times=np.minimum(np.arange(int(np.ceil(knots[-1]/dt))+1)*dt,knots[-1])
    positions=np.stack([np.interp(times,knots,path[:,j]) for j in range(7)],axis=-1)
    poses=np.array([kin.fk(q) for q in positions])
    orientation=Rotation.from_matrix(poses[:,:3,:3]@initial[:3,:3].T).magnitude()
    if orientation.max()>np.deg2rad(2):raise RuntimeError('Retimed retreat orientation left original tolerance')
    return dict(positions=positions,ik_waypoint_positions=path,poses=poses,effective_goal_pose=np.asarray(description['effective_goal_pose']),dt=dt,
        metadata=dict(candidate_id=description['id'],requested_goal_pose=description['requested_goal_pose'],
            effective_goal_pose=np.asarray(description['effective_goal_pose']).tolist(),
            retreat_waypoints=[np.asarray(p).tolist() for p in description['waypoints']],
            initial_lift_m=description['initial_lift_m'],outward_distance_m=description['outward_distance_m'],
            outward_direction_xy=description['outward_direction_xy'],retained_vertical_rise_m=description['retained_vertical_rise_m'],
            task_placement_goal_changed=False,collision_thresholds_changed=False,
            max_orientation_deviation_deg=float(np.rad2deg(orientation.max())),
            minimum_tcp_z_m=float(poses[:,2,3].min()),maximum_tcp_z_m=float(poses[:,2,3].max()),
            start_joint_error_rad=float(np.max(np.abs(positions[0]-start_q))),
            source_ik_leg_end_indices=leg_ends,
            segment_names=["initial_up", "outward", "final_up"],
            source_ik_leg_end_times_s=[float(knots[i]) for i in leg_ends],
            retimed_command_segment_ids=np.searchsorted(knots[np.asarray(leg_ends)],times,side="left").clip(max=2).tolist()))
