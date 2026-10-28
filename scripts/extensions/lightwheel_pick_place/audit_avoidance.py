"""Independent geometry counterfactual and actual-trajectory avoidance audit.

Never execute the unsafe baseline in the augmented physical scene. Raw measured
arm/finger/fruit poses are checked against real added collision geometry, using
the existing execution margins. Trajectory differences alone cannot pass.
"""
import argparse,hashlib,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.exact_primitives import static_collision_shapes,minimum_distance
from scan_avoidance import initial_intersections,fixed_shapes,minimum

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def audit(baseline,episode):
    br=json.loads((baseline/'result.json').read_text());r=json.loads((episode/'result.json').read_text())
    variant=r['variant'];name={'human':'person','property':'glass','robot':'barrier'}[variant]
    base=r['robot_base_pose'];kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2])
    stage=Usd.Stage.Open(str(episode/'scene.usda'));world=CollisionWorld(stage,kin)
    for key,state in r['initial_object_states'].items():world.set_obstacle_pose(key,state[:7])
    t=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/envs/env_0/'+name))).T
    initial_hazard=np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_quat()]
    with h5py.File(episode/'trajectory.h5') as f:
        if variant=='property':initial_hazard=f['obs/hazard_pose'][0]
    world.set_obstacle_pose(name,initial_hazard)
    hazards=[v for v in world.objects if world._matches(v[0],name)]
    if variant=='robot':hazards=static_collision_shapes(stage,'/World/envs/env_0/'+name)
    def hazard_check(q,gripper,lime):
        if variant=='robot':
            return minimum_distance(world.shapes(np.asarray(q),gripper,np.asarray(lime)),hazards,world.execution_clearances[name])
        return world.clearance(q,('lime',),object_pose=lime,obstacle_filter=[name],mode='execution',gripper_q=gripper)
    source=[v for v in world.objects if not world._matches(v[0],name)]+fixed_shapes(stage)
    intersections,supports=initial_intersections(source,hazards)
    initial_goal=[]
    for label,ep in [('baseline',baseline),('augmented',episode)]:
        metadata=json.loads((ep/'result.json').read_text());joints=metadata['robot_joint_names'];arm=[joints.index('panda_joint'+str(i)) for i in range(1,8)]
        with h5py.File(ep/'trajectory.h5') as f:
            for which,group,i in [('initial','obs',0),('goal','next_obs',-1)]:
                q=f[group+'/q'][i];lime=f[group+'/lime'][i,:7]
                row=hazard_check(q[arm],{n:q[joints.index(n)] for n in world.GRIPPER_JOINT_NAMES},lime)
                initial_goal.append({'episode':label,'state':which,**row})
    counterfactual=[]
    baseline_physics=json.loads((baseline/'full_world_audit.json').read_text())['measurements']
    for i,state in enumerate(baseline_physics):
        c=hazard_check(state['q_arm'],state['gripper_q'],state['lime_pose'])
        counterfactual.append({'physics_index':i,'action':state['action'],'phase':state['phase'],**c})
    actual=[]
    actual_physics=json.loads((episode/'full_world_audit.json').read_text())['measurements']
    for i,state in enumerate(actual_physics):
        if variant=='property':
            if 'hazard_pose' not in state:raise ValueError('Independent property replay requires every measured physics hazard pose')
            world.set_obstacle_pose(name,state['hazard_pose'])
        c=hazard_check(state['q_arm'],state['gripper_q'],state['lime_pose'])
        actual.append({'physics_index':i,'action':state['action'],'phase':state['phase'],**c})
    nominal_same=(br['source_yaml_sha256']==r['source_yaml_sha256'] and br['original_asset_initial_poses']==r['original_asset_initial_poses'] and np.allclose(br['robot_base_pose'],r['robot_base_pose'],atol=1e-7,rtol=0))
    # Authored reset equality does not imply identical post-settle dynamics.
    # Arm reset is a hard gate; independently disclose every source rigid-body
    # delta rather than hiding variations behind nominal asset poses.
    with h5py.File(baseline/'trajectory.h5') as bf,h5py.File(episode/'trajectory.h5') as af:
        b_ids=[br['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
        a_ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
        bq=bf['obs/q'][0][b_ids];aq=af['obs/q'][0][a_ids]
        arm_error=float(np.max(np.abs(aq-bq)));arm_same=arm_error<=1e-6
        object_deltas={}
        for key in sorted(set(br['initial_object_states']) & set(r['initial_object_states'])):
            bs=np.asarray(br['initial_object_states'][key]);av=np.asarray(r['initial_object_states'][key])
            # Prefer the synchronized first recorded state where it exists.
            if 'obs/'+key in bf and 'obs/'+key in af:
                bs=bf['obs/'+key][0];av=af['obs/'+key][0];origin='first recorded observation'
            else:origin='measured post-settle state immediately before trajectory'
            delta={'source':origin,'baseline_state':bs.tolist(),'augmented_state':av.tolist(),
                'position_difference_m':float(np.linalg.norm(av[:3]-bs[:3])),
                'orientation_difference_deg':float(np.rad2deg((Rotation.from_quat(av[3:7])*Rotation.from_quat(bs[3:7]).inv()).magnitude()))}
            if len(bs)>=13 and len(av)>=13:
                delta.update(linear_velocity_difference_m_s=float(np.linalg.norm(av[7:10]-bs[7:10])),angular_velocity_difference_rad_s=float(np.linalg.norm(av[10:13]-bs[10:13])))
            object_deltas[key]=delta
    task_pose_limits={'position_m':.001,'orientation_deg':1.}
    task_pose_same=all(object_deltas[key]['position_difference_m']<=task_pose_limits['position_m'] and object_deltas[key]['orientation_difference_deg']<=task_pose_limits['orientation_deg'] for key in ('lime','wooden_bowl'))
    same=nominal_same and arm_same and task_pose_same
    initial_state_comparison={'nominal_source_reset_identical':nominal_same,'measured_arm_reset_matches':arm_same,
        'arm_max_difference_rad':arm_error,'arm_numerical_tolerance_rad':1e-6,'baseline_arm_q':bq.tolist(),'augmented_arm_q':aq.tolist(),
        'measured_task_initial_pose_matches':task_pose_same,'task_initial_pose_limits':task_pose_limits,
        'source_object_deltas':object_deltas,'scope':'Exact authored original poses, measured arm reset (1e-6 rad), robot base (1e-7m), and measured lime/bowl initial pose (1mm/1degree) are gated. All source settling deltas are disclosed; both paths must also pass both measured bowl/distractor backgrounds. Nominal equality does not assert identical physical states. No execution-time object setters.'}
    control_keys=('seed','placement_seed','requested_grasp_tilt_deg','requested_grasp_yaw_deg','requested_grasp_height_offset_m','smooth_lift','allowed_grasp_fallback','selected_grasp')
    same_controls=all(k in br and k in r and br[k] is not None and r[k] is not None and br[k]==r[k] for k in control_keys)
    support_keys=('mode','root_z_gate','source_resting_extra_tolerance_m','source_resting_max_xy_displacement_m','surface','detach_clearance_m')
    baseline_support=br.get('source_support_contact_model',{});actual_support=r.get('source_support_contact_model',{})
    same_support=all(k in baseline_support and k in actual_support and baseline_support[k] is not None and actual_support[k] is not None and baseline_support[k]==actual_support[k] for k in support_keys)
    same_controls=bool(same_controls and same_support)
    def audit_passes(ep,name):
        path=ep/name
        return path.exists() and json.loads(path.read_text()).get('passed') is True
    baseline_gates={n:audit_passes(baseline,n) for n in ('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json')}
    actual_gates={n:audit_passes(episode,n) for n in ('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','fixed_base_hazard_audit.json','source_background_audit.json')}
    background_path=episode/'source_background_audit.json'
    background_report=json.loads(background_path.read_text()) if background_path.exists() else {}
    background_inputs={label:{n:digest(path/n) for n in ('result.json','full_world_audit.json')} for label,path in [('baseline',baseline),('augmented',episode)]}
    background_verified=bool(background_report.get('passed') and background_report.get('input_sha256')==background_inputs)
    background_motion={}
    for label,path,metadata in [('baseline',baseline,br),('augmented',episode,r)]:
        objects={}
        with h5py.File(path/'trajectory.h5') as f:
            for key in ('wooden_bowl','broccoli','sweet_potato','red_bell_pepper'):
                start=f['obs/'+key][0];states=np.concatenate([f['obs/'+key][:],f['next_obs/'+key][:]])
                world.set_obstacle_pose(key,start[:7])
                radius=max(float(np.linalg.norm(np.maximum(np.abs(lo-start[:3]),np.abs(hi-start[:3])))) for n,obj,lo,hi in world.objects if world._matches(n,key))
                translation=np.linalg.norm(states[:,:3]-start[:3],axis=1)
                angles=(Rotation.from_quat(states[:,3:7])*Rotation.from_quat(start[3:7]).inv()).magnitude()
                motion=translation+2*radius*np.sin(angles/2)
                objects[key]={'max_translation_m':float(translation.max()),'max_rotation_deg':float(np.rad2deg(angles.max())),
                    'conservative_collision_radius_m':radius,'max_surface_displacement_bound_m':float(motion.max()),'recorded_states':len(states)}
        bound=max(v['max_surface_displacement_bound_m'] for v in objects.values())
        cases={name:float(row['minimum']['slack_m']-bound) for name,row in background_report.get('combinations',{}).items() if name.endswith('_in_'+label+'_background')}
        background_motion[label]={'objects':objects,'maximum_background_surface_displacement_bound_m':bound,
            'cross_replay_slack_after_motion_bound_m':cases,'passed':bool(len(cases)==2 and all(v>=0 for v in cases.values()))}
    background_verified=bool(background_verified and all(v['passed'] for v in background_motion.values()))
    actual_gates['source_background_inputs_match']=background_verified
    baseline_success=bool(br.get('success') and br.get('final_on_destination') and br.get('final_live_success') and all(baseline_gates.values()))
    cfmin=min(counterfactual,key=lambda v:v['slack_m']);actualmin=min(actual,key=lambda v:v['slack_m'])
    risks=[v for v in counterfactual if not v['valid']];feasible=all(x['valid'] for x in initial_goal);legal=not intersections
    stability={'passed':True,'applicable':variant=='property'}
    if variant=='property':
        poses=np.array([v['hazard_pose'] for v in actual_physics]);displacement=np.linalg.norm(poses[:,:3]-initial_hazard[:3],axis=1)
        ups=Rotation.from_quat(poses[:,3:7]).as_matrix()[:,:,2];up=Rotation.from_quat(initial_hazard[3:7]).as_matrix()[:,2]
        tilts=np.rad2deg(np.arccos(np.clip(ups@up,-1,1)));angles=np.rad2deg((Rotation.from_quat(poses[:,3:7])*Rotation.from_quat(initial_hazard[3:7]).inv()).magnitude())
        stability.update(passed=bool(displacement.max()<=.005 and tilts.max()<=2.),max_translation_m=float(displacement.max()),max_tilt_deg=float(tilts.max()),max_orientation_change_deg=float(angles.max()),translation_limit_m=.005,tilt_limit_deg=2.,samples=len(poses),sample_frequency_hz=120,scope='Real rigid-body stability; no fracture simulation claim.')
    comparison={}
    with h5py.File(baseline/'trajectory.h5') as bf,h5py.File(episode/'trajectory.h5') as af:
        bp=bf['phase'].asstr()[:];ap=af['phase'].asstr()[:]
        for phase in ('pregrasp','transfer'):
            bi=np.flatnonzero(bp==phase);ai=np.flatnonzero(ap==phase)
            if not len(bi) or not len(ai):continue
            samples=np.linspace(0,1,101);metrics={}
            for key in ('q','tcp','lime'):
                left=bf['next_obs/'+key][:][bi];right=af['next_obs/'+key][:][ai]
                dims=7 if key=='q' else 3;left=left[:,:dims];right=right[:,:dims]
                li=np.column_stack([np.interp(samples,np.linspace(0,1,len(left)),left[:,j]) for j in range(dims)])
                ri=np.column_stack([np.interp(samples,np.linspace(0,1,len(right)),right[:,j]) for j in range(dims)])
                d=np.linalg.norm(li-ri,axis=1);metrics[key]={'max_normalized_phase_difference':float(d.max()),'rms_normalized_phase_difference':float(np.sqrt(np.mean(d*d))),'units':'rad' if key=='q' else 'm'}
            comparison[phase]={'baseline_frames':len(bi),'augmented_frames':len(ai),'metrics':metrics}
    changed=any(v['metrics']['q']['max_normalized_phase_difference']>.01 for v in comparison.values())
    effective=bool(risks and all(v['valid'] for v in actual) and changed)
    result={'accepted':bool(same and same_controls and baseline_success and effective and feasible and legal and stability['passed'] and all(actual_gates.values()) and r.get('success') and r.get('final_on_destination')),
        'baseline_episode':str(baseline),'baseline_sha256':{'result.json':digest(baseline/'result.json'),'trajectory.h5':digest(baseline/'trajectory.h5'),'full_world_audit.json':digest(baseline/'full_world_audit.json')},
        'baseline_original_success_verified':baseline_success,'same_source_layout':same,'same_source_layout_scope':initial_state_comparison['scope'],'initial_state_comparison':initial_state_comparison,
        'baseline_audit_gates':baseline_gates,'augmented_audit_gates':actual_gates,
        'source_settling_nuisance_verified':background_verified,
        'source_background_motion_envelopes':{'backgrounds':background_motion,'scope':'Conservative translation + 2*collision-radius*sin(rotation/2) bounds every observed 15Hz source background pose; both 120Hz paths retain original clearance even after subtracting the entire recorded motion envelope. The actual 120Hz full-world audit independently checks live source dynamics.'},
        'same_control_parameters':same_controls,'control_parameters':{k:r.get(k) for k in control_keys},
        'same_source_support_policy':same_support,'source_support_policy':{k:r.get('source_support_contact_model',{}).get(k) for k in support_keys},
        'avoidance_effective':effective,'property_stability':stability,
        'hazard_geometry_scope':'Exact analytic USD Cylinder/Cube/Sphere/Capsule or triangle mesh; no round-primitive AABB corner is used to establish counterfactual risk. Actual collection retains its stricter conservative execution checks.' if variant=='robot' else 'Actual authored hazard collision triangle meshes, with each measured dynamic vase pose during actual execution.',
        'baseline_counterfactual':{'passed':not bool(risks),'violates_safety_margin':bool(risks),'passed_semantics':'True means safe with the added hazard. Effective avoidance requires passed=false and violates_safety_margin=true. Geometry-only, never physically executed.','min_distance_m':cfmin['distance_m'],'required_m':cfmin['required_m'],'violations':len(risks),'samples':len(counterfactual),'sample_frequency_hz':120,'minimum':cfmin,'risk_phases':sorted(set(v['phase'] for v in risks)),'measurements':counterfactual},
        'augmented_actual':{'passed':all(v['valid'] for v in actual),'min_distance_m':actualmin['distance_m'],'required_m':actualmin['required_m'],'samples':len(actual),'sample_frequency_hz':120,'minimum':actualmin,'measurements':actual},
        'initial_and_goal_feasible':feasible,'initial_and_goal_checks':initial_goal,'hazard_environment_legal':legal,'initial_non_support_intersections':intersections,'allowed_support_contacts':supports,
        'path_comparison':{'changed':changed,'scope':'Actual 15 Hz action states, 101 normalized-time samples per unconstrained phase. Changes alone do not establish avoidance; counterfactual violations and complete actual clearance are mandatory.','phases':comparison},
        'hazard_initial_pose':initial_hazard.tolist(),'episode_scene_sha256':digest(episode/'scene.usda'),'episode_trajectory_sha256':digest(episode/'trajectory.h5')}
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',required=True,type=Path);p.add_argument('--episode',required=True,type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    destination=a.output or a.episode/'avoidance_audit.json'
    if destination.exists():raise FileExistsError(f'Preserve historical audit: {destination}; use --output for a new sidecar')
    result=audit(a.baseline,a.episode)
    with destination.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({k:({z:v for z,v in val.items() if z!='measurements'} if isinstance(val,dict) else val) for k,val in result.items()},indent=2));return 0 if result['accepted'] else 2
if __name__=='__main__':raise SystemExit(main())
