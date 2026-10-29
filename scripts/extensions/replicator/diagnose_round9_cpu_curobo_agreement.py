#!/usr/bin/env python3
"""Read-only cuRobo FK/collision checks of already-declared CPU configurations.

No IK/trajectory solve, simulation, candidate change, or acceptance override.
The shared GPU lock covers construction and every diagnostic query.
"""
import fcntl,gc,json,sys,time
from pathlib import Path
import numpy as np
import fcl
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref
OUT=ROOT/'reports/replicator_physical_retry_round9_20261029'
REPORT=OUT/'cpu_curobo_static_agreement.json'
HUMAN='u_shape_bowl_sink__human_default_mpfb'
MUSTARD='u_shape_mustard_bowl__robot_sharp_equipment_housing'


def diagnose(row,result,qs,labels,targets,source):
    from kitchen_safety.extensions.replicator.physical_retry_round2 import GraspRetryPlanner
    from curobo.types.state import JointState
    from curobo.geom.sdf.world import CollisionQueryBuffer
    probe=geo.Probe(row,result);world=probe.world;world.payload_geometry=probe.geometry
    world.attachment=np.array(result['nominal_attachment']);world.gripper_joint_position=result['planning_gripper_joint_rad']
    planner=GraspRetryPlanner(world,probe.kin,dt=1/30,seed=42,max_joint_speed=.4,use_cuda_graph=False,warmup=False)
    # A diagnostic must never silently become a new candidate solve.
    def forbidden(*args,**kwargs):raise RuntimeError('Planning/IK forbidden in static diagnostic')
    planner.motion_gen.plan_single=forbidden;planner.motion_gen.plan_single_js=forbidden
    planner.motion_gen.ik_solver.solve_single=forbidden
    checker=planner.motion_gen.world_coll_checker
    active=[name for name in planner.obstacle_names if not any(world._matches(name,m) for m in probe.masks)]
    for name in planner.obstacle_names:checker.enable_obstacle(name,enable=name in active)
    planner._sync_gripper_and_attachment(True)
    rollout=planner.motion_gen.ik_solver.rollout_fn
    state=rollout._get_augmented_state(JointState(position=planner.args.to_device(qs)))
    spheres=state.robot_spheres
    actual_world=rollout.primitive_collision_constraint.forward(spheres).detach().cpu().numpy().reshape(len(qs),-1).max(1)
    actual_self=rollout.robot_self_collision_constraint.forward(spheres).detach().cpu().numpy().reshape(len(qs),-1).max(1)
    config=rollout.kinematics.kinematics_config
    names={int(v):k for k,v in config.link_name_to_idx_map.items()}
    sphere_links=[names[int(i)] for i in config.link_sphere_idx_map.detach().cpu().tolist()]
    raw_spheres=spheres.detach().cpu().numpy().reshape(len(qs),-1,4)
    world_spheres=raw_spheres.copy();world_spheres[:,:,:3]=raw_spheres[:,:,:3]@probe.kin.base[:3,:3].T+probe.kin.base[:3,3]
    def distance(activation=0.,esdf=False):
        buffer=CollisionQueryBuffer.initialize_from_shape(spheres.shape,planner.args,checker.collision_types)
        return checker.get_sphere_distance(spheres,buffer,planner.args.to_device([1.]),planner.args.to_device([activation]),compute_esdf=esdf).detach().cpu().numpy().reshape(len(qs),-1)
    zero_cost=distance();active_cost=distance(.04);signed_distance=distance(esdf=True)
    positions=state.ee_pos_seq.detach().cpu().numpy().reshape(len(qs),3)
    quats=state.ee_quat_seq.detach().cpu().numpy().reshape(len(qs),4)
    rows=[]
    for i,(q,label,target) in enumerate(zip(qs,labels,targets)):
        actual=np.eye(4);actual[:3,3]=positions[i];actual[:3,:3]=Rotation.from_quat(quats[i,[1,2,3,0]]).as_matrix()
        expected=planner.T_B_W@target@np.linalg.inv(planner.T_ee_tcp)
        cpu_pose=probe.kin.fk(q);actual_tcp=probe.kin.base@actual@planner.T_ee_tcp
        rows.append(dict(label=label,q_rad=q.tolist(),
            source_fk_position_error_m=float(np.linalg.norm(cpu_pose[:3,3]-target[:3,3])),
            curobo_fk_position_error_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
            curobo_fk_rotation_error_rad=float(Rotation.from_matrix(actual[:3,:3]@expected[:3,:3].T).magnitude()),
            cpu_to_gpu_tcp_position_difference_m=float(np.linalg.norm(cpu_pose[:3,3]-actual_tcp[:3,3])),
            cpu_to_gpu_tcp_rotation_difference_rad=float(Rotation.from_matrix(cpu_pose[:3,:3]@actual_tcp[:3,:3].T).magnitude()),
            world_constraint_value=float(actual_world[i]),self_constraint_value=float(actual_self[i]),
            zero_activation_sphere_cost_max=float(zero_cost[i].max()),activation_40mm_cost_max=float(active_cost[i].max()),
            max_signed_penetration_m=float(signed_distance[i].max()),
            positive_signed_sphere_indices=np.flatnonzero(signed_distance[i]>1e-6).tolist(),world_object_pairs=[]))
    # Identify individual geometry contributions without a solve. Restore all
    # phase masks after this diagnostic-only object isolation.
    for name in planner.obstacle_names:checker.enable_obstacle(name,False)
    for name in active:
        checker.enable_obstacle(name,True);signed=distance(esdf=True);cost=distance();checker.enable_obstacle(name,False)
        for i,indices in enumerate(np.flatnonzero(row>-.04) for row in signed):
            for j in indices:
                if raw_spheres[i,j,3]<=0:continue
                sp=world_spheres[i,j];obj=fcl.CollisionObject(fcl.Sphere(float(sp[3])),fcl.Transform(sp[:3]))
                oi=next(k for k,x in enumerate(world.objects) if x[0]==name)
                cpu_gap=float(fcl.distance(obj,probe.obstacle(oi),fcl.DistanceRequest(),fcl.DistanceResult()))
                hit=bool(fcl.collide(obj,probe.obstacle(oi),fcl.CollisionRequest(),fcl.CollisionResult()))
                rows[i]['world_object_pairs'].append(dict(sphere_index=int(j),link=sphere_links[j],world_object=name,sphere_world_xyzr=sp.tolist(),
                    gpu_signed_penetration_m=float(signed[i,j]),gpu_zero_activation_cost=float(cost[i,j]),
                    cpu_fcl_surface_distance_m=cpu_gap,cpu_fcl_surface_intersection=hit,
                    interpretation='GPU positive means inside/overlap; CPU BVH triangle distance is unsigned surface distance and may not detect containment.'))
    for name in planner.obstacle_names:checker.enable_obstacle(name,enable=name in active)
    planner.torch.cuda.synchronize()
    report=dict(scene_id=row['scene_id'],phase=probe.phase,source_cpu_report=source,candidate_planning_result=row['candidate_planning_result'],
        config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        retained_scene_revision=row['scene_revision'],samples=rows,sample_count=len(rows),robot_world_sphere_buffer_m=.005,
        optimizer_collision_activation_m=.04,zero_collision_constraint_separate=True,masks=probe.masks,
        nominal_attachment=result['nominal_attachment'],planning_gripper_joint_rad=result['planning_gripper_joint_rad'],
        gpu_api_calls={'plan_single':0,'plan_single_js':0,'ik_solve':0},physical_episodes=0,
        summaries=dict(world_constraint_nonzero_samples=sum(r['world_constraint_value']>0 for r in rows),self_constraint_nonzero_samples=sum(r['self_constraint_value']>0 for r in rows),
            positive_signed_samples=sum(r['max_signed_penetration_m']>1e-6 for r in rows),max_fk_position_error_m=max(r['curobo_fk_position_error_m'] for r in rows),max_fk_rotation_error_rad=max(r['curobo_fk_rotation_error_rad'] for r in rows)))
    del planner,rollout,state,spheres;gc.collect()
    return report


def main():
    if REPORT.exists():raise FileExistsError(REPORT)
    lock=(ROOT/'reports/skill_trials/gpu.lock').open('a+')
    print('WAITING_GPU_LOCK',flush=True);fcntl.flock(lock,fcntl.LOCK_EX)
    manifest=OUT.parents[1]/'exports/replicator_physical_retry_round9_20261029/manifest.json'
    # ROOT is explicit so diagnostic source never depends on output nesting.
    manifest=ROOT/'exports/replicator_physical_retry_round9_20261029/manifest.json'
    rows={r['scene_id']:r for r in read(manifest)['entries']}
    human_report=OUT/'human_cpu/candidates.json';human=read(human_report);selected=human['trials'][human['selected_index']]
    hresult=read(geo.bound(rows[HUMAN]['candidate_planning_result']))
    cases=[(HUMAN,np.array([selected['lower_endpoint']['q_rad']]),['cpu_selected_lower'],[np.array(hresult['targets']['lower'])],ref(human_report))]
    mustard_report=OUT/'mustard_lift_corridor_cpu_preparation.json';mustard=read(mustard_report)
    qs=[];labels=[];targets=[]
    for branch in mustard['checks']:
        for n,s in enumerate(branch['samples']):
            qs.append(s['q_rad']);labels.append(branch['seed_kind']+f':{n}');targets.append(np.array(s['actual_tcp_world']))
    cases.append((MUSTARD,np.array(qs),labels,targets,ref(mustard_report)))
    report=dict(kind='round9_cpu_curobo_static_agreement',source_manifest=ref(manifest),source_script=ref(Path(__file__)),entries=[],planning_calls=0,physical_episodes=0,kit_started=False)
    for sid,qs,labels,targets,source in cases:
        result=read(geo.bound(rows[sid]['candidate_planning_result']))
        checked=diagnose(rows[sid],result,qs,labels,targets,source);report['entries'].append(checked)
        save(OUT/(sid+'_gpu_static_agreement.json'),checked)
        print(json.dumps(dict(scene_id=sid,samples=checked['sample_count'],**checked['summaries'])),flush=True)
    save(REPORT,report);print(json.dumps(ref(REPORT)),flush=True)

if __name__=='__main__':main()
