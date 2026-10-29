#!/usr/bin/env python3
"""Finite hazard-only translation candidates, source geometry CPU diagnostics."""
import json,os,sys,time
from pathlib import Path
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import planning_failure_cpu_geometry as geo
from scripts.extensions.replicator.prepare_round4_other_cpu import masks_for,pose_check,read,save,ref
from kitchen_safety.extensions.replicator.physical_retry_round2 import PHASES
OUT=ROOT/'reports/replicator_physical_retry_round7_20261029'
MANIFEST=ROOT/'exports/replicator_physical_retry_round6_20261029/manifest.json'
SIDS=('l_island_banana_bowl__robot_sharp_equipment_housing','u_shape_mustard_bowl__robot_sharp_equipment_housing')

def spheres(probe,q,phase,recipe):
    carrying=phase in ('lift','transfer','lower');grip=recipe['planning_gripper_joint_rad'] if carrying else 0.
    result=geo.robot_spheres(probe.kin,q,grip)
    if carrying:
        t=probe.kin.fk(q)@np.array(recipe['nominal_attachment']);c,r=probe.cover
        result += [('attached_object',i,t[:3,:3]@v+t[:3,3],float(radius)) for i,(v,radius) in enumerate(zip(c,r))]
    return result

def clearance(probe,sphere_rows,masks=(),hazard_only=False):
    fcl=geo.fcl;best={};collisions=[]
    for link,index,center,radius in sphere_rows:
        near=np.flatnonzero(np.linalg.norm(np.maximum(np.maximum(probe.low-center,center-probe.high),0),axis=1)<radius+.15)
        obj=fcl.CollisionObject(fcl.Sphere(radius),fcl.Transform(center))
        for i in near:
            name=probe.world.objects[i][0]
            if hazard_only and not name.startswith('/World/hazard/'):continue
            if any(probe.world._matches(name,mask) for mask in masks):continue
            other=probe.obstacle(i);hit=bool(fcl.collide(obj,other,fcl.CollisionRequest(),fcl.CollisionResult()))
            distance=float(fcl.distance(obj,other,fcl.DistanceRequest(),fcl.DistanceResult()))
            key=(link,name)
            row=dict(link=link,world_object=name,sphere_index=index,clearance_m=distance,intersection=hit)
            if key not in best or distance<best[key]['clearance_m']:best[key]=row
            if hit or distance<.001:collisions.append(row)
    ordered=sorted(best.values(),key=lambda x:x['clearance_m'])
    return dict(minimum_clearance_m=ordered[0]['clearance_m'] if ordered else None,nearest_pairs=ordered[:10],collision_pairs=collisions,
                pairs_within_40mm=[x for x in ordered if x['clearance_m']<.04])

def stage_mode(probe,recipe,phase,masks):
    probe.phase=phase;probe.carrying=phase in ('lift','transfer','lower');probe.grip=recipe['planning_gripper_joint_rad'] if probe.carrying else 0.
    probe.attachment=np.array(recipe['nominal_attachment']);probe.masks=masks[phase]

def original_diagnosis(row,previous,probe,masks):
    recipe=previous['recipe'];phase=previous['failed_phase'];seg=previous['phases'][-1]['segments'][-1]
    q0=np.array(seg['start_q']);goal=np.array(seg['joint_space_goal_q']);mode=phase
    paths=[('nominal_joint_linear',np.linspace(q0,goal,17))]
    with np.load(geo.bound(seg['trajectory']),allow_pickle=False) as z:paths.append(('retained_curobo_curve',z['q'][np.unique(np.linspace(0,len(z['q'])-1,17).astype(int))]))
    output=[]
    for label,qs in paths:
        samples=[]
        for i,q in enumerate(qs):
            s=spheres(probe,q,mode,recipe);c=clearance(probe,s,masks[mode]);h=clearance(probe,s,masks[mode],True)
            samples.append(dict(sample_index=i,world=c,hazard=h))
        output.append(dict(path_kind=label,samples=samples,sampled_world_collision_count=sum(bool(x['world']['collision_pairs']) for x in samples),sampled_hazard_collision_count=sum(bool(x['hazard']['collision_pairs']) for x in samples)))
    return dict(source_segment=seg['trajectory'],phases=phase,checks=output,
      model_scope='Same source robot spheres plus 5mm buffer, attached payload sphere cover and source triangles / primitive AABBs as planner; FCL query and cuRobo mesh SDF numerics may differ. 40mm is configured planner collision-activation distance, not physical margin.',source_planner_backend=ref(ROOT/'kitchen_safety/curobo_backend.py'))

def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA')
    OUT.mkdir(parents=True,exist_ok=True)
    records={r['scene_id']:r for r in read(MANIFEST)['entries']};report=dict(kind='round7_other_original_geometry_diagnosis',source_manifest=ref(MANIFEST),source_script=ref(Path(__file__)),entries=[],new_ik_attempts=0,new_gpu_calls=0,physics_episodes=0)
    for sid in SIDS:
        row=records[sid];previous=read(geo.bound(row['candidate_planning_result']));probe=geo.Probe(row,previous);masks=masks_for(read(ROOT/row['result_path']))
        d=original_diagnosis(row,previous,probe,masks);report['entries'].append(dict(scene_id=sid,candidate_planning_result=row['candidate_planning_result'],config=dict(path=row['config_path'],sha256=row['config_sha256']),diagnosis=d));save(OUT/'other_original_geometry_diagnosis.partial.json',report)
        print(sid,[(x['path_kind'],x['sampled_world_collision_count'],x['sampled_hazard_collision_count']) for x in d['checks']],flush=True);del probe
    save(OUT/'other_original_geometry_diagnosis.json',report)

def set_hazard_delta(probe, originals, delta):
    for i,name,low,high,mesh in originals:
        lo=low+delta;hi=high+delta
        probe.world.objects[i]=(name,None,lo,hi);probe.low[i]=lo;probe.high[i]=hi
        if mesh is not None:probe.world.meshes[name]=(mesh[0]+delta,mesh[1])
        probe.cache.pop(name,None)


def make_recipe(previous,sid):
    recipe=deepcopy(previous['recipe'])
    for key in ('evidence','cpu_preparation_evidence','cpu_endpoint_ik','joint_space_target_q_by_phase','planning_options_by_phase','phase_waypoints_world','previous_candidate_planning_result','latest_actual_result','selected_round4_candidate','selected_round5_candidate','selected_round6_candidate','cpu_screen_passed'):
        recipe.pop(key,None)
    recipe.update(retry_round=7,layout_fixed=True,segment_deviation_limits_m={},hold_orientation_phases=[],linear_motion_phases=[],phase_waypoints_world={},
       planning_path_policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,
        reason_zh='新布局使用默认笛卡尔碰撞检查规划，允许必要绕行；旧5mm直线门槛不是原任务成功判据，取消该门槛，原物理成功判据保持不变。'))
    if 'banana' in sid:
        grasp=np.asarray(recipe['target_poses_world']['approach']);lift=grasp.copy();lift[2,3]=1.25
        transfer=np.asarray(recipe['target_poses_world']['transfer']);transfer[2,3]=1.25
        recipe['target_poses_world']['lift']=lift.tolist();recipe['target_poses_world']['transfer']=transfer.tolist()
    recipe['limitations']='Finite CPU geometry checks only. Default collision-checked Cartesian planning may curve; no straight-line guarantee, physical success or safety acceptance.'
    return recipe


def build_path_samples(probe,recipe,previous,masks):
    # One deterministic continuity chain, no restart or alternative IK branch search.
    q=np.asarray(previous['start_q']);prior=probe.kin.fk(q);rows=[]
    from scipy.spatial.transform import Slerp
    for phase in PHASES:
        ends=[np.asarray(p) for p in recipe.get('phase_waypoints_world',{}).get(phase,[])]+[np.asarray(recipe['target_poses_world'][phase])]
        for segment,goal in enumerate(ends):
            start=prior.copy();fractions=[1.] if phase=='pregrasp' else np.linspace(0,1,13)
            slerp=Slerp([0,1],Rotation.from_matrix([start[:3,:3],goal[:3,:3]]))
            for f in fractions:
                target=goal.copy();target[:3,3]=(1-f)*start[:3,3]+f*goal[:3,3];target[:3,:3]=slerp(float(f)).as_matrix()
                try:q=probe.kin.ik(target,q,attempts=1)
                except RuntimeError as exc:
                    rows.append(dict(phase=phase,segment=segment,fraction=float(f),ik_solved=False,error=str(exc)));break
                fk=probe.kin.fk(q);rows.append(dict(phase=phase,segment=segment,fraction=float(f),ik_solved=True,q_rad=q.tolist(),tcp_world=fk.tolist(),position_error_m=float(np.linalg.norm(fk[:3,3]-target[:3,3])),sphere_rows=spheres(probe,q,phase,recipe)))
            prior=goal
    return rows


def static_checks(probe,hazard,position,yaw):
    from scripts.extensions.replicator.audit_source_layout_geometry import triangles,ray_z
    from kitchen_safety.geometry_audit import static_collision_shapes
    from kitchen_safety.extensions.replicator.scene_variant_builder import separated
    shapes,footprint=hazard.at(position,yaw)
    if not hasattr(probe,'static_source_rows'):
        probe.static_source_rows=static_collision_shapes(probe.stage,'/World/kitchen')+static_collision_shapes(probe.stage,'/World/objects')
        probe.support_meshes=triangles(probe.stage,'/World/kitchen')
    hits=[ray_z(probe.support_meshes,p) for p in footprint]
    support=bool(hits and all(h and -.001<=h['vertical_gap_m']<=.008 for h in hits))
    separated_ok,detail=separated(shapes,probe.static_source_rows,.001)
    return dict(passed=support and separated_ok,support_rays=hits,support_passed=support,source_objects_and_kitchen_separated=separated_ok,separation=detail)


def screen_main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA')
    from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard
    records={r['scene_id']:r for r in read(MANIFEST)['entries']}
    report=dict(kind='round7_other_finite_hazard_layout_cpu_preparation',source_manifest=ref(MANIFEST),source_script=ref(Path(__file__)),source_geometry_helper=ref(Path(geo.__file__)),original_geometry_diagnosis=ref(OUT/'other_original_geometry_diagnosis.json'),candidate_limit_per_scene=8,scene_mutation_scope=['obstacle.position','obstacle.yaw_deg'],gpu_calls=0,physics_episodes=0,entries=[])
    recipes={}
    candidates={'banana':[(.04,.02,0),(.06,.03,0),(.08,.04,0),(.10,.04,0),(.06,.06,0),(.08,.06,0),(.10,.06,0),(.12,.08,0)],
      'mustard':[(0,-.02,0),(0,-.04,0),(.02,-.04,0),(-.02,-.04,0),(0,-.06,0),(.04,-.04,0),(.06,-.02,0),(.08,0,0)]}
    for sid in SIDS:
        row=records[sid];previous=read(geo.bound(row['candidate_planning_result']));probe=geo.Probe(row,previous);masks=masks_for(read(ROOT/row['result_path']));recipe=make_recipe(previous,sid);layout=deepcopy(probe.layout);hazard=Hazard(layout['obstacle'])
        originals=[(i,name,np.array(lo),np.array(hi),deepcopy(probe.world.meshes.get(name))) for i,(name,_,lo,hi) in enumerate(probe.world.objects) if name.startswith('/World/hazard/')]
        samples=build_path_samples(probe,recipe,previous,masks);trials=[]
        print(sid,'IK samples',len(samples),'failed',sum(not x['ik_solved'] for x in samples),flush=True)
        for ci,delta in enumerate(candidates['banana' if 'banana' in sid else 'mustard']):
            delta=np.asarray(delta);pos=np.asarray(layout['obstacle']['position'])+delta;set_hazard_delta(probe,originals,delta)
            static=static_checks(probe,hazard,pos,layout['obstacle']['yaw_deg']);checks=[]
            for s in samples:
                if not s['ik_solved']:continue
                h=clearance(probe,s['sphere_rows'],masks[s['phase']],True)
                checks.append(dict(phase=s['phase'],fraction=s['fraction'],hazard=h))
            # Direct low carry is a relevance counterfactual, not a requested executable path.
            start=np.asarray(recipe['target_poses_world']['approach']);end=np.asarray(recipe['target_poses_world']['lower']);counter=[]
            for f in np.linspace(0,1,21):
                t=start.copy();t[:3,3]=(1-f)*start[:3,3]+f*end[:3,3];payload=t@np.asarray(recipe['nominal_attachment']);c,r=probe.cover
                ss=[('attached_object',i,payload[:3,:3]@v+payload[:3,3],float(radius)) for i,(v,radius) in enumerate(zip(c,r))]
                h=clearance(probe,ss,(),True);counter.append(dict(fraction=float(f),minimum_clearance_m=h['minimum_clearance_m'],collision_pairs=h['collision_pairs']))
            fine=[x for x in checks if x['phase'] in ('approach','lift')]
            gaps=[x['hazard']['minimum_clearance_m'] for x in fine if x['hazard']['minimum_clearance_m'] is not None]
            collision_count=sum(bool(x['hazard']['collision_pairs']) for x in fine)
            trial=dict(index=ci,position=pos.tolist(),yaw_deg=layout['obstacle']['yaw_deg'],delta_position_m=delta.tolist(),translation_norm_m=float(np.linalg.norm(delta)),static=static,path_samples=checks,
               approach_lift_minimum_clearance_m=min(gaps) if gaps else None,approach_lift_collision_samples=collision_count,
               low_direct_carry_counterfactual=counter,low_direct_carry_intersection_samples=sum(bool(x['collision_pairs']) for x in counter))
            trial['candidate_viable']=bool(static['passed'] and collision_count==0 and not any(not x['ik_solved'] for x in samples))
            trials.append(trial);print(sid,ci,'static',static['passed'],'fine_gap',trial['approach_lift_minimum_clearance_m'],'fine_hit',collision_count,'direct_hit',trial['low_direct_carry_intersection_samples'],flush=True)
        # Prefer modest displacement with >=20mm approach/lift clearance. Only the
        # declared eight poses are considered; retain evidence if none meets it.
        valid=[t for t in trials if t['candidate_viable']]
        adequate=[t for t in valid if (t['approach_lift_minimum_clearance_m'] or 999)>=.02]
        chosen=min(adequate or valid,key=lambda t:(t['translation_norm_m'],-(t['approach_lift_minimum_clearance_m'] or 1))) if valid else None
        entry=dict(scene_id=sid,previous_candidate=row['candidate_planning_result'],old_config=dict(path=row['config_path'],sha256=row['config_sha256']),old_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),original_masks=masks,
          cpu_samples=[{k:v for k,v in s.items() if k!='sphere_rows'} for s in samples],trials=trials,selected_index=None if chosen is None else chosen['index'])
        if chosen:
            new=deepcopy(layout);new['obstacle']['position']=chosen['position'];new['obstacle']['yaw_deg']=chosen['yaw_deg'];path=ROOT/'configs/extensions/replicator/round7_layouts_20261029'/f'{sid}.json';path.parent.mkdir(parents=True,exist_ok=True);save(path,new)
            assert {k:v for k,v in new.items() if k!='obstacle'}=={k:v for k,v in layout.items() if k!='obstacle'}
            assert {k:v for k,v in new['obstacle'].items() if k not in ('position','yaw_deg')}=={k:v for k,v in layout['obstacle'].items() if k not in ('position','yaw_deg')}
            recipe.update(scene_id=sid,task_id=row['task_id'],new_layout=ref(path),description='仅平移新增尖角外壳腾出抓取/抬升空间，原任务物品与物理参数不变；在新布局上使用默认碰撞检查的笛卡尔规划，允许绕行。'+('香蕉从原抓取点抬高后转运，低位直搬作为障碍意义对照。' if 'banana' in sid else '芥末瓶保留瓶身抓取，外壳仍贴近接近侧；是否实际绕行由本轮规划与物理结果决定。'))
            entry.update(new_config=ref(path),selected_recipe=deepcopy(recipe));recipes[sid]=recipe
        report['entries'].append(entry);save(OUT/'other_layout_cpu_preparation.partial.json',report)
        del probe
    final=OUT/'other_layout_cpu_preparation.json';save(final,report)
    for recipe in recipes.values():recipe['evidence']=ref(final)
    save(ROOT/'configs/extensions/replicator/physical_retry_round7_other_recipes_20261029.json',dict(kind='physical_retry_round7_other_recipes',recipes=recipes,evidence=ref(final)))
    print('selected',list(recipes),flush=True)



def finalize():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA')
    from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,InitialRobotScreen
    screen=read(OUT/'other_eight_layout_screen.json');records={r['scene_id']:r for r in read(MANIFEST)['entries']};recipes={}
    report=dict(kind='round7_other_frozen_layout_and_recipe_preparation',source_manifest=ref(MANIFEST),source_script=ref(Path(__file__)),source_geometry_helper=ref(Path(geo.__file__)),finite_eight_pose_screen=ref(OUT/'other_eight_layout_screen.json'),original_geometry_diagnosis=ref(OUT/'other_original_geometry_diagnosis.json'),unique_layout_candidates_per_scene=8,additional_layout_candidates=0,rejected_high_carry_check=ref(OUT/'other_high_carry_screen.json'),new_gpu_calls=0,physics_episodes=0,scope='CPU geometry and finite ideal samples only, not physical or safety acceptance.',entries=[])
    for oldentry in screen['entries']:
        sid=oldentry['scene_id'];row=records[sid];previous=read(geo.bound(row['candidate_planning_result']));probe=geo.Probe(row,previous);layout=deepcopy(probe.layout);masks=oldentry['original_masks'];banana='banana' in sid;chosen=oldentry['trials'][7 if banana else 0];recipe=make_recipe(previous,sid)
        if banana:
            lift=np.asarray(recipe['target_poses_world']['lift']);oldlift=np.asarray(previous['recipe']['target_poses_world']['lift']);lift=oldlift.copy();recipe['target_poses_world']['lift']=lift.tolist();recipe['target_poses_world']['transfer']=deepcopy(previous['recipe']['target_poses_world']['transfer'])
            elbow=lift.copy();elbow[2,3]=np.asarray(recipe['target_poses_world']['approach'])[2,3]+.008;recipe['phase_waypoints_world']={'lift':[elbow.tolist()]}
        new=deepcopy(layout);new['obstacle']['position']=chosen['position'];new['obstacle']['yaw_deg']=chosen['yaw_deg'];path=ROOT/'configs/extensions/replicator/round7_layouts_20261029'/f'{sid}.json';path.parent.mkdir(parents=True,exist_ok=True);save(path,new)
        assert {k:v for k,v in new.items() if k!='obstacle'}=={k:v for k,v in layout.items() if k!='obstacle'}
        assert {k:v for k,v in new['obstacle'].items() if k not in ('position','yaw_deg')}=={k:v for k,v in layout['obstacle'].items() if k not in ('position','yaw_deg')}
        originals=[(i,name,np.array(lo),np.array(hi),deepcopy(probe.world.meshes.get(name))) for i,(name,_,lo,hi) in enumerate(probe.world.objects) if name.startswith('/World/hazard/')];set_hazard_delta(probe,originals,np.asarray(chosen['delta_position_m']))
        samples=build_path_samples(probe,recipe,previous,masks);checks=[]
        for s in samples:
            if not s['ik_solved']:checks.append({k:v for k,v in s.items() if k!='sphere_rows'});continue
            stage_mode(probe,recipe,s['phase'],masks);world=probe.check(np.array(s['q_rad']));h=clearance(probe,s['sphere_rows'],masks[s['phase']],True)
            checks.append(dict(**{k:v for k,v in s.items() if k!='sphere_rows'},hazard=h,world=world))
        hz=Hazard(new['obstacle']);robot=InitialRobotScreen();robot.set_layout(new);shapes,_=hz.at(new['obstacle']['position'],new['obstacle']['yaw_deg']);initial=robot.check(shapes)
        fine=[x for x in checks if x['phase'] in ('approach','lift') and x['ik_solved']];gaps=[x['hazard']['minimum_clearance_m'] for x in fine if x['hazard']['minimum_clearance_m'] is not None]
        fine_hits=sum(bool(x['hazard']['collision_pairs']) for x in fine)
        overall_hits=sum(bool(x.get('world',{}).get('unique_world_pairs')) for x in checks)
        description=('香蕉外壳向世界+X平移120mm、+Y平移80mm；保持原抓姿，先低位向外移至一个中间点，再抬升至原已可达高度；转运由默认碰撞规划寻找绕行。低位直接搬运13/21样本穿过外壳，实际规划必须自行避障。' if banana else '芥末瓶外壳沿世界-Y平移20mm，保留原瓶身抓姿；接近/抬升最小源球间隙约25mm，仍在规划40mm激活近邻内。低位直接搬运载荷不与外壳相交，不能宣称该景强制转运绕行。')
        grasp=deepcopy(recipe['grasp_info']);recipe['grasp_info']={'rule':'reuse source screened body grasp; new hazard-only layout','nominal_only':True,'physical_contacts_verified':False,'body_local_tcp_pose':grasp.get('body_local_tcp_pose'),'selected_candidate':grasp.get('selected_candidate'),'source_estimated_contact_angle_rad':recipe['planning_gripper_joint_rad'],'closure_semantics':f"Execution target {recipe['nominal_gripper_joint_rad']} rad; planning contact estimate {recipe['planning_gripper_joint_rad']} rad is not a new physical measurement."}
        recipe.update(scene_id=sid,task_id=row['task_id'],new_layout=ref(path),description=description,layout_fixed=True,frozen_candidate_count=1,cpu_preselection_target_set_count=1)
        entry=dict(scene_id=sid,old_config=oldentry['old_config'],new_config=ref(path),old_source_scene=oldentry['old_source_scene'],previous_candidate=row['candidate_planning_result'],selected_layout_index=chosen['index'],delta_position_m=chosen['delta_position_m'],yaw_delta_deg=0.,only_authorized_json_fields_changed=True,static_source_check=chosen['static'],initial_robot_source_check=initial,original_masks=masks,ideal_path_samples=checks,
           approach_lift_minimum_hazard_clearance_m=min(gaps) if gaps else None,approach_lift_hazard_collision_samples=fine_hits,ideal_world_collision_samples=overall_hits,low_direct_carry_intersection_samples=chosen['low_direct_carry_intersection_samples'],low_direct_carry_counterfactual=chosen['low_direct_carry_counterfactual'],selected_recipe=deepcopy(recipe),interpretation_zh=description,source_scene_generation_api='kitchen_safety.extensions.replicator.scene_build_runtime.compose(new_config, new_source_usd)',limitations='Translation of source meshes/primitive AABBs is exact for unchanged yaw. Ideal CPU samples do not constrain future Cartesian cuRobo curve. New source USD and original asset identity audit must be composed by coordinator before planning.')
        report['entries'].append(entry);recipes[sid]=recipe;print(sid,'initial',initial['passed'],'fine',fine_hits,'world',overall_hits,'mingap',entry['approach_lift_minimum_hazard_clearance_m'],flush=True)
    reportpath=OUT/'other_layout_cpu_preparation.json';save(reportpath,report)
    for r in recipes.values():r['evidence']=ref(reportpath)
    configpath=ROOT/'configs/extensions/replicator/physical_retry_round7_other_recipes_20261029.json';save(configpath,dict(kind='physical_retry_round7_other_recipes',recipes=recipes,evidence=ref(reportpath),layout_changed=True))
    print(json.dumps(dict(recipes=ref(configpath),report=ref(reportpath))),flush=True)

if __name__=='__main__':finalize()
