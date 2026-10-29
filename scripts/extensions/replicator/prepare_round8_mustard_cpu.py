#!/usr/bin/env python3
"""Twelve declared new mustard approach/grasp candidates; CPU only, one frozen.

Original kitchen/robot/payloads/assets/physics/predicate are unchanged. Only the
added housing moves, in a separately saved config. CPU samples are not cuRobo
trajectories or evidence of real grasp, avoidance, or task success.
"""
from copy import deepcopy
import json,os,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import planning_failure_cpu_geometry as geo
from scripts.extensions.replicator.prepare_round2_mustard import Gripper,scene_shapes,read,save
from scripts.extensions.replicator.prepare_round3_mustard import source_screen
from scripts.extensions.replicator.prepare_round4_other_cpu import ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_round7_other_layout_cpu import set_hazard_delta,clearance,spheres,static_checks
from kitchen_safety.extensions.replicator.physical_retry_round2_mustard import make_targets,PHASES
from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,InitialRobotScreen
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
SID='u_shape_mustard_bowl__robot_sharp_equipment_housing'
OUT=ROOT/'reports/replicator_physical_retry_round8_20261029'
MANIFEST=ROOT/'exports/replicator_physical_retry_round7_20261029/manifest.json'
CONFIG=ROOT/'configs/extensions/replicator/round8_layouts_20261029'/f'{SID}.json'
RECIPES=ROOT/'configs/extensions/replicator/physical_retry_round8_mustard_recipes_20261029.json'
REPORT=OUT/'mustard_cpu_preparation.json'
DECLARED=((45,270,0,.035),(55,270,0,.025),(45,240,.010,.030),(55,240,0,.030),
          (45,300,0,.025),(55,300,.010,.025),(45,90,0,.030),(55,90,.010,.030),
          (35,180,.005,.025),(45,180,0,.025),(45,0,0,.025),(55,0,.010,.025))


def declaration(index,values):
    tilt,yaw,z,distance=values;rz=Rotation.from_euler('z',yaw,degrees=True).as_matrix();fixed=np.array([[0.,-1.,0.],[0.,0.,1.],[-1.,0.,0.]])
    local=np.eye(4);local[:3,:3]=rz@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@fixed;local[:3,3]=rz@np.array([0.,-.010,z])
    return dict(index=index,name=f'body_tilt{tilt}_yaw{yaw}_z{z:g}_short{distance:g}',bottle_to_tcp_local=local.tolist(),tilt_deg=tilt,rotation_about_bottle_z_deg=yaw,body_local_tcp_z_m=z,vertical_approach_distance_m=distance,planning_gripper_joint_rad=.30,execution_gripper_joint_rad=.70,hazard_delta_position_m=[0.,-.010,0.])


def finite_path(probe,recipe,previous,masks):
    q=np.asarray(previous['start_q']);prior=probe.kin.fk(q);checks=[]
    for phase in PHASES:
        goal=np.asarray(recipe['target_poses_world'][phase]);start=prior.copy();fractions=[1.] if phase=='pregrasp' else np.linspace(0,1,9)
        rotation=Slerp([0,1],Rotation.from_matrix([start[:3,:3],goal[:3,:3]]))
        for f in fractions:
            target=goal.copy();target[:3,3]=(1-f)*start[:3,3]+f*goal[:3,3];target[:3,:3]=rotation(float(f)).as_matrix()
            check=pose_check(probe,target,q,phase,recipe,masks);check.update(phase=phase,fraction=float(f));checks.append(check)
            if not check['passed']:return dict(passed=False,failed_phase=phase,checks=checks)
            q=np.asarray(check['q_rad']);check['hazard']=clearance(probe,spheres(probe,q,phase,recipe),masks[phase],True)
        prior=goal
    return dict(passed=True,checks=checks,scope='One deterministic CPU IK continuation branch, finite ideal Cartesian samples including source robot and attached payload cover; not actual cuRobo path.')


def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA for CPU-only preparation')
    if REPORT.exists() or RECIPES.exists():raise FileExistsError('Keep frozen round8 outputs')
    OUT.mkdir(parents=True,exist_ok=True)
    row=next(r for r in read(MANIFEST)['entries'] if r['scene_id']==SID);previous=read(geo.bound(row['candidate_planning_result']));probe=geo.Probe(row,previous)
    old=deepcopy(probe.layout);new=deepcopy(old);delta=np.array([0.,-.010,0.]);new['obstacle']['position']=(np.asarray(old['obstacle']['position'])+delta).tolist()
    CONFIG.parent.mkdir(parents=True,exist_ok=True);save(CONFIG,new)
    assert {k:v for k,v in new.items() if k!='obstacle'}=={k:v for k,v in old.items() if k!='obstacle'}
    assert {k:v for k,v in new['obstacle'].items() if k not in ('position','yaw_deg')}=={k:v for k,v in old['obstacle'].items() if k not in ('position','yaw_deg')}
    originals=[(i,name,np.array(lo),np.array(hi),deepcopy(probe.world.meshes.get(name))) for i,(name,_,lo,hi) in enumerate(probe.world.objects) if name.startswith('/World/hazard/')];set_hazard_delta(probe,originals,delta)
    shapes=scene_shapes(geo.bound(dict(path=row['source_scene_path'],sha256=row['source_scene_sha256'])))
    moved=[]
    for name,obj,lo,hi in shapes:
        if name.startswith('/World/hazard/'):
            obj.setTranslation(np.asarray(obj.getTranslation())+delta);lo=lo+delta;hi=hi+delta
        moved.append((name,obj,lo,hi))
    shapes=moved;gripper=Gripper();masks=masks_for(read(ROOT/row['result_path']));hazard=Hazard(new['obstacle']);static=static_checks(probe,hazard,new['obstacle']['position'],new['obstacle']['yaw_deg']);robot=InitialRobotScreen();robot.set_layout(new);hzshapes,_=hazard.at(new['obstacle']['position'],new['obstacle']['yaw_deg']);initial=robot.check(hzshapes)
    report=dict(kind='physical_retry_round8_mustard_cpu_preparation',scene_id=SID,source_manifest=ref(MANIFEST),previous_candidate=row['candidate_planning_result'],previous_scene_revision=row['scene_revision'],old_config=dict(path=row['config_path'],sha256=row['config_sha256']),new_config=ref(CONFIG),old_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),source_script=ref(Path(__file__)),source_geometry_helper=ref(Path(geo.__file__)),source_screen_helper=ref(ROOT/'scripts/extensions/replicator/prepare_round3_mustard.py'),max_declared_cpu_candidates=12,declared_candidates=[declaration(i,v) for i,v in enumerate(DECLARED)],hazard_delta_position_m=delta.tolist(),hazard_yaw_delta_deg=0,layout_changed=True,only_hazard_pose_changed=True,original_masks=masks,source_static_check=static,source_initial_robot_check=initial,gripper_planning_angle_is_measured=False,new_gpu_calls=0,physics_episodes=0,kit_started=False,trials=[])
    selected=None;best=None
    for i,values in enumerate(DECLARED):
        d=declaration(i,values);recipe=make_targets(previous,d);target=np.asarray(recipe['target_poses_world']['approach']);pre=target.copy();pre[2,3]+=d['vertical_approach_distance_m'];recipe['target_poses_world']['pregrasp']=pre.tolist()
        recipe.update(hold_orientation_phases=[],linear_motion_phases=[],phase_waypoints_world={},segment_deviation_limits_m={},layout_fixed=True,retry_round=8,planning_path_policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='新瓶身抓姿配合25–35mm短接近，默认笛卡尔碰撞规划可绕行；不恢复旧额外5mm直线门槛。'))
        trial=dict(candidate=d);trial['source']=source_screen(recipe,shapes,gripper)
        if trial['source']['passed']:
            recipe['planning_gripper_joint_rad']=trial['source']['estimated_contact_angle_rad'];trial['finite_path']=finite_path(probe,recipe,previous,masks)
            score=sum(x['passed'] for x in trial['finite_path']['checks'])
            if best is None or score>best[0]:best=(score,deepcopy(recipe),i)
            if trial['finite_path']['passed'] and static['passed'] and initial['passed']:selected=(recipe,i)
        report['trials'].append(trial);save(OUT/'mustard_cpu_preparation.partial.json',report)
        print(d['name'],'source',trial['source']['passed'],trial['source']['stage'],'path',trial.get('finite_path',{}).get('passed'),trial.get('finite_path',{}).get('failed_phase'),flush=True)
        if selected:break
    if selected is None:
        if best is None:save(OUT/'mustard_all_source_rejected.json',report);raise RuntimeError('All declared source grasp candidates rejected; no candidate frozen')
        selected=(best[1],best[2])
    recipe,index=selected;trial=report['trials'][index]
    recipe.update(scene_id=SID,task_id=row['task_id'],payload_name='mustard_bottle',frozen_candidate_count=1,new_layout=ref(CONFIG),
        description=f"芥末瓶改用{trial['candidate']['tilt_deg']}°俯仰/{trial['candidate']['rotation_about_bottle_z_deg']}°瓶身方位，预抓取到抓取缩短至{trial['candidate']['vertical_approach_distance_m']*1000:g}mm；外壳沿-Y再平移10mm，其他场景值和原任务谓词不变。有限源几何和单分支CPU路径用于筛选，实际cuRobo与物理结果尚未验证。",
        limitations='CPU finite source geometry and one IK continuation branch only; not real grasp, actual cuRobo path, avoidance or safety acceptance.')
    recipe['grasp_info']=dict(rule='new bottle-body tilt/yaw with short vertical approach',selected_candidate=trial['candidate'],body_local_tcp_pose=trial['candidate']['bottle_to_tcp_local'],source_estimated_contact_angle_rad=recipe['planning_gripper_joint_rad'],nominal_only=True,physical_contacts_verified=False,closure_semantics=f"Execution target .70rad; planning estimate {recipe['planning_gripper_joint_rad']}rad is a source geometry contact estimate, not measured physics.")
    validate_recipe(recipe,read(ROOT/row['result_path']),new)
    report.update(selected_index=index,evaluated_cpu_candidates=len(report['trials']),selected_recipe=deepcopy(recipe),selected_cpu_path_passed=trial.get('finite_path',{}).get('passed',False),scope='Source hazard translation preserves the same meshes/primitive AABBs; the coordinator must compose and audit the new USD before GPU planning. No safety or physical success claim.')
    save(REPORT,report);recipe['evidence']=ref(REPORT);save(RECIPES,dict(kind='physical_retry_round8_mustard_recipes',recipes={SID:recipe},evidence=ref(REPORT),layout_changed=True));print(json.dumps(dict(recipe=ref(RECIPES),report=ref(REPORT))),flush=True)

if __name__=='__main__':main()
