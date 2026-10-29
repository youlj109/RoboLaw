"""One grasp/placement candidate for each remaining physical failure.

The original layout and task predicate remain immutable. cuRobo's virtual
attachment is only a planning model; native execution never attaches objects.
"""
from copy import deepcopy
import gc
import json
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation

from .physical_retry_plan import PHASES, retime_candidate, fk_batch, EXECUTION_SETTINGS
from .quick_trajectory import QuickPlanner
from .quick_trajectory_retry2 import checked_pose
from .scene_build_runtime import ROOT, atomic_json, relative, sha, stamp


class GraspRetryPlanner(QuickPlanner):
    def constrained_plan(self, q, target, phase, masks, hold_orientation, timeout=8., linear_vertical=False, planning_options=None, joint_space_goal_q=None):
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        from curobo.rollout.cost.pose_cost import PoseCostMetric
        for name in self.obstacle_names:
            self.motion_gen.world_coll_checker.enable_obstacle(name,
                enable=not any(self.world._matches(name, mask) for mask in masks))
        self._sync_gripper_and_attachment(self.world.attachment is not None)
        options=checked_planning_options(planning_options or {})
        if options:self._reset_all_seeds()
        else:self.motion_gen.reset(reset_seed=True)
        weight=float(options.get('constraint_weight',1.))
        graph=options.get('enable_graph',True)
        start = JointState.from_position(self.args.to_device(np.asarray(q)[None]),
            joint_names=[f'panda_joint{i}' for i in range(1,8)])
        goal = Pose.from_list(self._pose_list(self.T_B_W @ target @ np.linalg.inv(self.T_ee_tcp)), self.args)
        metric = PoseCostMetric(hold_partial_pose=True,
            hold_vec_weight=self.args.to_device([weight,weight,weight,0.,0.,0.])) if hold_orientation else None
        if linear_vertical:
            metric = PoseCostMetric(hold_partial_pose=True, project_to_goal_frame=False,
                hold_vec_weight=self.args.to_device([weight,weight,weight,weight,weight,0.]))
        begin=time.monotonic()
        config=MotionGenPlanConfig(
            max_attempts=1,timeout=timeout,enable_graph=graph,enable_graph_attempt=1 if graph else None,
            enable_finetune_trajopt=True,finetune_attempts=1,ik_fail_return=1,
            pose_cost_metric=metric)
        if joint_space_goal_q is None:
            output=self.motion_gen.plan_single(start, goal, config)
        else:
            if hold_orientation or linear_vertical or 'constraint_weight' in options or graph:
                raise ValueError('Joint-space branch must not imply Cartesian soft constraints or graph fallback')
            goal_state=JointState.from_position(self.args.to_device(np.asarray(joint_space_goal_q)[None]),
                joint_names=[f'panda_joint{i}' for i in range(1,8)])
            output=self.motion_gen.plan_single_js(start, goal_state, config)
        self.torch.cuda.synchronize()
        ok=bool(output.success.all())
        detail=dict(phase=phase,status='generated' if ok else 'failed',solver_success=ok,
            solver_status='SUCCESS' if output.status is None else str(output.status),
            planning_seconds=time.monotonic()-begin, contact_masks=masks,
            nominal_payload_attached=self.world.attachment is not None,
            orientation_hold_requested=hold_orientation, vertical_motion_cost_requested=linear_vertical,
            physical_grasp_verified=False)
        if options:
            detail.update(planning_options=options,planner_seed_reset='all_including_finetune',
                graph_enabled=graph,graph_fallback_enabled=graph,
                pose_metric_hold_weights=None if metric is None else metric.hold_vec_weight.detach().cpu().tolist())
        if joint_space_goal_q is not None:
            detail.update(planner_api='MotionGen.plan_single_js',joint_space_goal_q=np.asarray(joint_space_goal_q).tolist(),
                joint_space_seed_scope='Frozen continuous-IK goal branch; cuRobo supplies its joint-space seed. No CPU path is executed.')
        if not ok: return None,detail
        path=output.get_interpolated_plan().position.detach().cpu().numpy()
        if path.ndim!=2 or path.shape[1]!=7 or not np.isfinite(path).all() or np.max(np.abs(path[0]-q))>1e-4:
            raise ValueError('Invalid cuRobo path or discontinuous start')
        detail.update(samples=len(path),interpolation_dt=float(output.interpolation_dt))
        return path,detail


def checked_planning_options(options):
    """Only predeclared solver options; no retries or physics changes."""
    if not isinstance(options,dict) or set(options)-{'constraint_weight','enable_graph'}:
        raise ValueError('Unknown frozen planning option')
    if 'constraint_weight' in options:
        value=options['constraint_weight']
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or not 1.<=value<=50.:
            raise ValueError('Invalid frozen constraint weight')
    if 'enable_graph' in options and type(options['enable_graph']) is not bool:
        raise ValueError('Graph option must be Boolean')
    return deepcopy(options)


def validate_recipe(recipe, old, layout):
    if recipe['scene_id']!=old['scene_id'] or recipe['task_id']!=layout['task_id'] or recipe.get('layout_fixed') is not True:
        raise ValueError('Recipe identity/layout mismatch')
    if recipe['payload_name']!=old['payload_name'] or set(recipe['target_poses_world'])!=set(PHASES):
        raise ValueError('Recipe payload or phases mismatch')
    targets={p:checked_pose(recipe['target_poses_world'][p]) for p in PHASES}
    initial=np.asarray(old['targets']['approach'])@np.asarray(old['nominal_attachment'])
    attachment=np.linalg.inv(targets['approach'])@initial
    if not np.allclose(attachment,checked_pose(recipe['nominal_attachment']),atol=1e-7,rtol=0):
        raise ValueError('Recipe nominal grasp attachment inconsistent')
    grip=float(recipe['nominal_gripper_joint_rad'])
    if not 0.<grip<=.785: raise ValueError('Gripper target outside source limits')
    # A release adjustment is bounded around the previous destination; this
    # check does not prove containment, which the unchanged predicate measures.
    old_final=np.asarray(old['targets']['lower'])@np.asarray(old['nominal_attachment'])
    final=targets['lower']@attachment
    region=recipe.get('destination_region_check',{})
    if old['payload_name']=='bowl' and region.get('passed') is True:
        if sha(ROOT/region['evidence_path'])!=region['sha256']:raise ValueError('Sink region evidence changed')
    z_limit=float(recipe.get('destination_nominal_z_limit_m',.06))
    if z_limit != .06:
        permitted={'l_shape_bowl_sink__human_default_mpfb','u_shape_bowl_sink__human_default_mpfb'}
        if (recipe['scene_id'] not in permitted or z_limit != .09 or
                region.get('passed') is not True or sha(ROOT/region['evidence_path'])!=region['sha256']):
            raise ValueError('Expanded nominal TCP compensation requires bound sink geometry evidence')
        evidence=json.loads((ROOT/region['evidence_path']).read_text())
        floor=np.asarray(evidence['sink_floor_world_bounds_m'],dtype=float)
        bowl=np.asarray(evidence['predicted_source_bowl_world_bounds_m'],dtype=float)
        if (evidence.get('passed') is not True or floor.shape!=(2,3) or bowl.shape!=(2,3)
                or not np.isfinite(floor).all() or not np.isfinite(bowl).all()
                or not np.all(bowl[0,:2]>=floor[0,:2]) or not np.all(bowl[1,:2]<=floor[1,:2])
                or abs(bowl[0,2]-floor[1,2]-.003)>1e-6
                or not np.allclose(evidence['nominal_goal_world'],final,atol=1e-7,rtol=0)):
            raise ValueError('Nominal compensation must preserve the measured-reference bowl floor target')
        for field in ('original_config','original_source_scene'):
            binding=evidence[field]
            if sha(ROOT/binding['path'])!=binding['sha256']:raise ValueError('Compensation source geometry changed')
    if np.linalg.norm(final[:2,3]-old_final[:2,3])>.06 or abs(final[2,3]-old_final[2,3])>z_limit:
        raise ValueError('Release goal moved outside the bounded original destination neighborhood')
    waypoints=recipe.get('phase_waypoints_world',{})
    limits=recipe.get('segment_deviation_limits_m',{})
    if not isinstance(waypoints,dict) or not isinstance(limits,dict) or not set(waypoints)<=set(PHASES) or not set(limits)<=set(PHASES):
        raise ValueError('Unknown segment phase')
    if sum(len(points) for points in waypoints.values())>16:
        raise ValueError('Too many frozen intermediate waypoints')
    for phase,points in waypoints.items():
        if not isinstance(points,list) or len(points)>8:raise ValueError('Too many phase waypoints')
        for point in points:checked_pose(point)
    for limit in limits.values():
        if not np.isfinite(float(limit)) or not 0<float(limit)<=.02:raise ValueError('Invalid segment deviation limit')
    options=recipe.get('planning_options_by_phase',{})
    if not isinstance(options,dict) or not set(options)<=set(PHASES):raise ValueError('Unknown planning option phase')
    for option in options.values():checked_planning_options(option)
    joint_targets=recipe.get('joint_space_target_q_by_phase',{})
    if not isinstance(joint_targets,dict) or not set(joint_targets)<=set(PHASES):
        raise ValueError('Unknown joint-space phase')
    for phase,values in joint_targets.items():
        array=np.asarray(values,dtype=float)
        if array.shape!=(len(waypoints.get(phase,[]))+1,7) or not np.isfinite(array).all():
            raise ValueError('Joint targets must match every frozen endpoint')
        if (phase in recipe.get('hold_orientation_phases',['lift','transfer','lower'])
                or phase in recipe.get('linear_motion_phases',[])
                or options.get(phase)!={'enable_graph':False}):
            raise ValueError('Joint-space phase requires explicit no-graph options and no Cartesian soft cost')
    return targets,attachment,grip


def segment_deviation(poses,start,end):
    """Maximum actual FK TCP distance from the finite frozen line segment."""
    xyz=np.asarray(poses)[:,:3,3];a=np.asarray(start)[:3,3];b=np.asarray(end)[:3,3]
    direction=b-a;denom=float(direction@direction)
    t=np.clip((xyz-a)@direction/denom,0.,1.) if denom>1e-20 else np.zeros(len(xyz))
    return float(np.max(np.linalg.norm(xyz-(a+t[:,None]*direction),axis=1)))


def plan_phase(planner,kin,q,target,phase,masks,recipe,folder,timeout):
    """Execute each declared segment once; preserve rejected solver paths."""
    waypoints=recipe.get('phase_waypoints_world',{}).get(phase,[])
    limit=recipe.get('segment_deviation_limits_m',{}).get(phase)
    segmented=bool(waypoints) or limit is not None
    endpoints=[checked_pose(p) for p in waypoints]+[target]
    segments=[];paths=[]
    for index,endpoint in enumerate(endpoints):
        start=kin.fk(q)
        rotation_error=Rotation.from_matrix(start[:3,:3]@endpoint[:3,:3].T).magnitude()
        hold=bool(phase in recipe.get('hold_orientation_phases',['lift','transfer','lower']) and rotation_error<.04)
        vertical=phase in recipe.get('linear_motion_phases',[])
        if vertical and (rotation_error>=.04 or np.linalg.norm(start[:2,3]-endpoint[:2,3])>.004):
            raise ValueError('Vertical motion cost requires aligned orientation and XY start')
        options=recipe.get('planning_options_by_phase',{}).get(phase)
        extra={} if not options else dict(planning_options=options)
        joint_targets=recipe.get('joint_space_target_q_by_phase',{}).get(phase)
        goal_check={}
        if joint_targets is not None:
            goal_q=np.asarray(joint_targets[index],dtype=float)
            if goal_q.shape!=(7,) or not np.isfinite(goal_q).all() or np.any(goal_q<kin.limits[:,0]) or np.any(goal_q>kin.limits[:,1]):
                raise ValueError('Frozen joint goal outside source joint limits')
            goal_fk=kin.fk(goal_q)
            position_error=float(np.linalg.norm(goal_fk[:3,3]-endpoint[:3,3]))
            rotation_error=float(Rotation.from_matrix(goal_fk[:3,:3]@endpoint[:3,:3].T).magnitude())
            if position_error>.001 or rotation_error>.01:
                raise ValueError('Frozen joint goal does not reach the declared TCP endpoint')
            extra['joint_space_goal_q']=goal_q
            goal_check=dict(target_goal_fk_position_error_m=position_error,target_goal_fk_rotation_error_rad=rotation_error)
        path,detail=planner.constrained_plan(q,endpoint,phase,masks,hold,timeout,linear_vertical=vertical,**extra)
        detail.update(goal_check)
        if path is not None:
            poses=fk_batch(kin,path)
            if vertical:
                detail['max_tcp_xy_goal_deviation_m']=float(np.max(np.linalg.norm(poses[:,:2,3]-endpoint[:2,3],axis=1)))
                detail['vertical_motion_scope']='Soft planner cost; measured TCP deviation, not a physical path guarantee'
        if not segmented:return path,detail
        detail.update(segment_index=index,target_world=endpoint.tolist(),start_world=start.tolist(),
            attempts=1,max_attempts=1,start_q=np.asarray(q).tolist(),samples=0 if path is None else len(path),
            phase_sample_start=sum(map(len,paths)),phase_sample_stop=sum(map(len,paths))+(0 if path is None else len(path)))
        if path is not None:
            file=folder/f'{phase}_segment_{index:02d}.npz'
            with file.open('xb') as f:np.savez_compressed(f,q=path,tcp_world=poses)
            detail['trajectory']=dict(path=relative(file),sha256=sha(file))
            deviation=segment_deviation(poses,start,endpoint)
            detail.update(max_tcp_segment_deviation_m=deviation,deviation_limit_m=limit,
                deviation_gate_passed=limit is None or deviation<=float(limit))
            if not detail['deviation_gate_passed']:
                detail.update(status='failed',solver_status_before_gate=detail['solver_status'],solver_status='TCP_SEGMENT_DEVIATION_LIMIT')
        segments.append(detail)
        if path is None or detail.get('deviation_gate_passed') is False:
            parent=dict(detail,segments=segments,failed_segment_index=index,samples=sum(map(len,paths)),
                segment_count_declared=len(endpoints),segment_count_attempted=len(segments),per_segment_max_attempts=1)
            return None,parent
        paths.append(path);q=path[-1].copy()
    parent=dict(segments[-1],segments=segments,samples=sum(map(len,paths)),
        segment_count_declared=len(endpoints),segment_count_attempted=len(segments),per_segment_max_attempts=1)
    # Each complete segment, including its repeated start vertex, is retained.
    return np.concatenate(paths),parent


def run_candidate(row,recipe,output,timeout=8.):
    from pxr import Usd
    from kitchen_safety.planning import DroidKinematics
    from .quick_trajectory import PlanningWorld,task_targets,authored_start
    from scripts.extensions.replicator.quick_trajectory_generate import save_path,dumpable
    folder=output/'scenes'/row['scene_id']; folder.mkdir(parents=True,exist_ok=False)
    old=json.loads((ROOT/row['result_path']).read_text())
    layout=json.loads((ROOT/row['config_path']).read_text())
    result={k:row[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256')}
    result.update(schema_version=2,status='running',stage='initializing',artifacts={},phases=[],
        trajectory_complete=False,planning_only=True,executed=False,task_success_verified=False,
        start_q=old['start_q'],composition=deepcopy(old['composition']),layout_fixed=True,
        candidate_count=1,attempts=1,per_stage_max_attempts=1,started_at=stamp(),
        recipe=recipe,prior_result_path=row['result_path'],prior_result_sha256=row['result_sha256'],
        scope='One frozen grasp/placement candidate; layout and source predicate unchanged; not physical success or safety evidence.')
    paths=[];labels=[];grips=[];planner=None;kin=None;begin=time.monotonic()
    try:
        if 'scene_revision' in row:
            from .scene_revision import validate_scene_revision
            if json.loads((ROOT/row['scene_revision']['path']).read_text()).get('retry_round') == 8:
                from .scene_revision_round8 import validate_scene_revision
            checked=validate_scene_revision(ROOT,row['scene_revision'],expected_scene_id=row['scene_id'],current=row)
            policy=recipe.get('planning_path_policy',{})
            if (policy.get('kind')!='collision_checked_free_path' or policy.get('previous_straight_line_gate_removed') is not True
                    or recipe.get('segment_deviation_limits_m') or recipe.get('joint_space_target_q_by_phase')):
                raise ValueError('Revision must freeze the declared collision-checked free-path policy')
            result.update(scene_revision=deepcopy(row['scene_revision']), layout_changed=True,
                scene_settings_changed_once=True,source_asset_geometry_unchanged=True,
                composition=deepcopy(checked['document']['new_composition']),
                scope='One candidate on a separately versioned added-hazard pose; original task/assets/predicate unchanged. Collision-checked free path; no physical success or safety claim.')
        for path,key in [('config_path','config_sha256'),('result_path','result_sha256'),('source_scene_path','source_scene_sha256')]:
            if sha(ROOT/row[path])!=row[key]: raise ValueError('Frozen source changed: '+path)
        for path,digest in old['composition']['source_paths_sha256'].items():
            if sha(ROOT/str(path).removeprefix('/workspace/'))!=digest: raise ValueError('Source geometry changed')
        stage=Usd.Stage.Open(str(ROOT/row['source_scene_path']))
        kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
        q=authored_start(stage,kin)
        if not np.allclose(q,old['start_q'],atol=1e-8,rtol=0): raise ValueError('Initial q changed')
        targets,attachment,grip=validate_recipe(recipe,old,layout)
        world=PlanningWorld(stage)
        assignment=next(t for t in json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())['tasks'] if t['id']==row['task_id'])
        task=task_targets(layout,world,assignment);world.payload_geometry=task['geometry']
        planning_grip=float(recipe.get('planning_gripper_joint_rad',grip))
        if not 0.<planning_grip<=.785:raise ValueError('Invalid planning grip angle')
        result.update(targets=dumpable(targets),nominal_attachment=dumpable(attachment),
            nominal_gripper_joint_rad=grip,payload_name=old['payload_name'],grasp_info=recipe.get('grasp_info',{}),
            planning_gripper_joint_rad=planning_grip,
            planning_grip_scope='Predicted source-contact angle; commanded closing target is separate, no measured-force guarantee.',
            world_collision_shapes=len(world.objects),collision_geometry_unchanged=True,added_hazard_masks=False)
        if 'scene_revision' in row: result.pop('collision_geometry_unchanged',None)
        atomic_json(folder/'result.json',result)
        planner=GraspRetryPlanner(world,kin,dt=1/30,seed=42,max_joint_speed=.4,use_cuda_graph=False)
        prior_masks={p['phase']:p['contact_masks'] for p in old['phases']}
        for phase in PHASES:
            result['stage']=phase;atomic_json(folder/'result.json',result)
            carrying=phase in ('lift','transfer','lower')
            world.attachment=attachment if carrying else None
            world.gripper_joint_position=planning_grip if carrying else 0.
            masks=prior_masks[phase]
            path,detail=plan_phase(planner,kin,q,targets[phase],phase,masks,recipe,folder,timeout)
            result['phases'].append(detail)
            if path is None:
                result.update(status='failed',failure_reason='cuRobo rejected '+phase+': '+detail['solver_status'],failed_phase=phase)
                break
            paths.append(path);labels.append(phase);grips.append(grip if carrying else 0.);q=path[-1].copy()
        else:result.update(status='generated',stage='complete',failure_reason=None,failed_phase=None)
    except Exception as error:
        result.update(status='failed',failure_reason=repr(error),traceback=traceback.format_exc())
    finally:
        if kin is not None: save_path(folder,result,paths,labels,grips,kin,1/30)
        result.update(finished_at=stamp(),elapsed_seconds=time.monotonic()-begin)
        atomic_json(folder/'result.json',dumpable(result))
        if planner is not None:del planner
        gc.collect()
        try:
            import torch
            torch.cuda.empty_cache()
        except ImportError:pass
    return result,folder/'result.json'


def retime_generated(result,output_path,layout):
    from kitchen_safety.planning import DroidKinematics
    source=dict(np.load(ROOT/result['artifacts']['trajectory']['path'],allow_pickle=False))
    adjusted,validation=retime_candidate(source)
    kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
    adjusted['tcp_world']=fk_batch(kin,adjusted['q'])
    with output_path.open('xb') as f:np.savez_compressed(f,**adjusted)
    return validation
