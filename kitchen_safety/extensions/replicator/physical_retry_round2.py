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
    def constrained_plan(self, q, target, phase, masks, hold_orientation, timeout=8., linear_vertical=False):
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        from curobo.rollout.cost.pose_cost import PoseCostMetric
        for name in self.obstacle_names:
            self.motion_gen.world_coll_checker.enable_obstacle(name,
                enable=not any(self.world._matches(name, mask) for mask in masks))
        self._sync_gripper_and_attachment(self.world.attachment is not None)
        self.motion_gen.reset(reset_seed=True)
        start = JointState.from_position(self.args.to_device(np.asarray(q)[None]),
            joint_names=[f'panda_joint{i}' for i in range(1,8)])
        goal = Pose.from_list(self._pose_list(self.T_B_W @ target @ np.linalg.inv(self.T_ee_tcp)), self.args)
        metric = PoseCostMetric(hold_partial_pose=True,
            hold_vec_weight=self.args.to_device([1.,1.,1.,0.,0.,0.])) if hold_orientation else None
        if linear_vertical:
            metric = PoseCostMetric(hold_partial_pose=True, project_to_goal_frame=False,
                hold_vec_weight=self.args.to_device([1.,1.,1.,1.,1.,0.]))
        begin=time.monotonic()
        output=self.motion_gen.plan_single(start, goal, MotionGenPlanConfig(
            max_attempts=1,timeout=timeout,enable_graph=True,enable_graph_attempt=1,
            enable_finetune_trajopt=True,finetune_attempts=1,ik_fail_return=1,
            pose_cost_metric=metric))
        self.torch.cuda.synchronize()
        ok=bool(output.success.all())
        detail=dict(phase=phase,status='generated' if ok else 'failed',solver_success=ok,
            solver_status='SUCCESS' if output.status is None else str(output.status),
            planning_seconds=time.monotonic()-begin, contact_masks=masks,
            nominal_payload_attached=self.world.attachment is not None,
            orientation_hold_requested=hold_orientation, vertical_motion_cost_requested=linear_vertical,
            physical_grasp_verified=False)
        if not ok: return None,detail
        path=output.get_interpolated_plan().position.detach().cpu().numpy()
        if path.ndim!=2 or path.shape[1]!=7 or not np.isfinite(path).all() or np.max(np.abs(path[0]-q))>1e-4:
            raise ValueError('Invalid cuRobo path or discontinuous start')
        detail.update(samples=len(path),interpolation_dt=float(output.interpolation_dt))
        return path,detail


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
    if np.linalg.norm(final[:2,3]-old_final[:2,3])>.06 or abs(final[2,3]-old_final[2,3])>.06:
        raise ValueError('Release goal moved outside the bounded original destination neighborhood')
    return targets,attachment,grip


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
        atomic_json(folder/'result.json',result)
        planner=GraspRetryPlanner(world,kin,dt=1/30,seed=42,max_joint_speed=.4,use_cuda_graph=False)
        prior_masks={p['phase']:p['contact_masks'] for p in old['phases']}
        for phase in PHASES:
            result['stage']=phase;atomic_json(folder/'result.json',result)
            carrying=phase in ('lift','transfer','lower')
            world.attachment=attachment if carrying else None
            world.gripper_joint_position=planning_grip if carrying else 0.
            masks=prior_masks[phase]
            hold=phase in recipe.get('hold_orientation_phases',['lift','transfer','lower'])
            # Constraints require aligned initial and final orientations. A
            # deliberate grasp/placement reorientation is planned normally.
            rotation_error=Rotation.from_matrix(kin.fk(q)[:3,:3]@targets[phase][:3,:3].T).magnitude()
            hold=bool(hold and rotation_error<.04)
            vertical = phase in recipe.get('linear_motion_phases', [])
            if vertical and (rotation_error >= .04 or np.linalg.norm(kin.fk(q)[:2,3]-targets[phase][:2,3]) > .004):
                raise ValueError('Vertical motion cost requires aligned orientation and XY start')
            path,detail=planner.constrained_plan(q,targets[phase],phase,masks,hold,timeout,linear_vertical=vertical)
            if path is not None and vertical:
                poses=fk_batch(kin,path)
                detail['max_tcp_xy_goal_deviation_m']=float(np.max(np.linalg.norm(poses[:,:2,3]-targets[phase][:2,3],axis=1)))
                detail['vertical_motion_scope']='Soft planner cost; measured TCP deviation, not a physical path guarantee'
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
