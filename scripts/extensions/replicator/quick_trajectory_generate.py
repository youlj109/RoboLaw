#!/usr/bin/env python3
"""Freeze current dashboard scenes, then generate one bounded cuRobo candidate each."""
from __future__ import annotations
import argparse
import fcntl
import gc
import json
import os
from pathlib import Path
import sys
import time
import traceback
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import atomic_json, relative, sha, stamp

SCOPE = 'One-shot nominal cuRobo trajectory generation only. No physical execution, long-term stability, posthoc obstacle-avoidance audit or task-success verification.'
STATES = ('pending', 'running', 'generated', 'failed', 'skipped')


def initialize(output, url):
    document = json.loads(urlopen(url, timeout=15).read())
    visual = document['visual_only_review']['entries']
    placements = {row['scene_id']: row for row in document['visual_placement']['entries']}
    rows = []
    for scene in visual:
        path = scene.get('config_path')
        if not path:
            candidates = [item['url'].removeprefix('/artifact/') for item in placements[scene['scene_id']].get('artifacts', [])
                if item.get('url', '').startswith('/artifact/configs/')]
            path = next((candidate for candidate in candidates if (ROOT/candidate).is_file() and sha(ROOT/candidate) == scene['config_sha256']), None)
        if not path or sha(ROOT/path) != scene['config_sha256']:
            raise ValueError('Cannot resolve exact current scene config: '+scene['scene_id'])
        rows.append({**scene, 'config_path': path, 'status': 'pending', 'stage': 'queued', 'updated_at': stamp(),
            'planning_only': True, 'executed': False, 'task_success_verified': False, 'artifacts': {}})
    if len(rows) != 154 or len({row['scene_id'] for row in rows}) != 154:
        raise ValueError('Expected 154 unique current scenes')
    output.mkdir(parents=True, exist_ok=True)
    atomic_json(output/'scene_snapshot.json', dict(schema_version=1, source_url=url, created_at=stamp(), total=len(rows), entries=rows))
    progress = dict(schema_version=1, total=len(rows), stage='preparing', scope=SCOPE, updated_at=stamp(), entries=rows,
        source_snapshot_path=relative(output/'scene_snapshot.json'), source_snapshot_sha256=sha(output/'scene_snapshot.json'),
        checks_not_performed=['physics_execution','long_term_stability','independent_collision_or_avoidance_audit','physical_grasp','task_success'])
    save_progress(output, progress)
    return progress


def save_progress(output, progress):
    progress['updated_at'] = stamp()
    progress['counts'] = {state: sum(row['status'] == state for row in progress['entries']) for state in STATES}
    atomic_json(output/'progress.json', progress)


def dumpable(value):
    import numpy as np
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {key: dumpable(item) for key, item in value.items()}
    if isinstance(value, (tuple,list)): return [dumpable(item) for item in value]
    return value


def suggest(code, stage):
    if 'START_STATE' in code or 'INVALID_START' in code:
        if stage == 'pregrasp':
            return '布局固定：核对初始关节、坐标变换与碰撞模型映射；若初始碰撞真实存在，仅优化抓点无法消除，应保留失败，不改初态或摆位。'
        return '检查该阶段起点（上一阶段终点）及新启用的闭爪/携物包络；标注抓点、夹爪姿态或携物接触模型需要怎样调整，后续单独重试。'
    if 'IK' in code:
        return '检查失败阶段目标是否可达；标注抓点、夹爪朝向或预抓取/抬升高度的调整方案。'
    if 'GOAL' in code:
        return '检查目标抓点或落点的空间；标注目标高度、朝向和接近方向的调整方案。'
    return '查看失败阶段与已生成前缀；标注需要调整的抓点、姿态、抬升高度或中间路点，后续单独重试。'


def save_path(destination, result, paths, phases, grips, kin, dt):
    import numpy as np
    if not paths:
        return
    q = np.concatenate(paths)
    prefix = '' if result['status'] == 'generated' else 'partial_'
    file = destination/(prefix+'trajectory.npz')
    tcp = np.asarray([kin.fk(value) for value in q])
    phase = np.concatenate([np.full(len(path), label) for path,label in zip(paths,phases)])
    gripper = np.concatenate([np.full(len(path), grip) for path,grip in zip(paths,grips)])
    np.savez_compressed(file, q=q, tcp_world=tcp, phase=phase, gripper=gripper, dt=np.asarray(dt))
    key = 'trajectory' if result['status'] == 'generated' else 'partial_trajectory'
    result['artifacts'][key] = {'path':relative(file),'sha256':sha(file)}
    result.update(trajectory_complete=result['status']=='generated', trajectory_samples=len(q), trajectory_dt=dt,
        partial_trajectory=result['status']!='generated')
    compact = destination/(prefix+'trajectory.json')
    atomic_json(compact, dumpable(dict(schema_version=1,scene_id=result['scene_id'],config_sha256=result['config_sha256'],
        complete=result['status']=='generated',phase_ranges=[dict(phase=label,start=sum(map(len,paths[:index])),stop=sum(map(len,paths[:index+1]))) for index,label in enumerate(phases)],
        targets=result.get('targets'),nominal_gripper_joint_rad=result.get('nominal_gripper_joint_rad'),
        npz_path=relative(file),npz_sha256=sha(file),scope=SCOPE)))
    result['artifacts'][key+'_json'] = {'path':relative(compact),'sha256':sha(compact)}


def run_scene(row, output, timeout, retry_recipe=None):
    import numpy as np
    from pxr import Usd
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.scene_build_runtime import compose
    from kitchen_safety.extensions.replicator.quick_trajectory import PlanningWorld, QuickPlanner, authored_start, task_targets
    destination = output/'scenes'/row['scene_id']
    destination.mkdir(parents=True, exist_ok=True)
    result = {key:row.get(key) for key in ('scene_id','task_id','variant_id','category','config_path','config_sha256')}
    result.update(schema_version=1,status='running',stage='source_composition',scope=SCOPE,planning_only=True,
        executed=False,task_success_verified=False,trajectory_complete=False,artifacts={},phases=[],started_at=stamp(),
        attempts=1,per_stage_max_attempts=1,per_stage_timeout_s=timeout,posthoc_collision_audit=False,
        checks_not_performed=['physics_execution','long_term_stability','avoidance_effectiveness','physical_grasp','task_success'])
    if retry_recipe and retry_recipe.get('scene_adjustment'):
        result.update(scene_adjustment=retry_recipe['scene_adjustment'], scene_settings_changed_once=True,
                      layout_fixed=False, new_layout_frozen=True)
    started, paths, labels, grips, kin, planner = time.monotonic(), [], [], [], None, None
    try:
        config = ROOT/row['config_path']
        if sha(config) != row['config_sha256']:
            raise ValueError('Current config hash changed after snapshot; refusing old or mixed layout')
        layout = json.loads(config.read_text())
        source = destination/'source_scene.usda'
        composition = compose(config, source)
        result['composition'] = composition
        stage = Usd.Stage.Open(str(source))
        kin = DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
        q = authored_start(stage,kin)
        result.update(start_q=q.tolist(),start_tcp_world=kin.fk(q).tolist(),initial_state_source='Actual source-authored USD link/joint transforms, inferred at rendered pose; no physics reset')
        result['stage'] = 'target_preparation'
        world = PlanningWorld(stage)
        assignment = next(task for task in json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())['tasks'] if task['id']==row['task_id'])
        task = task_targets(layout,world,assignment)
        if retry_recipe is not None:
            if retry_recipe.get('target_poses_world'):
                from kitchen_safety.extensions.replicator.quick_trajectory_retry2 import apply_retry_targets
            else:
                from kitchen_safety.extensions.replicator.quick_trajectory_retry import apply_retry_targets
            task = apply_retry_targets(task, layout, world, assignment, retry_recipe)
            result.update(retry_round=row.get('retry_round', 1), retry=task['retry'],
                prior_result_path=row['prior_result_path'], prior_result_sha256=row['prior_result_sha256'])
        world.payload_geometry = task['geometry']
        result.update(targets=dumpable(task['targets']),payload_name=task['payload_name'],
            nominal_gripper_joint_rad=task['nominal_gripper_joint_rad'],nominal_attachment=dumpable(task['attachment']),
            grasp_info=dumpable(task['grasp_info']),world_collision_shapes=len(world.objects),
            planning_contact_policy='Payload object masked at approach/carry; source support only during lift; destination during lower/retreat. Added human/property/robot hazard is never masked. Nominal attachment and closing angle are unverified candidate assumptions.')
        result['stage']='planner_initialization'
        atomic_json(destination/'result.json',dumpable(result))
        planner = QuickPlanner(world,kin,dt=1/30,seed=42,max_joint_speed=.4,use_cuda_graph=False)
        result['planner_backend']='cuRobo MotionGen'
        for phase,target in task['targets'].items():
            result['stage']=phase
            atomic_json(destination/'result.json',dumpable(result))
            if phase in ('lift','transfer','lower'):
                world.attachment=task['attachment']
                world.gripper_joint_position=task['nominal_gripper_joint_rad']
            else:
                world.attachment=None
                world.gripper_joint_position=0.
            masks=[]
            if phase != 'pregrasp':masks.append('/World/objects/'+task['payload_name'])
            if phase == 'lift':
                masks.extend(task.get('source_supports', [task['source_support']] if task['source_support'] else []))
            if phase in ('lower','retreat'):masks.append(task['destination_mask'])
            path,detail=planner.quick_plan(q,target,phase,masks,timeout)
            result['phases'].append(detail)
            if path is None:
                result.update(status='failed',failure_code=detail['solver_status'],failure_reason='cuRobo rejected '+phase+': '+detail['solver_status'])
                break
            paths.append(path);labels.append(phase);grips.append(world.gripper_joint_position);q=path[-1].copy()
        else:
            result.update(status='generated',stage='complete',failure_code=None,failure_reason=None)
    except Exception as error:
        result.update(status='failed',failure_code=type(error).__name__,failure_reason=str(error),traceback=traceback.format_exc())
    finally:
        result.update(updated_at=stamp(),elapsed_seconds=time.monotonic()-started,
            suggestion=suggest(result.get('failure_code') or '',result['stage']) if result['status']=='failed' else '人工查看抓点、抬升、转运、落点和撤离；如需修改，直接记录调整方案。规划生成不代表物理任务成功。')
        if result.get('scene_settings_changed_once'):
            result['suggestion'] = ('本景已完成一次场景调整及一次规划，本轮不再改动或重试。' + result['suggestion'])
        try:
            if kin is not None:
                save_path(destination,result,paths,labels,grips,kin,1/30)
            if sha(ROOT/row['config_path']) != row['config_sha256']:
                result.update(status='failed',failure_code='STALE_CONFIG',failure_reason='Config changed during generation')
        except Exception as error:
            result.update(status='failed',failure_code='ARTIFACT_SAVE_ERROR',failure_reason=str(error),trajectory_complete=False)
        result['failed_phase']=result['stage'] if result['status']=='failed' else None
        atomic_json(destination/'result.json',dumpable(result))
        if planner is not None:
            del planner
        gc.collect()
        try:
            import torch
            torch.cuda.empty_cache()
        except Exception:
            pass
    result['result_path']=relative(destination/'result.json')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'reports/replicator_trajectory_generation_20261026')
    parser.add_argument('--url',default='http://127.0.0.1:8765/api/progress')
    parser.add_argument('--initialize-only',action='store_true')
    parser.add_argument('--limit',type=int)
    parser.add_argument('--phase-timeout',type=float,default=6.)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args();args.output=args.output.resolve()
    progress_path=args.output/'progress.json'
    if progress_path.exists():
        if not args.resume:parser.error('Existing run retained; use --resume (completed scenes never retried)')
        progress=json.loads(progress_path.read_text())
    else:
        progress=initialize(args.output,args.url)
    if args.initialize_only:
        print(json.dumps(dict(total=progress['total'],counts=progress['counts'],output=str(args.output))))
        return
    lock_path=ROOT/'reports/skill_trials/gpu.lock';lock_path.parent.mkdir(parents=True,exist_ok=True)
    with lock_path.open('a+') as lock:
        print('WAITING_GPU_LOCK',flush=True);fcntl.flock(lock,fcntl.LOCK_EX);print('GPU_LOCK_ACQUIRED',flush=True)
        progress['stage']='running';save_progress(args.output,progress)
        count=0
        for row in progress['entries']:
            if row['status'] in ('generated','failed','skipped'):continue
            if args.limit is not None and count>=args.limit:break
            row.update(status='running',stage='planning',updated_at=stamp());save_progress(args.output,progress)
            print('SCENE_START',row['scene_id'],flush=True)
            try:
                result=run_scene(row,args.output,args.phase_timeout)
            except Exception as error:
                destination=args.output/'scenes'/row['scene_id'];destination.mkdir(parents=True,exist_ok=True)
                result=dict(scene_id=row['scene_id'],task_id=row['task_id'],variant_id=row['variant_id'],config_sha256=row['config_sha256'],
                    status='failed',stage='worker_error',failed_phase='worker_error',failure_code=type(error).__name__,failure_reason=str(error),
                    planning_only=True,executed=False,task_success_verified=False,trajectory_complete=False,artifacts={},updated_at=stamp(),
                    result_path=relative(destination/'result.json'),suggestion='查看错误日志，修正运行依赖或输入后另开单独重试。')
                atomic_json(destination/'result.json',result)
            row.update({key:result.get(key) for key in ('status','stage','failed_phase','failure_code','failure_reason','suggestion','elapsed_seconds','updated_at','result_path','artifacts','trajectory_complete','trajectory_samples')})
            save_progress(args.output,progress)
            print('SCENE_DONE',row['scene_id'],row['status'],row['stage'],row.get('failure_code'),json.dumps(progress['counts']),flush=True)
            count+=1
        if not any(row['status'] in ('pending','running') for row in progress['entries']):progress['stage']='complete'
        save_progress(args.output,progress)


if __name__=='__main__':main()
