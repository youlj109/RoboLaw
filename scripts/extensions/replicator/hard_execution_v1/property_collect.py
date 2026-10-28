#!/usr/bin/env python3
"""Property hard execution: observe, grasp glass, relocate glass, then move bowl.

--dry-run prepares CPU geometry only. Native stages record real state, contacts
and control targets; execute requires glass release before the source bowl task.
Automated execution evidence is separate from final independent acceptance.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))


def write(path,value):
    def convert(x):
        if hasattr(x,'tolist'):return x.tolist()
        if isinstance(x,Path):return str(x)
        raise TypeError(type(x).__name__)
    Path(path).write_text(json.dumps(value,indent=2,default=convert,allow_nan=False)+'\n')


def preflight(args):
    import numpy as np
    from pxr import Usd,UsdPhysics
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_environment import read_config,sha
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_geometry import build_world,grasp_target,source_initial_q,straight_candidate
    from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
    cfg=read_config(args.config);stage,world,kin,pose=build_world(cfg)
    values,source_q=source_initial_q();q=np.asarray([values[f'panda_joint{i}'] for i in range(1,8)])
    initial_valid=world.valid(q,gripper_q=np.zeros(6));initial_failure=world.last_clearance
    accepted=Usd.Stage.Open(str(ROOT/cfg['accepted_scene']));glass=accepted.GetPrimAtPath('/World/hazard/Asset')
    glass_rigid=UsdPhysics.RigidBodyAPI(glass)
    rigid_ok=glass.HasAPI(UsdPhysics.RigidBodyAPI) and glass_rigid.GetRigidBodyEnabledAttr().Get() is not False and glass_rigid.GetKinematicEnabledAttr().Get() is not True
    task=json.loads((ROOT/cfg['accepted_task']).read_text())['specification']
    candidates=[]
    for height in cfg['grasp_candidates']['height_local_m']:
        for yaw in cfg['grasp_candidates']['jaw_yaw_world_deg']:
            row=dict(height_local_m=height,jaw_yaw_world_deg=yaw,passed=False)
            candidates.append(row)
            target=grasp_target(pose,height,yaw,cfg['grasp_candidates'].get('approach','vertical'))
            retreat=float(cfg['grasp_candidates'].get('tcp_retreat_from_center_m',0.))
            if not np.isfinite(retreat) or not 0<=retreat<=.018:raise ValueError('Glass TCP retreat outside tested 0–18 mm')
            target[:3,3]-=target[:3,0]*retreat;above=target.copy()
            above[:3,3]-=target[:3,0]*cfg['grasp_candidates']['pregrasp_height_m']
            try:
                pre_q=kin.ik(above,q,attempts=3);world.set_phase('pregrasp')
                row['pregrasp_valid']=bool(world.valid(pre_q,gripper_q=np.zeros(6)))
                row['pregrasp_failure']=world.last_clearance
                if not row['pregrasp_valid']:continue
                candidate=straight_candidate(kin,pre_q,target)
                world.set_phase('approach');failures=[]
                for index,arm in enumerate(candidate['positions']):
                    if not world.valid(arm,gripper_q=np.zeros(6)):
                        failures.append(dict(sample=index,**world.last_clearance));break
                row.update(approach_samples=len(candidate['positions']),approach_failures=failures,
                    target_tcp=target,pregrasp_tcp=above,pregrasp_q=pre_q,
                    grasp_q=candidate['positions'][-1],passed=not failures)
                if row['passed']:
                    np.savez_compressed(args.output/'glass_approach_cpu_candidate.npz',positions=candidate['positions'],
                        target_tcp=target,pregrasp_tcp=above,initial_q=q,pose_xyz_xyzw=pose)
                    break
            except Exception as e:row['error']=f'{type(e).__name__}: {e}'
        if candidates[-1]['passed']:break
    selected=next((r for r in candidates if r['passed']),None)
    checks=dict(input_hashes_match=True,independent_dynamic_glass=bool(rigid_ok),all_41_original_glass_colliders=len(world.payload_geometry.provenance['colliders'])==41,
        initial_source_geometry_clear=bool(initial_valid),open_gripper_cartesian_approach_candidate=selected is not None)
    report=dict(schema_version=1,task_id=cfg['task_id'],mode='cpu_preflight',checks=checks,
        cpu_preflight_passed=all(checks.values()),config_sha256=sha(args.config),input_sha256=cfg['input_sha256'],
        accepted_scene_unchanged=True,source_initial_q=source_q,initial_failure=initial_failure,
        glass_geometry=world.payload_geometry.provenance,accepted_source_support=task['support'],safe_placement=cfg['safe_placement'],
        grasp_candidates=candidates,selected_grasp=selected,
        original_bowl_controller=dict(profile=cfg['bowl_controller_profile'],support_policy=cfg['bowl_support_policy'],
            reference='Reviewed base/attempt_04 bowl_supported_collect; must execute from newly observed post-glass state'),
        physics_launched=False,cuda_launched=False,physical_execution_verified=False,accepted=False,
        complete_robot_trajectory_generated=False,
        pending=['Native glass/bowl neutral support and settling','cuRobo world/self collision checks and initial-to-pregrasp plan',
            'Observed bilateral glass grasp, lift, stable countertop release','Live-state bowl/sink continuation using original support policy',
            'Independent physical contacts, source predicate, RGB alignment and neutral-duration acceptance'])
    write(args.output/'preflight.json',report)
    print(json.dumps(dict(cpu_preflight_passed=report['cpu_preflight_passed'],selected_grasp=None if selected is None else [selected['height_local_m'],selected['jaw_yaw_world_deg']],output=str(args.output)),indent=2))
    return 0 if report['cpu_preflight_passed'] else 2


def main():
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_environment import DEFAULT_CONFIG
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--stage',choices=['neutral','glass-grasp','glass-transfer','execute'],default='neutral')
    p.add_argument('--duration-seconds',type=float,default=8.)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--device',choices=['cpu'],default='cpu')
    p.add_argument('--physics-threads',type=int,default=2)
    p.add_argument('--torch-threads',type=int,default=2)
    args=p.parse_args();args.config=args.config.resolve();args.output=args.output.resolve()
    if args.duration_seconds<6 or args.duration_seconds>120:p.error('Neutral duration must be in [6,120] seconds')
    args.output.mkdir(parents=True,exist_ok=False)
    if args.dry_run:return preflight(args)
    import fcntl,os
    lock_path=Path('/workspace/reports/skill_trials/gpu.lock')
    if ROOT!=Path('/workspace') or not lock_path.exists():raise RuntimeError('Native run requires verified /workspace mount and shared GPU lock')
    with lock_path.open('a+') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        os.environ['HEADLESS']='1'
        from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
        from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
        cli=get_isaaclab_arena_cli_parser().parse_args(['--enable_cameras','--device',args.device,'--seed',str(args.seed)])
        with SimulationAppContext(cli):
            from kitchen_safety.extensions.replicator.hard_execution_v1.property_runtime import neutral
            code=neutral(args)
            if code:raise RuntimeError('Property native '+args.stage+' did not pass all requested-stage checks; see result.json')
    return 0


if __name__=='__main__':raise SystemExit(main())
