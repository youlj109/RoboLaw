"""Load one native task and freeze its actual asset inventory without risk grading.

This diagnostic is a stage/mesh binding receipt, never a full trajectory pass.
Run through arena_python.sh with --device cpu --num_envs 1 (no visualizer).
"""
import json
from pathlib import Path
import sys
import time
import traceback
import os
import tempfile
from contextlib import contextmanager

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    parser=get_isaaclab_arena_cli_parser()
    parser.add_argument('--task-contract',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--obstacle-binding',type=Path)
    parser.add_argument('--capture-hold-actions',type=int,default=0,
        help='Diagnostic only: record native DROID hold actions; not task completion')
    parser.add_argument('--policy-factory',help='Repository module:function returning a task policy; native capture only')
    parser.add_argument('--policy-config',type=Path)
    parser.add_argument('--max-actions',type=int,default=3000)
    parser.add_argument('--asset-cache',type=Path,required=True,
        help='Large Lightwheel downloads go here, separately from the system disk')
    args=parser.parse_args()
    # Kit/USD and placement GPU compilation can otherwise fill the system disk.
    # This does not alter HOME or credentials and is scoped to this process.
    for variable,suffix in [('WARP_CACHE_PATH','warp_kernels'),('CUDA_CACHE_PATH','cuda_cache'),('TMPDIR','tmp')]:
        folder=args.asset_cache/suffix;folder.mkdir(parents=True,exist_ok=True)
        os.environ[variable]=str(folder.resolve())
    tempfile.tempdir=None
    if args.num_envs != 1:raise ValueError('Inspect each task separately with one environment')
    args.output.mkdir(parents=True,exist_ok=False)
    import faulthandler,signal
    stack_log=(args.output/'python_stacks.log').open('x')
    faulthandler.register(signal.SIGUSR1,file=stack_log,all_threads=True)
    contract=json.loads(args.task_contract.read_text())
    report=dict(task_id=contract['task_id'],scope='loaded native task, reset and frozen asset inventory',
        full_trajectory_validated=False,risk_evaluated=False,complete=False)
    started=time.perf_counter()
    def write():
        report['wall_s']=time.perf_counter()-started
        (args.output/'inventory_receipt.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    write()
    @contextmanager
    def preserve_failure():
        try:yield
        except BaseException as exc:
            report.update(error=repr(exc),traceback=traceback.format_exc(),complete=False)
            write()
            raise
    env=None
    try:
        with SimulationAppContext(args),preserve_failure():
            import importlib
            for kind in ('floorplan','object'):
                module=importlib.import_module('lightwheel_sdk.loader.'+kind)
                cache=args.asset_cache/kind;cache.mkdir(parents=True,exist_ok=True)
                module.CACHE_PATH=cache
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from kitchen_safety.task_environment import build_environment,resolve_loaded_roots
            from kitchen_safety.task_mesh import freeze_stage
            obstacle=json.loads(args.obstacle_binding.read_text()) if args.obstacle_binding else None
            arena,metadata=build_environment(contract,enable_cameras=False,obstacle=obstacle,asset_cache=args.asset_cache,
                placement_seed=args.placement_seed if args.placement_seed is not None else args.seed)
            report['native_assets']=metadata;write()
            env=ArenaEnvBuilder(arena,ArenaEnvBuilderCfg(device=args.device,presets='physx',
                num_envs=1,solve_relations=True,disable_fabric=True,seed=args.seed,
                placement_seed=args.placement_seed if args.placement_seed is not None else args.seed)).make_registered()
            # One immutable episode: native termination must not start a new
            # placement or clear terminal success before the recorder closes.
            env.unwrapped.configure_episode_limit(1)
            env.reset(seed=args.seed)
            metadata['static_placement_writes']=getattr(env.unwrapped,'_risk_static_placement_writes',{})
            report['random_seeds']=dict(environment=args.seed,
                placement=args.placement_seed if args.placement_seed is not None else args.seed,
                source='explicit native builder/reset seeds; actual draws saved separately')
            stage=env.unwrapped.sim.stage
            roots=resolve_loaded_roots(stage,metadata,'/World/envs/env_0')
            inventory=freeze_stage(stage,'/World/envs/env_0',args.output/'meshes')
            (args.output/'mesh_inventory.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2)+'\n')
            from kitchen_safety.articulation_capture import SceneCapture
            from kitchen_safety.task_binding import derive_bindings,robot_self_pairs,bind_droid_model,bind_operation_contacts,bind_human_surface_omissions
            from kitchen_safety.task_scope import bind_scope
            scene=env.unwrapped.scene
            telemetry=SceneCapture(scene['robot']._physics_sim_view,inventory)
            measured=telemetry.capture()
            required={e['actor'] for e in inventory['colliders'] if e['dynamic']}
            missing=sorted(required-set(measured['actor_states_world']))
            measured.update(missing_actor_paths=missing,complete=measured['complete'] and not missing,
                source_physics_step=-1,source_time_s=0,scope='initial state only; not a trajectory')
            (args.output/'initial_measured_state.json').write_text(json.dumps(measured,ensure_ascii=False,indent=2)+'\n')
            robot_root=str(scene['robot'].root_view.prim_paths[0])
            config=json.loads((ROOT/'configs/safety_observer_arena.json').read_text())
            grip_names=set(config['pt_v4_geometry']['allowed_grip_body_names'])
            body_paths=[str(p) for p in scene['robot'].root_view.link_paths[0]]
            grip_paths=[p for p in body_paths if p.rsplit('/',1)[-1] in grip_names]
            if {p.rsplit('/',1)[-1] for p in grip_paths}!=grip_names:
                raise ValueError('Current DROID grip profile does not bind all declared bodies')
            human_roots=[roots[obstacle['asset_id']]] if obstacle and obstacle['category']=='HS' else []
            bindings=derive_bindings(contract,inventory,roots,robot_root,
                human_roots=human_roots,expected_humans=bool(human_roots),fixed_hs_authorization=True,
                grip_actor_paths=grip_paths,backend_fixed_actor_paths=[p for m in telemetry.articulation.metadata for p in m['fixed_actor_paths']])
            bind_human_surface_omissions(inventory,bindings,obstacle)
            bindings['robot_model']=bind_droid_model(body_paths,scene['robot'].joint_names,config,ROOT)
            semantic_catalog=json.loads((ROOT/'configs/risk_task_semantics.json').read_text())
            if semantic_catalog['schema_version']!='robolaw-task-contact-semantics-1':
                raise ValueError('Unsupported task contact semantics catalog')
            bind_operation_contacts(contract,inventory,bindings,semantic_catalog['tasks'].get(contract['task_id']))
            from kitchen_safety.task_contact_policy import load_policy
            support_policy=load_policy(ROOT,contract)
            from kitchen_safety.task_workspace import load_policy as load_workspace, materialize as materialize_workspace
            workspace_request=load_workspace(ROOT,contract)
            if support_policy is not None:
                from kitchen_safety.task_target_policy import materialize_target_selection
                bindings['pt_normal_support_exclusions']=support_policy if workspace_request else materialize_target_selection(
                    contract,inventory,bindings,measured,args.output/'meshes',support_policy)
            if workspace_request:
                bindings['workspace_policy']=materialize_workspace(contract,inventory,bindings,measured,
                    args.output/'meshes',stage,workspace_request,json.loads((ROOT/config['profile_file']).read_text()))
            scope=bind_scope(contract,inventory,bindings)
            (args.output/'relation_scope.json').write_text(json.dumps(scope,ensure_ascii=False,indent=2)+'\n')
            rs=robot_self_pairs(inventory,robot_root,explicit_structural_pairs=bindings['robot_model']['structural_pairs'])
            (args.output/'rs_pair_scope.json').write_text(json.dumps(rs,ensure_ascii=False,indent=2)+'\n')
            report.update(complete=inventory['complete'] and measured['complete'] and scope['coverage']['scope_resolved'],resolved_asset_roots=roots,
                collider_count=len(inventory['colliders']),joint_count=len(inventory['joints']),
                mesh_errors=inventory['errors'],online_risk_queries=0,
                initial_telemetry_complete=measured['complete'],missing_actor_paths=missing,
                scope_resolved=scope['coverage']['scope_resolved'],scope_unresolved=scope['unresolved'],
                physics_hz=1/env.unwrapped.cfg.sim.dt,action_hz=1/(env.unwrapped.cfg.sim.dt*env.unwrapped.cfg.decimation))
            # The pinned SimulationAppContext exits the process on teardown;
            # persist the receipt while the application is still alive.
            write()
            if not report['complete']:raise ValueError('Incomplete stage inventory, measured state or task scope')
            if args.capture_hold_actions and args.policy_factory:raise ValueError('Choose hold diagnostic or task policy')
            if args.policy_factory:
                from kitchen_safety.task_capture import TaskRawRecorder
                module,sep,function=args.policy_factory.partition(':')
                if not sep or not module.startswith('kitchen_safety.'):
                    raise ValueError('Policy factory must identify an explicit kitchen_safety module:function')
                policy_config=json.loads(args.policy_config.read_text()) if args.policy_config else {}
                factory=getattr(importlib.import_module(module),function)
                policy=factory(env=env,contract=contract,bindings=bindings,config=policy_config)
                recorder=TaskRawRecorder(env,args.output/'raw_episode',contract,inventory,scope,rs,config,args.output/'meshes')
                (recorder.directory/'capture_context.json').write_text(json.dumps(dict(random_seeds=report['random_seeds'],native_assets=metadata),indent=2)+'\n')
                (recorder.directory/'policy_binding.json').write_text(json.dumps(dict(factory=args.policy_factory,config=policy_config),indent=2)+'\n')
                execution_error=None;task_success=None;last_result=None;stop_reason="action_limit"
                try:
                    for _ in range(args.max_actions):
                        command=policy.act(recorder.latest,last_result)
                        if command is None:
                            stop_reason="controller_stopped";break
                        if not isinstance(command,dict) or not {'action','phase'}<=command.keys():
                            raise ValueError('Policy must emit actual action and semantic phase')
                        last_result=recorder.step(command['action'],phase=command['phase'],operation_id=command.get('operation_id'),
                            controller_evidence=command.get('evidence'),diagnostic_actuation=command.get('diagnostic_actuation'))
                        if recorder.terminal:
                            stop_reason="environment_terminal";break
                    task_success=(recorder.last_action_outcome or {}).get('success')
                except BaseException as exc:execution_error=repr(exc);raise
                finally:recorder.close(task_success=task_success,execution_error=execution_error,stop_reason=stop_reason)
                report['native_capture']=dict(actions=recorder.action_index,physics_steps=recorder.sink.steps,
                    task_success=task_success,policy_ended=stop_reason=="controller_stopped",stop_reason=stop_reason,full_task_execution_verified=False,
                    evaluation_status='pending_offline')
                write()
            elif args.capture_hold_actions:
                import torch
                from kitchen_safety.task_capture import TaskRawRecorder
                if contract['roles']['robot']['registry_name']!='droid_abs_joint_pos':
                    raise ValueError('The diagnostic hold policy only binds DROID absolute joint actions')
                robot=scene['robot'];arm=[robot.joint_names.index('panda_joint'+str(i)) for i in range(1,8)]
                action=torch.zeros((1,8),device=env.unwrapped.device)
                action[0,:7]=robot.data.joint_pos[0,arm].clone()
                recorder=TaskRawRecorder(env,args.output/'raw_hold',contract,inventory,scope,rs,config,args.output/'meshes')
                (recorder.directory/'capture_context.json').write_text(json.dumps(dict(random_seeds=report['random_seeds'],native_assets=metadata),indent=2)+'\n')
                try:
                    for _ in range(args.capture_hold_actions):
                        recorder.step(action,phase='settle',controller_evidence={'kind':'diagnostic_hold_not_task_execution'})
                        if recorder.terminal:break
                finally:recorder.close(task_success=None)
                report['hold_probe']=dict(actions=recorder.action_index,physics_steps=recorder.sink.steps,
                    full_task_execution_verified=False)
                write()
            env.close();env=None
    except Exception as exc:
        report.update(error=repr(exc),traceback=traceback.format_exc())
    finally:write()
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return 0 if report['complete'] else 1


if __name__=='__main__':raise SystemExit(main())
