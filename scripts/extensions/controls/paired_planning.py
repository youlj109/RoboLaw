"""Paired hazard-toggle planning. Unsafe counterfactuals are never executed."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.extensions.controls.planning import plan_prepress, plan_control_pose, button_prepress_pose
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
from kitchen_safety.extensions.controls.execution_contract import controller_signature, paired_coverage_gates as paired_gates
from kitchen_safety.extensions.controls.temperature_execution import temperature_pregrasp_pose
from scripts.extensions.controls.render_first_violation import label_robot_hulls

ROOT=Path(__file__).resolve().parents[3]


def verify_asset_manifests(config):
    """Bind referenced human geometry/textures, not just its USD wrapper."""
    verified={}
    for variant,spec in config['variants'].items():
        manifest=(ROOT/spec['asset']).parent/'artifact_manifest.json'
        binding=config.get('human_pose_revision',{}) if variant=='human' else {}
        if not manifest.exists():
            if binding.get('manifest'):raise ValueError('Declared human manifest is missing')
            continue
        digest=hashlib.sha256(manifest.read_bytes()).hexdigest()
        if binding.get('manifest') and (ROOT/binding['manifest']).resolve()!=manifest.resolve():
            raise ValueError('Human manifest path differs from the selected asset')
        if binding.get('manifest_sha256') and digest!=binding['manifest_sha256']:
            raise ValueError('Declared human manifest hash differs')
        data=json.loads(manifest.read_text());files={}
        if not data.get('generated_files'):raise ValueError('Empty asset manifest')
        for name,expected in data['generated_files'].items():
            actual=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            if actual!=expected:
                raise ValueError('Asset manifest mismatch: '+name)
            files[name]=actual
        verified[variant]={'manifest_path':str(manifest.relative_to(ROOT)),
            'manifest_sha256':digest,
            'verified_files':files,'passed':True}
    return verified


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',required=True)
    p.add_argument('--baseline',default='datasets/avoidance_trials/controls/human_attempt02')
    p.add_argument('--stage',default='reports/skill_trials/controls/inspection/stage.usda')
    p.add_argument('--output',required=True)
    p.add_argument('--seeds',type=int,nargs='+',default=[42,43,44])
    p.add_argument('--variants',nargs='+',default=['base','human','property','robot'])
    p.add_argument('--one-shot',action='store_true',help='One external MotionGen query per seed/variant, without retrying failures')
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    if len(set(a.seeds))!=len(a.seeds) or len(set(a.variants))!=len(a.variants):
        p.error('Seeds and variants must be unique')
    if not a.variants or a.variants[0]!='base' or not set(a.variants)<={'base','human','property','robot'}:
        p.error('Variants must start with base and use supported names')
    config=json.loads(Path(a.config).read_text())
    (out/'config.json').write_text(json.dumps(config,indent=2))
    source=Path(a.baseline);result=json.loads((source/'result.json').read_text())
    rows=json.loads((source/'records.json').read_text())
    baseline_config=json.loads((source/'config.json').read_text())
    if not result.get('accepted') or controller_signature(baseline_config)!=controller_signature(config):
        raise ValueError('Paired query requires a successful baseline with identical controller parameters')
    temperature='turn_oven_temperature_knob' in result['task_id']
    phase='pregrasp' if temperature else 'prepress'
    grip_binary=0 if temperature else 1
    state=next(x['before'] for x in rows if x['phase']==phase)
    names=result['robot_joint_names'];ids=[names.index('panda_joint'+str(i)) for i in range(1,8)]
    q=np.asarray(state['q'])[ids];grip=state['q'][names.index('finger_joint')]
    pose=result['initial_robot_pose'];kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    if result['task_id']!=config['task_id']:
        raise ValueError('Paired baseline/config task mismatch')
    parameters=result['control_parameters']
    if temperature:
        target=temperature_pregrasp_pose(config)
        policy=config.get('prepress_planning',{'method':'curobo_pose'})
        if parameters.get('prepress_planning')!=policy:
            raise ValueError('Temperature paired policy differs from physical baseline')
    else:
        target=button_prepress_pose(config,parameters['roll_deg'],parameters['tcp_x_offset_m'],parameters['tcp_z_offset_m'])
        policy=config['prepress_planning']
    summary={'scope':'Offline paired MotionGen planning. Identical start, fixed IK branch, original target and control policy per seed. Unsafe hazard-off paths are geometry-only; no unsafe playback.',
        'config_sha256':hashlib.sha256(Path(a.config).read_bytes()).hexdigest(),
        'stage_path':a.stage,'stage_sha256':hashlib.sha256(Path(a.stage).read_bytes()).hexdigest(),
        'hazard_asset_sha256':{k:hashlib.sha256((ROOT/v['asset']).read_bytes()).hexdigest() for k,v in config['variants'].items()},
        'asset_manifest_verification':verify_asset_manifests(config),
        'baseline_state_source':str(source),'source_records_sha256':hashlib.sha256((source/'records.json').read_bytes()).hexdigest(),
        'start_q':q.tolist(),'gripper_joint_position':grip,'requested_tcp':target.tolist(),
        'seeds':a.seeds,'policy':policy,'runs':[]}
    summary['one_shot']=a.one_shot
    summary['motiongen_policy']=dict(external_queries_per_seed_condition=1,
        sdk_max_attempts=1 if a.one_shot else 4,
        graph_enable_attempt=None if a.one_shot else 2,
        scope='Quick graph-first diagnostic' if a.one_shot else 'Unchanged physical collector SDK policy; internal attempts are not manual scene retries')
    summary['variants']=a.variants
    summary['requested_endpoint_scope']='Configured precontact only; adapted pregrasps are not proof of the original grasp or full manipulation.'
    if temperature:
        summary['scope']='Offline paired MotionGen planning: identical start, original TCP, open gripper, seed and physical baseline policy. Fixed joint branch when explicitly configured; otherwise redundant pose endpoints may differ. Unsafe hazard-off paths are geometry only.'
    for seed in a.seeds:
        for variant in a.variants:
            stage=Usd.Stage.CreateInMemory();stage.GetRootLayer().subLayerPaths.append(str((ROOT/a.stage).resolve()))
            key={'human':'person','property':'glass','robot':'barrier'}.get(variant)
            if key:
                spec=config['variants'][variant]
                prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+key)
                prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
                prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
                prim.AddRotateZOp().Set(spec['yaw_deg'])
            world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0',
                fixture_cylinder_segments=config.get('planning_fixture_cylinder_segments',1))
            world.gripper_joint_position=grip
            def robot_shapes(x):
                raw=[s for s in world.shapes(x,dict(zip(names,state['q']))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
                return label_robot_hulls(raw,[body for body,_,_ in world.robot_hulls])
            if config.get('planning_gripper_geometry')=='held_articulated':
                from kitchen_safety.extensions.controls.collision import held_gripper_pose
                world.planning_gripper_joint_positions=held_gripper_pose(names,state['q'],state['dq'],0. if temperature else np.pi/4)
            run={'seed':seed,'variant':variant,'success':False,
                 'fixture_geometry_refinement':world.fixture_geometry_refinement};started=time.monotonic()
            try:
                planner=CuroboPlanner(world,kin,dt=result['dt'],seed=seed,max_joint_speed=.4)
                if a.one_shot:
                    from scripts.extensions.controls.quick_plan_scene import motiongen_query_once
                    planner._prepare_path_query((),grip_binary,phase)
                    mode='pose_goal' if policy['method']=='curobo_pose' else 'joint_goal'
                    goal=None if mode=='pose_goal' else kin.ik(target,np.asarray(policy['ik_branch_seed']),attempts=1)
                    run.update(motiongen_calls=1,attempts_limit=1,backend=mode)
                    query=motiongen_query_once(planner,q,target,goal,mode)
                    planner.torch.cuda.synchronize()
                    run.update(sdk_success=bool(query.success.all()),sdk_status=str(query.status))
                    if not run['sdk_success']:raise RuntimeError('One-shot query failed: '+str(query.status))
                    raw=query.get_interpolated_plan().position.detach().cpu().numpy()
                    plan=dict(positions=planner.retime_path(raw,source_dt=float(query.interpolation_dt)),
                        metadata=dict(one_external_query=True,internal_ik_seeds=32 if mode=='pose_goal' else 0))
                elif temperature:
                    plan=plan_control_pose(planner,q,target,policy,grip_binary=0,phase=phase)
                else:
                    planner._prepare_path_query((),1,phase)
                    goal=kin.ik(target,np.asarray(config['prepress_planning']['ik_branch_seed']),attempts=1)
                    run['endpoint_curobo_valid']=planner.motion_gen.ik_solver.check_valid(planner.args.to_device(np.array([q,goal]))).detach().cpu().numpy().tolist()
                    plan=plan_prepress(planner,q,target,config['prepress_planning'])
                positions=plan['positions'];np.save(out/f'{variant}_seed{seed}_positions.npy',positions)
                validation=planner.validate_path(positions,grip_binary=grip_binary,phase=phase)
                tcp=np.array([kin.fk(x)[:3,3] for x in positions])
                run.update(metadata=plan['metadata'],validation=validation,
                    success=bool(validation['valid']),tcp_path_m=float(np.linalg.norm(np.diff(tcp,axis=0),axis=1).sum()),
                    joint_path_rad=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
                    positions_file=f'{variant}_seed{seed}_positions.npy')
                if key:
                    exact=static_collision_shapes(stage,'/World/envs/env_0/'+key)
                    raw=[]
                    for i,x in enumerate(positions):
                        shapes=robot_shapes(x)
                        raw.append(dict(index=i,**minimum_distance(shapes,exact,world.execution_clearances[key])))
                    run['true_shape_actual']={'minimum':min(raw,key=lambda x:x['distance_m']),'violations':[x for x in raw if not x['valid']],'samples':len(raw)}
                    basefile=out/f'base_seed{seed}_positions.npy'
                    if basefile.exists():
                        raw=[]
                        for i,x in enumerate(np.load(basefile)):
                            shapes=robot_shapes(x)
                            raw.append(dict(index=i,**minimum_distance(shapes,exact,world.execution_clearances[key])))
                        minimum=min(raw,key=lambda x:x['distance_m'])
                        run['true_shape_counterfactual']={'minimum':minimum,'violations':[x for x in raw if not x['valid']],'samples':len(raw),
                            'robust_violation':minimum['distance_m']<=minimum['required_m']-.005,
                            'initial_and_goal_safe':raw[0]['valid'] and raw[-1]['valid']}
                del planner
            except Exception as e:
                import traceback
                run['error']=f'{type(e).__name__}: {e}';traceback.print_exc()
            run['seconds']=time.monotonic()-started;summary['runs'].append(run)
            (out/'paired_planning.json').write_text(json.dumps(summary,indent=2))
            print('PAIR_RUN',seed,variant,run['success'],run.get('tcp_path_m'),run.get('error'),flush=True)
    summary.update(paired_gates(summary['runs'],a.seeds,a.variants))
    (out/'paired_planning.json').write_text(json.dumps(summary,indent=2))


if __name__=='__main__':main()
