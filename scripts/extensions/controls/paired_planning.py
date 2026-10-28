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
from kitchen_safety.extensions.controls.planning import plan_prepress
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance

ROOT=Path(__file__).resolve().parents[3]


def verify_asset_manifests(config):
    """Bind referenced human geometry/textures, not just its USD wrapper."""
    verified={}
    for variant,spec in config['variants'].items():
        manifest=(ROOT/spec['asset']).parent/'artifact_manifest.json'
        if not manifest.exists():
            continue
        data=json.loads(manifest.read_text());files={}
        for name,expected in data['generated_files'].items():
            actual=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            if actual!=expected:
                raise ValueError('Asset manifest mismatch: '+name)
            files[name]=actual
        verified[variant]={'manifest_path':str(manifest.relative_to(ROOT)),
            'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),
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
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    config=json.loads(Path(a.config).read_text())
    (out/'config.json').write_text(json.dumps(config,indent=2))
    source=Path(a.baseline);result=json.loads((source/'result.json').read_text())
    rows=json.loads((source/'records.json').read_text())
    state=next(x['before'] for x in rows if x['phase']=='prepress')
    names=result['robot_joint_names'];ids=[names.index('panda_joint'+str(i)) for i in range(1,8)]
    q=np.asarray(state['q'])[ids];grip=state['q'][names.index('finger_joint')]
    pose=result['initial_robot_pose'];kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    target=np.eye(4);target[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    target[:3,3]=[2.08087763,-.44,1.02713965]
    summary={'scope':'Offline paired MotionGen planning. Identical start, fixed IK branch, original target and control policy per seed. Unsafe hazard-off paths are geometry-only; no unsafe playback.',
        'config_sha256':hashlib.sha256(Path(a.config).read_bytes()).hexdigest(),
        'stage_path':a.stage,'stage_sha256':hashlib.sha256(Path(a.stage).read_bytes()).hexdigest(),
        'hazard_asset_sha256':{k:hashlib.sha256((ROOT/v['asset']).read_bytes()).hexdigest() for k,v in config['variants'].items()},
        'asset_manifest_verification':verify_asset_manifests(config),
        'baseline_state_source':str(source),'source_records_sha256':hashlib.sha256((source/'records.json').read_bytes()).hexdigest(),
        'start_q':q.tolist(),'gripper_joint_position':grip,'requested_tcp':target.tolist(),
        'seeds':a.seeds,'policy':config['prepress_planning'],'runs':[]}
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
            world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
            world.gripper_joint_position=grip
            run={'seed':seed,'variant':variant,'success':False};started=time.monotonic()
            try:
                planner=CuroboPlanner(world,kin,dt=result['dt'],seed=seed,max_joint_speed=.4)
                planner._prepare_path_query((),1,'prepress')
                goal=kin.ik(target,np.asarray(config['prepress_planning']['ik_branch_seed']),attempts=1)
                run['endpoint_curobo_valid']=planner.motion_gen.ik_solver.check_valid(planner.args.to_device(np.array([q,goal]))).detach().cpu().numpy().tolist()
                plan=plan_prepress(planner,q,target,config['prepress_planning'])
                positions=plan['positions'];np.save(out/f'{variant}_seed{seed}_positions.npy',positions)
                validation=planner.validate_path(positions,grip_binary=1,phase='prepress')
                tcp=np.array([kin.fk(x)[:3,3] for x in positions])
                run.update(metadata=plan['metadata'],validation=validation,
                    success=bool(validation['valid']),tcp_path_m=float(np.linalg.norm(np.diff(tcp,axis=0),axis=1).sum()),
                    joint_path_rad=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
                    positions_file=f'{variant}_seed{seed}_positions.npy')
                if key:
                    exact=static_collision_shapes(stage,'/World/envs/env_0/'+key)
                    raw=[]
                    for i,x in enumerate(positions):
                        shapes=[s for s in world.shapes(x,dict(zip(names,state['q']))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
                        raw.append(dict(index=i,**minimum_distance(shapes,exact,world.execution_clearances[key])))
                    run['true_shape_actual']={'minimum':min(raw,key=lambda x:x['distance_m']),'violations':[x for x in raw if not x['valid']],'samples':len(raw)}
                    basefile=out/f'base_seed{seed}_positions.npy'
                    if basefile.exists():
                        raw=[]
                        for i,x in enumerate(np.load(basefile)):
                            shapes=[s for s in world.shapes(x,dict(zip(names,state['q']))) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
                            raw.append(dict(index=i,**minimum_distance(shapes,exact,world.execution_clearances[key])))
                        run['true_shape_counterfactual']={'minimum':min(raw,key=lambda x:x['distance_m']),'violations':[x for x in raw if not x['valid']],'samples':len(raw)}
                del planner
            except Exception as e:
                import traceback
                run['error']=f'{type(e).__name__}: {e}';traceback.print_exc()
            run['seconds']=time.monotonic()-started;summary['runs'].append(run)
            (out/'paired_planning.json').write_text(json.dumps(summary,indent=2))
            print('PAIR_RUN',seed,variant,run['success'],run.get('tcp_path_m'),run.get('error'),flush=True)
    summary['all_plans_valid']=all(x['success'] for x in summary['runs'])
    summary['all_hazard_off_paths_require_adjustment']=all(x.get('true_shape_counterfactual',{}).get('violations') for x in summary['runs'] if x['variant']!='base')
    (out/'paired_planning.json').write_text(json.dumps(summary,indent=2))


if __name__=='__main__':main()
