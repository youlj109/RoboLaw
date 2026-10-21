import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
args=get_isaaclab_arena_cli_parser().parse_args()
with SimulationAppContext(args):
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.environment import KitchenSafetyCfg,KitchenSafetyEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    env=ArenaEnvBuilder(KitchenSafetyEnvironment().build(KitchenSafetyCfg(variant='base')),ArenaEnvBuilderCfg(device=args.device,presets='physx',num_envs=1,solve_relations=False,disable_fabric=True)).make_registered()
    env.reset()
    def arr(v): return (v.torch if hasattr(v,'torch') else v).detach().cpu().numpy()
    robot=env.unwrapped.scene['robot']
    ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    q=arr(robot.data.joint_pos)[0,ids]
    action=torch.tensor([list(q)+[0]],device=env.unwrapped.device)
    for _ in range(30): env.step(action)
    cfg=json.loads(Path('/workspace/configs/layout.json').read_text())
    base=np.array(cfg['robot_position']);base[2]+=cfg['stand_height']
    kin=DroidKinematics(base,cfg['robot_yaw_deg'])
    q=arr(robot.data.joint_pos)[0,ids]
    from pxr import Usd,UsdGeom,UsdPhysics
    stage=env.unwrapped.sim.stage
    bb=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy])
    for prim in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and 'Robot' not in str(prim.GetPath()):
            bounds=bb.ComputeWorldBound(prim).ComputeAlignedRange();lo=bounds.GetMin();hi=bounds.GetMax()
            if lo[0]<2.2 and hi[0]>1.8 and lo[1]<-.3 and hi[1]>-.6 and hi[2]>1.15:
                print('OBSTACLE',prim.GetPath(),lo,hi,prim.GetAttribute('physics:approximation').Get(),flush=True)
    k=kin.fk(q)
    bi=robot.body_names.index('base_link')
    p=arr(robot.data.body_pos_w)[0,bi]
    quat=arr(robot.data.body_quat_w)[0,bi]
    r=Rotation.from_quat(quat).as_matrix()
    print('PROBE',json.dumps({'q':q.tolist(),'body_names':robot.body_names,'fk':k.tolist(),'actual_tcp':(p+r@np.array([.131,0,0])).tolist(),'actual_rot':r.tolist(),'quat':quat.tolist(),'limits':kin.limits.tolist()}),flush=True)
    # Approach down, fingers close along world X.
    target=np.eye(4);target[:3,:3]=np.array([[0,1,0],[0,0,-1],[-1,0,0]])
    for xyz in [[2,-.4,1.04],[2,-.4,1.20],[2.49,-.31,1.15]]:
        target[:3,3]=xyz
        try: print('IK',xyz,kin.ik(target,q).tolist(),flush=True)
        except RuntimeError as e: print(str(e),flush=True)
    env.close()
