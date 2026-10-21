"""Execute staged, physics-based grasp/place; save synchronized robot observations."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
parser=get_isaaclab_arena_cli_parser()
parser.add_argument('--variant',choices=['base','human','property','robot'],default='base')
parser.add_argument('--output',default='datasets/kitchen_auto/probe')
parser.add_argument('--video',action='store_true')
args=parser.parse_args()
args.enable_cameras=args.video
with SimulationAppContext(args):
    import numpy as np
    import torch
    import h5py
    from scipy.spatial.transform import Rotation
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    environment=KitchenSafetyEnvironment().build(KitchenSafetyCfg(variant=args.variant,enable_cameras=args.video))
    builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device=args.device,presets='physx',num_envs=1,solve_relations=False,disable_fabric=True))
    cfg,kwargs=builder.compose_manager_cfg()
    cfg.sim.use_newton_actuators=False
    from isaaclab.sensors import ContactSensorCfg
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kwargs)
    env.reset()
    import omni.physx
    from pxr import PhysicsSchemaTools
    contacts=set()
    def contact_report(headers,data):
        for h in headers:
            a=str(PhysicsSchemaTools.intToSdfPath(h.actor0));b=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if 'Robot' in a or 'Robot' in b:
                key=(a,b)
                if key not in contacts: print('CONTACT',a,b,flush=True);contacts.add(key)
    subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_report)
    e=env.unwrapped; scene=e.scene; robot=scene['robot']
    ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    def arr(v): return (v.torch if hasattr(v,'torch') else v).detach().cpu().numpy().copy()
    layout=json.loads(Path('/workspace/configs/layout.json').read_text())
    base=np.array(layout['robot_position']);base[2]+=layout['stand_height']
    kin=DroidKinematics(base,layout['robot_yaw_deg'])
    q=arr(robot.data.joint_pos)[0,ids]
    grip=0.; phase='settle'; records=[]; terminated=False; terminal={}
    out=Path('/workspace')/args.output;out.mkdir(parents=True,exist_ok=True)
    bi=robot.body_names.index('base_link')
    def snapshot():
        p=arr(robot.data.body_pos_w)[0,bi];quat=arr(robot.data.body_quat_w)[0,bi]
        tcp=p+Rotation.from_quat(quat).apply([.131,0,0])
        return dict(q=arr(robot.data.joint_pos)[0],dq=arr(robot.data.joint_vel)[0],tcp=tcp,
                    mustard=arr(scene['mustard_bottle'].data.root_state_w)[0],
                    bowl=arr(scene['bowl'].data.root_state_w)[0])
    reset_original=e._reset_idx
    def capture_reset(env_ids):
        if len(env_ids):
            terminal['state']=snapshot()
            terminal['terms']={n:bool(arr(e.termination_manager.get_term(n))[0]) for n in e.termination_manager.active_terms}
        return reset_original(env_ids)
    e._reset_idx=capture_reset
    def step(command,gripper):
        global terminated
        if terminated: return
        before=snapshot()
        if len(records)%30==0:
            print('DEBUG', 'forces',arr(scene['robot_contacts'].data.net_forces_w).round(1).tolist(), 'torques',arr(robot.data.applied_torque).round(2).tolist(),flush=True)
        a=np.r_[command,gripper].astype(np.float32)
        obs,reward,done,truncated,info=env.step(torch.tensor(a[None],device=e.device))
        terminated=bool(arr(done)[0] or arr(truncated)[0])
        after=terminal['state'] if terminated else snapshot()
        records.append((phase,a,before,after))
        if len(records)%30==0:
            print('STEP',len(records),phase,'tcp',after['tcp'].round(3),'bottle',after['mustard'][:3].round(3),'grip',after['q'][7:].round(3),flush=True)
    def move(target,seconds=2):
        global q
        start=q.copy()
        for u in np.linspace(0,1,max(2,round(seconds/e.step_dt))):
            s=u*u*(3-2*u)
            step(start+s*(target-start),grip)
            if terminated: break
        q=target.copy()
    def cart(xyz,seconds=2):
        target=np.eye(4)
        target[:3,:3]=Rotation.from_euler('x',35,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
        target[:3,3]=xyz
        move(kin.ik(target,q),seconds)
    result={'variant':args.variant,'layout':layout,'success':False}
    try:
        for _ in range(30): step(q,grip)
        p=snapshot()['mustard'][:3].copy();b=snapshot()['bowl'][:3].copy()
        print('FK_ERROR',np.linalg.norm(kin.fk(arr(robot.data.joint_pos)[0,ids])[:3,3]-snapshot()['tcp']),flush=True)
        phase='pregrasp';cart([p[0],p[1],1.16],3)
        phase='approach';cart([p[0],p[1],1.035],2)
        phase='close';grip=1
        for _ in range(30):step(q,grip)
        phase='lift';cart([p[0],p[1],1.18],3)
        result['lift_height']=float(snapshot()['mustard'][2]-p[2])
        phase='transfer';cart([b[0],b[1],1.18],4)
        phase='lower';cart([b[0],b[1],1.085],3)
        phase='release';grip=0
        for _ in range(30):step(q,grip)
        phase='retreat';cart([b[0],b[1],1.18],2)
        phase='settle'
        for _ in range(60):step(q,grip)
        result['success']=terminal.get('terms',{}).get('success',False)
        result['terminations']=terminal.get('terms',{})
    except Exception as ex:
        result['error']=repr(ex)
        print('COLLECTION_ERROR',repr(ex),flush=True)
    finally:
        with h5py.File(out/'trajectory.h5','w') as f:
            f.attrs['dt']=e.step_dt;f.attrs['quaternion_order']='xyzw'
            f.create_dataset('phase',data=np.array([x[0] for x in records],dtype=h5py.string_dtype()))
            f.create_dataset('action',data=np.array([x[1] for x in records]))
            for key in records[0][2]:
                f.create_dataset('obs/'+key,data=np.array([x[2][key] for x in records]))
                f.create_dataset('next_obs/'+key,data=np.array([x[3][key] for x in records]))
        result['steps']=len(records)
        result['final_state']={k:v.tolist() for k,v in (terminal.get('state') or snapshot()).items()}
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print('COLLECTION_RESULT',json.dumps(result),flush=True)
        env.close()
