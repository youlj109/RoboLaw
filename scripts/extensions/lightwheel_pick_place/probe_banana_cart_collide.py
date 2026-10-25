"""Coarse-to-fine collision diagnosis on banana_2 local cart hover line.
Sparse fraction checkpoints first; binary-search first-violation interval.
"""
import json
from pathlib import Path
import h5py
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.planning import DroidKinematics, transform

ep_a2r = Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_20261024T135925Z')
ep_r2 = Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_20261024T122330Z')
r = json.loads((ep_a2r/'result.json').read_text())
base = np.array([1.8419278860092163,-2.216993808746338,0.8100370168685913,0,0,0,1], float)
poses = {n: np.asarray(r['initial_object_states'][n][:7], float)
         for n in ['banana_1','banana_2','wooden_bowl','plate','apple','orange']}
with h5py.File(ep_r2/'trajectory.h5') as f:
    q0 = np.asarray(f['obs/q'][0,:7])
scene = ep_a2r/'scene.usda'
if not scene.exists() or scene.stat().st_size < 10000:
    scene = ep_r2/'scene.usda'
PICK='banana_2'

# configs to diagnose: current tip + best residual near-misses
CFGS = [
    {'tilt':35.0,'yaw':90.0,'gho':0.02,'st':0.16,'tag':'A2_current'},
    {'tilt':35.0,'yaw':90.0,'gho':0.02,'st':0.18,'tag':'A1_gho02'},
    {'tilt':35.0,'yaw':90.0,'gho':0.01,'st':0.18,'tag':'near_miss_00416'},
    {'tilt':0.0,'yaw':90.0,'gho':0.02,'st':0.18,'tag':'best_fallback_00407'},
]

SPARSE = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
BIN_ITERS = 8

kin=DroidKinematics(base[:3],0.0)
stage=Usd.Stage.Open(str(scene))
rigid=('banana_1','banana_2','wooden_bowl','plate','apple','orange')
world=CollisionWorld(stage,kin,pick_object=PICK,destination_object='wooden_bowl',
    source_surface_suffix='/plate',
    movable_objects=tuple(n for n in rigid if n!=PICK)+('glass',))
for n,p in poses.items():
    try: world.set_obstacle_pose(n,p)
    except Exception: pass
world.bind_pick(PICK)
start=world.clearance(q0,(PICK,))
print('START_FCL', start.get('valid'), 'slack', start.get('slack_m'),
      'part', start.get('robot_part'), 'obs', start.get('obstacle'), flush=True)

planner=LimePlanner(world,kin,dt=1/30,seed=43,max_joint_speed=.4)
R0=np.array([[0,-1,0],[0,0,1],[-1,0,0]],float)

def make_hover(tilt,yaw,gho,st):
    center=(transform(poses[PICK][:3],poses[PICK][3:7])@np.r_[world.object_center,1])[:3]+np.array([0.,0.,float(gho)])
    R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@R0
    approach=R[:,2]/np.linalg.norm(R[:,2])
    hover=center-float(st)*approach
    return center, hover, R, approach

def eval_frac(q_start, tcp0, hover, frac, ignore=(PICK,)):
    """Hold start orientation (matches plan_cartesian_line); interpolate position."""
    tcp=tcp0.copy()
    tcp[:3,3]=tcp0[:3,3]+frac*(hover-tcp0[:3,3])
    info={'frac':float(frac),'tcp':tcp[:3,3].tolist()}
    try:
        q=kin.ik(tcp,q_start,attempts=12)
        info['ik_ok']=True
        info['q']=q.tolist()
        # joint-branch jump check vs start of segment seed
        info['dq_max']=float(np.max(np.abs(q-q_start)))
        info['branch_jump']=bool(info['dq_max']>0.15)
    except Exception as e:
        info['ik_ok']=False
        info['ik_err']=str(e)[:160]
        info['collision']=False
        info['curobo_ok']=None
        return info, None
    c=world.clearance(q,ignore)
    info['fcl_valid']=bool(c.get('valid'))
    info['fcl_slack']=c.get('slack_m')
    info['fcl_part']=c.get('robot_part')
    info['fcl_obs']=c.get('obstacle')
    info['fcl_intersecting']=bool(c.get('intersecting'))
    # cuRobo sphere validity at waypoint
    try:
        checker=planner.motion_gen.world_coll_checker
        # enable all non-ignored
        names=list(planner.obstacle_names)
        for n in names:
            checker.enable_obstacle(n, enable=(n not in ignore and not n.startswith(PICK)))
        ok=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(q[None])).all())
        info['curobo_ok']=ok
    except Exception as e:
        info['curobo_ok']=None
        info['curobo_err']=str(e)[:120]
    bad=(not info['ik_ok']) or info.get('branch_jump') or (not info.get('fcl_valid',True)) or (info.get('curobo_ok') is False)
    info['bad']=bool(bad)
    return info, q

def diagnose(cfg):
    center,hover,R,approach=make_hover(cfg['tilt'],cfg['yaw'],cfg['gho'],cfg['st'])
    tcp0=kin.fk(q0)
    row={'tag':cfg['tag'],'tilt':cfg['tilt'],'yaw':cfg['yaw'],'gho':cfg['gho'],'st':cfg['st'],
         'center':center.tolist(),'hover':hover.tolist(),'approach':approach.tolist(),
         'sparse':[],'collision_found':False,'narrowed':None}
    print(f"\n=== {cfg['tag']} t{cfg['tilt']} y{cfg['yaw']} gho{cfg['gho']} st{cfg['st']} hover={hover}", flush=True)
    # endpoint residual probe
    Th=np.eye(4); Th[:3,:3]=tcp0[:3,:3]; Th[:3,3]=hover
    try:
        # reuse kin.ik residual path
        q_end=kin.ik(Th,q0,attempts=12)
        row['endpoint_ik_ok']=True
        # compute residual manually
        t=kin.fk(q_end)
        from scipy.spatial.transform import Rotation as RR
        res=np.linalg.norm(np.r_[t[:3,3]-Th[:3,3], .25*RR.from_matrix(Th[:3,:3]@t[:3,:3].T).as_rotvec()])
        row['endpoint_residual']=float(res)
    except Exception as e:
        row['endpoint_ik_ok']=False
        msg=str(e)
        row['endpoint_ik_err']=msg[:160]
        import re
        m=re.search(r'residual ([0-9.]+)', msg)
        if m: row['endpoint_residual']=float(m.group(1))

    seed=q0
    sparse_qs=[]
    first_bad=None
    last_good=0.0
    for frac in SPARSE:
        info,q=eval_frac(seed if sparse_qs else q0, tcp0, hover, frac)
        # warm-start along line when ok
        if q is not None and info.get('ik_ok') and not info.get('branch_jump'):
            seed=q
            sparse_qs.append(q)
        row['sparse'].append(info)
        flag='BAD' if info.get('bad') else 'ok'
        print(f"  sparse {frac:.2f} {flag} ik={info.get('ik_ok')} fcl={info.get('fcl_valid')} "
              f"cu={info.get('curobo_ok')} jump={info.get('branch_jump')} "
              f"slack={info.get('fcl_slack')} obs={info.get('fcl_obs')} part={info.get('fcl_part')} "
              f"ik_err={info.get('ik_err') or info.get('endpoint_ik_err','')}", flush=True)
        if info.get('bad') and first_bad is None:
            first_bad=frac
        if not info.get('bad'):
            last_good=frac

    if first_bad is None:
        row['collision_found']=False
        row['narrowed']=None
        print('  NO_COLLISION_SPARSE residual=', row.get('endpoint_residual'), flush=True)
        return row

    row['collision_found']=True
    # binary search between last_good and first_bad
    lo,hi=last_good,first_bad
    # find last_good before first_bad from sparse list
    lo=0.0
    for s in row['sparse']:
        if s['frac']<first_bad and not s.get('bad'):
            lo=s['frac']
    hi=first_bad
    best_bad=None
    seed=q0
    # re-seed to lo
    info_lo,q_lo=eval_frac(q0,tcp0,hover,lo)
    if q_lo is not None: seed=q_lo
    for it in range(BIN_ITERS):
        mid=0.5*(lo+hi)
        info,q=eval_frac(seed,tcp0,hover,mid)
        print(f"  bin{it} mid={mid:.5f} bad={info.get('bad')} fcl={info.get('fcl_valid')} "
              f"cu={info.get('curobo_ok')} jump={info.get('branch_jump')} "
              f"obs={info.get('fcl_obs')} slack={info.get('fcl_slack')}", flush=True)
        if info.get('bad'):
            hi=mid
            best_bad=info
        else:
            lo=mid
            if q is not None: seed=q
    row['narrowed']={
        'lo_ok':float(lo),'hi_bad':float(hi),'width':float(hi-lo),
        'first_sparse_bad':float(first_bad),
        'at_bad':best_bad,
    }
    print(f"  NARROWED lo_ok={lo:.5f} hi_bad={hi:.5f} width={hi-lo:.5f}", flush=True)
    return row

reports=[]
for cfg in CFGS:
    reports.append(diagnose(cfg))

# recommendation
def score(r):
    # prefer configs with no collision and lowest residual
    res=r.get('endpoint_residual')
    if res is None: res=9.0
    return (1 if r.get('collision_found') else 0, res)

best=sorted(reports, key=score)[0]
rec={
    'mode':'coarse_to_fine_cart_collide',
    'start_fcl':{k:start.get(k) for k in ['valid','slack_m','robot_part','obstacle','intersecting']},
    'sparse_fractions':SPARSE,
    'bin_iters':BIN_ITERS,
    'reports':reports,
    'best_cfg':{k:best.get(k) for k in ['tag','tilt','yaw','gho','st','endpoint_residual','collision_found','narrowed']},
}
out=Path('/root/autodl-tmp/lgd-b03-logs/cart_collide_diag.json')
out.write_text(json.dumps(rec,indent=2,default=str))
print('WROTE', out, flush=True)
print('BEST', rec['best_cfg'], flush=True)
print('COMPLETE', flush=True)
