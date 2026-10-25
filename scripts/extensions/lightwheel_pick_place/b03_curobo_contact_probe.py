"""Find the rejecting cuRobo obstacle for the offline contact line; no simulation."""
import json
from pathlib import Path
import h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task,BANANA_TASK
p=Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_review_20261025T122401Z');r=json.load(open(p/'result.json'));spec=resolve_task(BANANA_TASK);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:]).as_euler('xyz',degrees=True)[2]);world=CollisionWorld(Usd.Stage.Open(str(p/'scene.usda')),kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!=spec.pick_object));planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner._prepare_path_query((spec.pick_object,),0.,'lift');positions=np.load('/root/autodl-tmp/B03-review/contact_line_probe.npz')['positions'];q=planner.args.to_device(positions);checker=planner.motion_gen.world_coll_checker;names=[n for n in planner.obstacle_names if not planner._ignored(n,(spec.pick_object,))]
def valid(group):
 selected=set(group)
 for name in planner.obstacle_names:checker.enable_obstacle(name,enable=name in selected)
 return planner.motion_gen.ik_solver.check_valid(q).detach().cpu().numpy().astype(bool)
full=valid(names);empty=valid([]);bad=[]
def bisect(group):
 if not group or valid(group).all():return
 if len(group)==1:bad.extend(group);return
 half=len(group)//2;bisect(group[:half]);bisect(group[half:])
if empty.all():bisect(names)
report={'samples':len(positions),'all_valid':bool(full.all()),'invalid_indices':np.flatnonzero(~full).tolist(),'self_and_joint_only_valid':bool(empty.all()),'rejecting_obstacles':bad};Path('/root/autodl-tmp/B03-review/curobo_contact_probe_v4.json').write_text(json.dumps(report,indent=2));print('PROBE',json.dumps(report),flush=True)
with h5py.File(p/'trajectory.h5') as h:
    qreset=h['obs/q'][0,:7];qhover=h['next_obs/q'][-1,:7]
base_hover=kin.fk(qhover);base_contact=np.array(r['approach_contact_standoff_m']);rows=[]
for bias,height in [(-.022,0.)]:
    row={'grasp_lateral_x_m':bias,'height_add_m':height}
    hover=base_hover.copy();hover[:3,3]+=np.array([bias,0,height]);contact=base_hover.copy();contact[:3,3]=base_contact+np.array([bias,0,height])
    try:
        pre=planner.plan_pose(qreset,hover,(spec.pick_object,),0.,phase='transfer');qend=pre['positions'][-1]
        line=planner.plan_cartesian_line(qend,contact,(spec.pick_object,),0.,phase='lift')
        checked=planner.validate_path(line['positions'],(spec.pick_object,),0.,phase='transfer')
        row.update(passed=bool(checked['valid']),pre_samples=len(pre['positions']),contact_samples=len(line['positions']),validation=checked)
    except Exception as exc:row.update(passed=False,error=str(exc))
    rows.append(row);print('CANDIDATE',json.dumps(row),flush=True)
    if row['passed']:break
Path('/root/autodl-tmp/B03-review/offline_grasp_candidates_v4.json').write_text(json.dumps({'scope':'Offline two-segment planning only, no physical grasp claim','rows':rows},indent=2))
