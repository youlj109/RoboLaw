"""CPU check of the original YAML's On/NextTo/anchor/yaw relations.

Calls upstream validators and bounding-box helpers unchanged, without Kit. Scene
assets are lightweight holders for those validators; geometry is the real USD.
Episode mode reads the saved authored layout, never the current mutable config.
"""
import argparse,hashlib,json,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np,yaml,h5py
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
from isaaclab_arena.relations.validation.pre_physics import NextToValidator,OnRelationValidator
from isaaclab_arena.relations.relations import NextTo,On,RotateAroundSolution
from isaaclab_arena.relations.bounding_box_helpers import rotate_candidate_bboxes
from isaaclab_arena.relations.relation_loss_strategies import SIDE_CONFIGS,next_to_violations
from isaaclab_arena.utils.bounding_box import AxisAlignedBoundingBox
from isaaclab_arena.utils.usd.helpers import compute_local_bounding_box_from_usd_at_joint_pos,compute_local_bounding_box_from_usd
ROOT=Path(__file__).resolve().parents[3]
YAML=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_replicator_g_shape_mustard_bowl.yaml'
ASSEMBLY=ROOT/'runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd'
class Asset:
 def __init__(self,name):self.name=name;self.relations=[]
 def get_relations(self):return self.relations

def check_layout(layout):
 source=yaml.safe_load(YAML.read_text());assets={name:Asset(name) for name in ('droid','floor','island_top','mustard_bottle','bowl')};robot=assets['droid'];kitchen=Usd.Stage.Open(str(ROOT/layout['kitchen_usd']));cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_]);bounds={}
 for name,path in [('floor','/World/KitchenRoom/Floor'),('island_top','/World/CapSurface_03')]:
  box=cache.ComputeWorldBound(kitchen.GetPrimAtPath(path)).ComputeAlignedRange();bounds[assets[name]]=AxisAlignedBoundingBox(tuple(box.GetMin()),tuple(box.GetMax()))
 bounds[robot]=compute_local_bounding_box_from_usd_at_joint_pos(str(ASSEMBLY),{},prim_path='/panda/panda_link0/stand_instanceable')
 for name,path in [('mustard_bottle','assets/robolab/objects/hope/mustard_bottle.usd'),('bowl','assets/robolab/objects/ycb/bowl.usd')]:bounds[assets[name]]=compute_local_bounding_box_from_usd(str(ROOT/path))
 for spec in source['relations']:
  cls={'on':On,'next_to':NextTo,'rotate_around_solution':RotateAroundSolution}.get(spec['kind'])
  if cls: assets[spec['subject']].relations.append(cls(assets[spec['reference']],**spec['params']) if 'reference' in spec else cls(**spec['params']))
 positions={assets['floor']:(0.,0.,0.),assets['island_top']:(0.,0.,0.),robot:tuple(np.array(layout['robot_position'])+[0,0,layout['stand_height']]),assets['mustard_bottle']:tuple(layout['mustard_position']),assets['bowl']:tuple(layout['bowl_position'])}
 yaw={robot:math.radians(layout['robot_yaw_deg']),assets['mustard_bottle']:math.radians(layout.get('mustard_yaw_deg',0)),assets['bowl']:math.radians(layout.get('bowl_yaw_deg',0))};bounds=rotate_candidate_bboxes(list(assets.values()),bounds,[yaw]);params=SimpleNamespace(verbose=False,on_relation_z_tolerance_m=.005);rows=[]
 for spec in source['relations']:
  subject=assets[spec['subject']];kind=spec['kind'];row=dict(kind=kind,subject=subject.name,source_params=spec['params'])
  if kind in ('on','next_to'):
   parent=assets[spec['reference']];pos={subject:positions[subject],parent:positions[parent]};row['reference']=parent.name
   validator=OnRelationValidator(params) if kind=='on' else NextToValidator(params);row['passed']=bool(validator._validate(pos,bounds));row['validator']=type(validator).__name__
   if kind=='next_to':
    rel=next(r for r in subject.relations if isinstance(r,NextTo));half,distance=next_to_violations(SIDE_CONFIGS[rel.side],bounds[subject].min_point.new_tensor([positions[subject]]),bounds[subject],bounds[parent].translated(positions[parent]),rel.distance_m);row.update(side_violation_m=float(half),distance_violation_m=float(distance),facing_edge_gap_m=float(positions[subject][1]+bounds[subject].min_point[0,1]-positions[parent][1]-bounds[parent].max_point[0,1]))
   else:
    rel=next(r for r in subject.relations if isinstance(r,On));child=bounds[subject].translated(positions[subject]);support=bounds[parent].translated(positions[parent]);margins=np.r_[child.min_point.numpy()[0,:2]-support.min_point.numpy()[0,:2],support.max_point.numpy()[0,:2]-child.max_point.numpy()[0,:2]];row.update(edge_margin_required_m=rel.edge_margin_m,measured_edge_margins_xy_m=margins.tolist(),support_height_gap_m=float(child.min_point[0,2]-support.max_point[0,2]))
  elif kind=='rotate_around_solution':row.update(passed=bool(abs(yaw[subject]-spec['params']['yaw_rad'])<1e-8),actual_yaw_rad=yaw[subject])
  elif kind=='is_anchor':row.update(passed=True,detail='Source kitchen USD and anchor prim transforms unchanged; world origin identity')
  else:raise ValueError(f'Unvalidated source relation: {spec}')
  rows.append(row)
 return dict(passed=all(x['passed'] for x in rows),scope='Original YAML relations at authored initial placement; unmodified upstream OnRelationValidator and NextToValidator, original posed bbox and rotation helpers. Dynamic task success and all-world trajectory collision require independent checks.',source_yaml=str(YAML.relative_to(ROOT)),source_yaml_sha256=hashlib.sha256(YAML.read_bytes()).hexdigest(),stand_usd=str(ASSEMBLY.relative_to(ROOT)),positions={a.name:list(p) for a,p in positions.items()},bounds={a.name:dict(min=b.min_point.tolist(),max=b.max_point.tolist()) for a,b in bounds.items()},checks=rows)

def main():
 p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument('--episode');g.add_argument('--layout');p.add_argument('--report');a=p.parse_args()
 if a.episode:
  directory=Path(a.episode);result=json.loads((directory/'result.json').read_text());layout=result['layout'];target=Path(a.report) if a.report else directory/'source_relation_audit.json'
 else:layout=json.loads(Path(a.layout).read_text());target=Path(a.report) if a.report else ROOT/'reports/skill_trials/replicator/proposed_source_relations.json'
 report=check_layout(layout)
 if a.episode and (directory/'trajectory.h5').exists():
  with h5py.File(directory/'trajectory.h5') as f:
   names=json.loads(f.attrs['robot_body_names']);pose=f['obs/robot_body_pose_w'][0,names.index('panda_link0')]
  expected=np.array(layout['robot_position'])+[0,0,layout['stand_height']];translation_error=float(np.linalg.norm(pose[:3]-expected));actual=Rotation.from_quat(pose[3:]);desired=Rotation.from_euler('z',layout['robot_yaw_deg'],degrees=True);angle_error=float((desired.inv()*actual).magnitude())
  recorded=dict(passed=translation_error<1e-5 and angle_error<1e-5,position_error_m=translation_error,orientation_error_rad=angle_error,recorded_xyz_xyzw=pose.tolist(),scope='Recorded fixed panda_link0 must agree with the reset base pose within float32 numerical tolerance; original relations above are pre-physics authored-placement checks.')
  report['recorded_base_pose_matches_layout']=recorded;report['passed']=report['passed'] and recorded['passed']
 target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(passed=report['passed'],checks=report['checks']),indent=2));return int(not report['passed'])
if __name__=='__main__':raise SystemExit(main())
