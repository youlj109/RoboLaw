"""CPU re-evaluation of source NextTo using unmodified upstream geometry/validator code.

The lightweight assets expose only get_relations/name; no Kit scene is constructed.
Inputs are recorded base pose, actual composed USD, source YAML relation, and the
actual island USD. Does not change any scene, source config, or existing trajectory.
"""
import hashlib,json,math
from pathlib import Path
from types import SimpleNamespace
import h5py,numpy as np,yaml
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
from isaaclab_arena.relations.validation.pre_physics import NextToValidator
from isaaclab_arena.relations.relations import NextTo,RotateAroundSolution
from isaaclab_arena.relations.bounding_box_helpers import rotate_candidate_bboxes
from isaaclab_arena.relations.relation_loss_strategies import SIDE_CONFIGS,next_to_violations
from isaaclab_arena.utils.bounding_box import AxisAlignedBoundingBox
from isaaclab_arena.utils.usd.helpers import compute_local_bounding_box_from_usd_at_joint_pos
ROOT=Path(__file__).resolve().parents[3]
YAML=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_replicator_g_shape_mustard_bowl.yaml'
ASSEMBLY=ROOT/'runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd'
class Asset:
 def __init__(self,name):self.name=name;self.relations=[]
 def get_relations(self):return self.relations
source=yaml.safe_load(YAML.read_text());next_params=next(r['params'] for r in source['relations'] if r['kind']=='next_to');yaw_params=next(r['params'] for r in source['relations'] if r['kind']=='rotate_around_solution')
robot=Asset('droid');island=Asset('island_top');robot.relations=[NextTo(island,**next_params),RotateAroundSolution(**yaw_params)]
validator=NextToValidator(SimpleNamespace(verbose=False))
stand_bounds=compute_local_bounding_box_from_usd_at_joint_pos(str(ASSEMBLY),{},prim_path='/panda/panda_link0/stand_instanceable')
layout=json.loads((ROOT/'configs/extensions/replicator/g_shape_mustard.json').read_text());kitchen=Usd.Stage.Open(str(ROOT/layout['kitchen_usd']));range_=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_]).ComputeWorldBound(kitchen.GetPrimAtPath('/World/CapSurface_03')).ComputeAlignedRange();island_bounds=AxisAlignedBoundingBox(tuple(range_.GetMin()),tuple(range_.GetMax()))
base_bboxes={robot:stand_bounds,island:island_bounds}
def bbox_dict(b):return dict(min=b.min_point.tolist(),max=b.max_point.tolist())
def check(position,bboxes):
 positions={robot:tuple(position),island:(0.,0.,0.)};passed=validator._validate(positions,bboxes)
 half,distance=next_to_violations(SIDE_CONFIGS[robot.relations[0].side],bboxes[robot].min_point.new_tensor([position]),bboxes[robot],bboxes[island],next_params['distance_m'])
 edge_gap=float(position[1]+bboxes[robot].min_point[0,1]-bboxes[island].max_point[0,1])
 return dict(passed=bool(passed),position_world=list(map(float,position)),side_violation_m=float(half),distance_violation_m=float(distance),facing_edge_gap_m=edge_gap)
records=[]
for variant,attempt in [('human','attempt_01'),('property','attempt_03'),('robot','attempt_02')]:
 with h5py.File(ROOT/'datasets/skill_trials/replicator'/variant/attempt/'trajectory.h5') as f:
  names=json.loads(f.attrs['robot_body_names']);pose=f['obs/robot_body_pose_w'][0,names.index('panda_link0')]
 yaw=float(Rotation.from_quat(pose[3:]).as_euler('xyz')[2]);bboxes=rotate_candidate_bboxes([robot,island],base_bboxes,[{robot:yaw}])
 # panda_link0's authored offset from /panda is only numerical roundoff (~24nm).
 # Undo it so the pose matches the root-relative upstream geometry helper.
 assembly=Usd.Stage.Open(str(ASSEMBLY));local=UsdGeom.Xformable(assembly.GetPrimAtPath('/panda/panda_link0')).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation();position=np.array(pose[:3],dtype=float)-Rotation.from_quat(pose[3:]).apply(np.array(local))
 current=check(position,bboxes);shift=max(0.,current['facing_edge_gap_m']-(next_params['distance_m']+next_params['tolerance_m']))
 minimum_candidate=position.copy();minimum_candidate[1]-=shift+1e-6
 margin_candidate=position.copy();margin_candidate[1]=.018
 nominal_candidate=position.copy();nominal_candidate[1]-=current['facing_edge_gap_m']-next_params['distance_m']
 records.append(dict(variant=variant,attempt=attempt,root_pose_derived_from_recorded_link0=True,world_yaw_rad=yaw,stand_bbox_rotated_root_relative=bbox_dict(bboxes[robot]),current=current,minimum_toward_island_translation_m=shift,minimum_plus_1um=check(minimum_candidate,bboxes),proposed_2mm_margin=check(margin_candidate,bboxes),nominal_gap=check(nominal_candidate,bboxes)))
report=dict(passed=all(r['current']['passed'] for r in records),implementation='Unmodified imported isaaclab_arena.relations.validation.pre_physics.NextToValidator._validate; original helper compute_local_bounding_box_from_usd_at_joint_pos and rotate_candidate_bboxes',source_yaml=str(YAML.relative_to(ROOT)),source_yaml_sha256=hashlib.sha256(YAML.read_bytes()).hexdigest(),source_next_to=next_params,source_rotate_around_solution=yaw_params,stand_bbox_unrotated=bbox_dict(stand_bounds),island_world_bbox=bbox_dict(island_bounds),episodes=records)
p=ROOT/'reports/skill_trials/replicator/source_relation_audit.json';p.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
