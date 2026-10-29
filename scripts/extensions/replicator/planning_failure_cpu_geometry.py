"""Reusable CPU-only robot sphere/source geometry diagnostic helpers.

Vendored from the source-bound round2 diagnostic without changing its geometry,
5 mm robot sphere buffer, contact masks or IK implementation. No Kit, cuRobo
solve or physics entry point is present. Probe is an endpoint diagnostic; it
cannot establish continuous path feasibility, real contacts or safety.

Legacy source: reports/replicator_physical_retry_round2_20261028/
    diagnose_planning_failures_cpu.py
Legacy SHA256: de6c1b311ab82208184840f215944d59bdfb14447e15842b6a18221f381fce5f
"""
from pathlib import Path
import hashlib
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import fcl
from scipy.spatial.transform import Rotation
from pxr import Usd

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.quick_trajectory import PlanningWorld
from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
from kitchen_safety.planning import DroidKinematics

ASSET=ROOT/'assets/curobo/droid'
CFG=yaml.safe_load((ASSET/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics']
JOINTS=ET.parse(ASSET/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint')
CALIBRATION=np.asarray(json.loads((ASSET/'tcp_calibration.json').read_text())['T_urdfEE_USDtcp'])
PHASES=('pregrasp','approach','lift','transfer','lower','retreat')

def sha(path):
 with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def local(path):
 result=(ROOT/str(path).removeprefix('/workspace/')).resolve()
 if not result.is_relative_to(ROOT):raise ValueError('Outside workspace')
 return result

def bound(ref):
 path=local(ref['path'])
 if sha(path)!=ref['sha256']:raise ValueError('Bound input changed: '+str(path))
 return path

def binding(path):return dict(path=str(path.relative_to(ROOT)),sha256=sha(path))

def robot_spheres(kin,q,grip):
 angles={f'panda_joint{i+1}':float(value) for i,value in enumerate(q)}
 angles.update(dict(zip(('finger_joint','right_outer_knuckle_joint','left_inner_finger_joint',
  'right_inner_finger_joint','left_inner_finger_knuckle_joint','right_inner_finger_knuckle_joint'),
  np.array([1,1,-1,1,-1,-1])*grip)))
 frames={'panda':kin.base};pending=JOINTS.copy()
 while pending:
  count=len(pending)
  for joint in pending.copy():
   parent=joint.find('parent').get('link');child=joint.find('child').get('link')
   if parent not in frames:continue
   origin=joint.find('origin');transform=np.eye(4)
   if origin is not None:
    transform[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
    transform[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
   if joint.get('type') in ('revolute','continuous'):
    transform[:3,:3]=transform[:3,:3]@Rotation.from_rotvec(np.fromstring(joint.find('axis').get('xyz'),sep=' ')*angles.get(joint.get('name'),0)).as_matrix()
   frames[child]=frames[parent]@transform;pending.remove(joint)
  if len(pending)==count:raise ValueError('URDF tree unresolved')
 if not np.allclose(frames['base_link']@CALIBRATION,kin.fk(q),atol=2e-6,rtol=0):raise ValueError('URDF/USD TCP calibration mismatch')
 return [(name,i,frames[name][:3,:3]@sphere['center']+frames[name][:3,3],float(sphere['radius'])+.005)
  for name in CFG['collision_link_names'] if name in frames
  for i,sphere in enumerate(CFG['collision_spheres'].get(name,[]))]

class Probe:
 def __init__(self,entry,result):
  self.result=result;self.entry=entry
  self.layout=json.loads(bound(dict(path=entry['config_path'],sha256=entry['config_sha256'])).read_text())
  stage_path=bound(dict(path=entry['source_scene_path'],sha256=entry['source_scene_sha256']))
  self.stage=Usd.Stage.Open(str(stage_path));self.world=PlanningWorld(self.stage)
  self.kin=DroidKinematics(np.asarray(self.layout['robot_position'])+[0,0,self.layout['stand_height']],self.layout['robot_yaw_deg'])
  self.payload=result['payload_name'];self.geometry=PayloadGeometry(local(self.layout['objects'][self.payload]['usd_path']))
  self.cover=self.geometry.sphere_cover(100)[:2];self.cache={}
  self.low=np.asarray([x[2] for x in self.world.objects]);self.high=np.asarray([x[3] for x in self.world.objects])
  self.attachment=np.asarray(result['nominal_attachment'])
  self.phase=result.get('failed_phase') or result['stage']
  self.masks=next(p['contact_masks'] for p in result['phases'] if p['phase']==self.phase)
  self.carrying=self.phase in ('lift','transfer','lower')
  self.grip=float(result.get('planning_gripper_joint_rad',result['nominal_gripper_joint_rad'])) if self.carrying else 0.
 def obstacle(self,i):
  name=self.world.objects[i][0]
  if name not in self.cache:
   if name in self.world.meshes:
    vertices,faces=self.world.meshes[name];model=fcl.BVHModel();model.beginModel(len(vertices),len(faces));model.addSubModel(vertices,faces);model.endModel();obj=fcl.CollisionObject(model)
   else:obj=fcl.CollisionObject(fcl.Box(*np.maximum(self.high[i]-self.low[i],1e-9)),fcl.Transform((self.high[i]+self.low[i])/2))
   self.cache[name]=obj
  return self.cache[name]
 def check(self,q):
  spheres=robot_spheres(self.kin,q,self.grip)
  if self.carrying:
   transform=self.kin.fk(q)@self.attachment;centers,radii=self.cover
   spheres += [('attached_object',i,transform[:3,:3]@center+transform[:3,3],float(radius)) for i,(center,radius) in enumerate(zip(centers,radii))]
  hits=[]
  for link,index,center,radius in spheres:
   near=np.flatnonzero(np.linalg.norm(np.maximum(np.maximum(self.low-center,center-self.high),0),axis=1)<radius+.001)
   obj=fcl.CollisionObject(fcl.Sphere(radius),fcl.Transform(center))
   for i in near:
    name=self.world.objects[i][0]
    if any(self.world._matches(name,mask) for mask in self.masks):continue
    other=self.obstacle(i);distance=float(fcl.distance(obj,other,fcl.DistanceRequest(),fcl.DistanceResult()))
    intersection=bool(fcl.collide(obj,other,fcl.CollisionRequest(),fcl.CollisionResult()))
    if intersection or distance<.001:
     hits.append(dict(link=link,sphere_index=index,world_object=name,sphere_xyzr=[*center.tolist(),radius],
      intersecting=intersection,distance_query_m=distance,within_1mm=True,
      world_representation='source_triangle_mesh' if name in self.world.meshes else 'source_primitive_AABB'))
  self_hits=[];ignored=CFG['self_collision_ignore'];buffers=CFG.get('self_collision_buffer',{})
  for i,(la,ia,ca,ra) in enumerate(spheres):
   for lb,ib,cb,rb in spheres[i+1:]:
    if la==lb or lb in ignored.get(la,[]) or la in ignored.get(lb,[]):continue
    gap=float(np.linalg.norm(ca-cb)-ra-rb-float(buffers.get(la,0))-float(buffers.get(lb,0)))
    if gap<0:self_hits.append(dict(link_a=la,sphere_a=ia,link_b=lb,sphere_b=ib,signed_clearance_m=gap))
  unique=sorted({(r['link'],r['world_object']) for r in hits})
  return dict(world_pairs=hits,unique_world_pairs=[dict(link=a,world_object=b) for a,b in unique],
              world_intersection_pairs=sum(r['intersecting'] for r in hits),self_sphere_overlaps=self_hits,
              robot_world_sphere_buffer_m=.005,planning_gripper_joint_rad=self.grip,masked_objects=self.masks,
              URDF_TCP_matches_USD_FK=True,payload_planning_attachment_enabled=self.carrying)

def summarize(entry):
 start=entry['failed_phase_start'];goal=entry['fixed_goal_one_seed_ik']
 start_hits=start['collision_diagnostic']['unique_world_pairs']
 goal_hits=goal.get('collision_diagnostic',{}).get('unique_world_pairs',[])
 facts=[f"原 cuRobo 在 {entry['failed_phase']} 返回 {entry['solver_status']}。",f"CPU 检查使用原失败阶段同一接触mask，规划夹爪角 {entry['planning_gripper_joint_rad']:.3f} rad；实际闭合命令 {entry['commanded_gripper_joint_rad']:.3f} rad，二者分开记录。"]
 def names(rows):return '；'.join(r['link']+' ↔ '+r['world_object'] for r in rows[:5])
 if start_hits:facts.append('失败阶段实际规划起点的保守机器人球/源碰撞几何存在接触或1mm内近邻：'+names(start_hits))
 else:facts.append('该失败阶段规划起点未检出球/源场景世界碰撞或1mm内近邻。')
 if goal['solved']:
  facts.append(f"固定原目标、仅一次CPU IK获得候选，位置残差 {goal['position_error_m']:.6f}m，姿态残差 {goal['orientation_error_rad']:.6f}rad。")
  if goal_hits:facts.append('该单一IK分支目标存在保守球/源场景接触或1mm内近邻：'+names(goal_hits))
  else:facts.append('该单一IK分支目标未检出球/源场景世界碰撞或1mm内近邻；未检查连续路径。')
 else:facts.append('同一固定目标的一次CPU IK未达到现有容差；未搜索其他种子或目标。')
 if start_hits and 'INVALID_START' in entry['solver_status']:
  interpretation='CPU静态检查与规划器的起点碰撞报错相容；受限的是该候选上一阶段末端/切换夹爪状态，不能据此认定布局整体无解。'
  recommendation='下一次授权时优先调整上一阶段末端及张爪撤离方向，使下一阶段起点在原碰撞约束下可行；本轮保留失败，不移动场景物体。'
 elif goal_hits:
  interpretation='该固定目标的一个CPU IK分支受所列几何约束影响，可能解释本候选的规划失败；其他关节分支及连续路径未穷尽，未确认全局根因。'
  recommendation='下一次授权时针对列出的夹爪/手腕/携物包络与障碍，微调抓取方位、腕部姿态或原目标区域内的落点，并先复核该阶段端点；保持布局和碰撞规则。'
 elif not goal['solved']:
  interpretation='一次局部CPU IK也未解出同一目标，提示该种子/姿态组合较困难；不能把一次IK失败等同于任务无解。'
  recommendation='下一次授权时先在同一目标检查一个不同关节分支或小幅腕部姿态调整的可达性，再决定路径；不改布局。'
 else:
  interpretation='本次CPU端点检查未给出世界碰撞解释；仍可能涉及自碰撞、其他IK分支、方向约束或连续轨迹优化。未确认根因。'
  recommendation='下一次授权时优先检查记录的自碰撞球对和方向约束，并对原目标的另一关节分支/中间过渡状态进行规划；本轮不追加求解。'
 return facts,interpretation,recommendation
