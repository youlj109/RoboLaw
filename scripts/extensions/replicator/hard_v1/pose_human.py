"""Blender CPU: bend the original MPFB adult and pose its connected right hand.

Run with Blender --background --python this_file -- --state gap|clear.
Exports a static complete skinned person; no animation or robot trajectory.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import bpy
from mathutils import Matrix, Vector

p=argparse.ArgumentParser()
p.add_argument('--state',choices=('gap','clear'),required=True)
p.add_argument('--bend-deg',type=float,default=0.)
p.add_argument('--wrist',type=float,nargs=3)
a=p.parse_args(sys.argv[sys.argv.index('--')+1:])
root=Path.cwd(); source=root/'assets/people/mpfb_corridor/posed.blend'
out=root/'assets/extensions/replicator/hard_v1'/('human_upper_'+a.state)
out.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=str(source))
rig=bpy.data.objects['KitchenAdult.rig']
def bones(): return {b.name:[list(rig.matrix_world@b.head),list(rig.matrix_world@b.tail)] for b in rig.pose.bones}
before=bones()
# Rotate upper-body bones about the original lumbar joint; pelvis/legs/feet stay put.
spine=rig.pose.bones['spine_01']; pivot=rig.matrix_world@spine.head
forward=Vector((-.819152,.573576,0)); axis=Vector((0,0,1)).cross(forward)
world_rotate=Matrix.Translation(pivot)@Matrix.Rotation(math.radians(a.bend_deg),4,axis)@Matrix.Translation(-pivot)
spine.matrix=rig.matrix_world.inverted()@world_rotate@rig.matrix_world@spine.matrix
bpy.context.view_layer.update()
upper=rig.pose.bones['upperarm_r']; lower=rig.pose.bones['lowerarm_r']; hand=rig.pose.bones['hand_r']
head=upper.head.copy(); elbow=lower.head.copy(); wrist=hand.head.copy()
l1=(elbow-head).length; l2=(wrist-elbow).length
wrist_world=Vector(a.wrist or ([-.53,.40,1.55] if a.state=='gap' else [-.40,.25,1.40]))
target=rig.matrix_world.inverted()@wrist_world; d=(target-head).length
if not abs(l1-l2)<d<l1+l2:
 raise ValueError(f'Unreachable natural arm target: distance={d}, length={l1+l2}, shoulder={list(rig.matrix_world@head)}')
axis=(target-head).normalized(); along=(l1*l1-l2*l2+d*d)/(2*d)
radius=math.sqrt(max(0,l1*l1-along*along)); bend=elbow-head-axis*(elbow-head).dot(axis)
if bend.length<1e-8: bend=Vector((0,0,1))-axis*axis.z
up=rig.matrix_world.inverted().to_3x3()@Vector((0,0,1))
bend=up-axis*up.dot(axis)
bend.normalize(); goal_elbow=head+axis*along+bend*radius

def aim(bone,point):
 h=bone.head.copy();q=(bone.tail-h).rotation_difference(point-h)
 bone.matrix=Matrix.Translation(h)@q.to_matrix().to_4x4()@Matrix.Translation(-h)@bone.matrix
 bpy.context.view_layer.update()
aim(upper,goal_elbow);aim(lower,target)
# Keep palm/fingers directed into the gap, horizontally; rotate original hand frame.
oldhead=Vector(before['hand_r'][0]); oldtail=Vector(before['hand_r'][1]); direction=Vector((.965926,.258819,0) if a.state=='gap' else (-.9,.435,0)).normalized()
rotation=(oldtail-oldhead).rotation_difference(direction)
# Recover original hand frame from the source rest of the saved pose independently.
current_world=rig.matrix_world@hand.matrix
q=(current_world.to_3x3()@Vector((0,1,0))).rotation_difference(direction)
new_world=Matrix.Translation(current_world.translation)@q.to_matrix().to_4x4()@Matrix.Translation(-current_world.translation)@current_world
hand.matrix=rig.matrix_world.inverted()@new_world
bpy.context.view_layer.update();after=bones()
unchanged=['Root','pelvis','thigh_l','calf_l','foot_l','ball_l','thigh_r','calf_r','foot_r','ball_r']
max_other=max((Vector(before[n][j])-Vector(after[n][j])).length for n in unchanged for j in (0,1))
error=(rig.matrix_world@hand.head-wrist_world).length
assert error<1e-5 and max_other<1e-5,(error,max_other)
for ob in bpy.data.objects:
 for mod in ob.modifiers:
  if mod.type=='ARMATURE':mod.use_deform_preserve_volume=True
bpy.context.view_layer.update();deps=bpy.context.evaluated_depsgraph_get()
meshes=[o for o in bpy.data.objects if o.type=='MESH'];bpy.ops.object.select_all(action='DESELECT');clones=[];hand_points=[]
for original in meshes:
 mesh=bpy.data.meshes.new_from_object(original.evaluated_get(deps),preserve_all_data_layers=True,depsgraph=deps)
 ob=bpy.data.objects.new('Posed_'+original.name,mesh);bpy.context.collection.objects.link(ob)
 ob.matrix_world=original.matrix_world.copy();ob.select_set(True);clones.append(ob)
 if original.name=='KitchenAdult':
  groupids={g.index for g in original.vertex_groups if g.name in {'hand_r',*[b.name for b in hand.children_recursive]}}
  hand_indices=[v.index for v in mesh.vertices if sum(g.weight for g in v.groups if g.group in groupids)>.5]
  hand_points=[list(ob.matrix_world@mesh.vertices[i].co) for i in hand_indices]
  triangles=[]
  selected=set(hand_indices)
  for poly in mesh.polygons:
   if all(i in selected for i in poly.vertices):
    ids=list(poly.vertices)
    triangles.extend([ids[0],ids[j],ids[j+1]] for j in range(1,len(ids)-1))
  remap={v:i for i,v in enumerate(hand_indices)}
  (out/'hand_surface.json').write_text(json.dumps({'points_asset_m':hand_points,'triangles':[[remap[i] for i in t] for t in triangles],'method':'Original connected body mesh vertices, hand/finger skin weight > 0.5; extracted only for CPU measurement, body remains complete.'},indent=2)+'\n')
for image in bpy.data.images:
 candidate=root/'assets/people/mpfb_corridor/textures'/Path(bpy.path.abspath(image.filepath)).name
 if candidate.is_file():image.filepath=str(candidate)
bpy.context.scene.render.engine='BLENDER_EEVEE'
bpy.ops.wm.usd_export(filepath=str(out/'person.usdc'),selected_objects_only=True,export_animation=False,export_materials=True,export_textures=True,overwrite_textures=True,relative_paths=True,root_prim_path='/Person')
report={'source':str(source.relative_to(root)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'state':a.state,'blender_version':bpy.app.version_string,'method':'Original MPFB skeleton with parameterized lumbar angle and natural-length two-bone right-arm IK; original connected skin and fingers retained. Original pelvis, legs, root, feet unchanged. Static evaluated complete human mesh, no animation.','bend_deg':a.bend_deg,'target_wrist_world_asset_m':list(wrist_world),'actual_wrist_world_asset_m':list(rig.matrix_world@hand.head),'wrist_error_m':error,'max_unchanged_bone_error_m':max_other,'before':before,'after':after,'hand_vertex_count':len(hand_points),'exported_meshes':[o.name for o in clones]}
(out/'pose.json').write_text(json.dumps(report,indent=2)+'\n')
print('HARD_HUMAN_POSE',a.state,error,max_other,len(hand_points),flush=True)
