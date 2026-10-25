"""Blender CPU: two-bone IK on the original MPFB rig, feet/root unchanged."""
import bpy,json,math,hashlib,argparse,sys
from pathlib import Path
from mathutils import Vector,Matrix
p=argparse.ArgumentParser();p.add_argument('--wrist',type=float,nargs=3,default=[-.59115082,.48153019,1.32]);p.add_argument('--asset-dir',default='assets/extensions/doors/human_reach_high');p.add_argument('--elbow-up',action='store_true');p.add_argument('--clavicle-y-deg',type=float,default=45.);args=p.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
root=Path.cwd();source=root/'assets/people/mpfb_corridor/posed.blend';out=root/args.asset_dir;out.mkdir(parents=True,exist_ok=True)
rig=bpy.data.objects['KitchenAdult.rig'];upper=rig.pose.bones['upperarm_r'];lower=rig.pose.bones['lowerarm_r'];hand=rig.pose.bones['hand_r']
before={b.name:[list(rig.matrix_world@b.head),list(rig.matrix_world@b.tail)] for b in rig.pose.bones}
source_hand_rotation=hand.matrix.to_3x3().copy()
clav=rig.pose.bones['clavicle_r'];pivot=clav.head.copy()
world_rotation=Matrix.Rotation(math.radians(args.clavicle_y_deg),3,'Y')
local_rotation=rig.matrix_world.to_3x3().inverted()@world_rotation@rig.matrix_world.to_3x3()
clav.matrix=Matrix.Translation(pivot)@local_rotation.to_4x4()@Matrix.Translation(-pivot)@clav.matrix
bpy.context.view_layer.update()
head=upper.head.copy();elbow=lower.head.copy();wrist=hand.head.copy();l1=(elbow-head).length;l2=(wrist-elbow).length
target_world=Vector(args.wrist);target=rig.matrix_world.inverted()@target_world
d=(target-head).length
assert abs(l1-l2)<d<l1+l2
axis=(target-head).normalized();along=(l1*l1-l2*l2+d*d)/(2*d);radius=math.sqrt(max(0,l1*l1-along*along));bend=elbow-head-axis*(elbow-head).dot(axis);bend.normalize();goal_elbow=head+axis*along+bend*radius
if args.elbow_up:
 up=rig.matrix_world.inverted().to_3x3()@Vector((0,0,1));bend=(up-axis*up.dot(axis)).normalized();goal_elbow=head+axis*along+bend*radius
hand_rotation=source_hand_rotation.copy()
def aim(bone,point):
 h=bone.head.copy();q=(bone.tail-h).rotation_difference(point-h)
 bone.matrix=Matrix.Translation(h)@q.to_matrix().to_4x4()@Matrix.Translation(-h)@bone.matrix
 bpy.context.view_layer.update()
aim(upper,goal_elbow);aim(lower,target)
hm=hand_rotation.to_4x4();hm.translation=hand.head.copy();hand.matrix=hm;bpy.context.view_layer.update()
after={b.name:[list(rig.matrix_world@b.head),list(rig.matrix_world@b.tail)] for b in rig.pose.bones}
affected={clav.name,*[b.name for b in clav.children_recursive]}
unchanged=[n for n in before if n not in affected]
max_other=max((Vector(before[n][j])-Vector(after[n][j])).length for n in unchanged for j in [0,1])
error=(rig.matrix_world@hand.head-target_world).length
print('POSE_NUMERIC_CHECK',error,max_other,flush=True)
assert error<1e-5 and max_other<1e-5
for ob in bpy.data.objects:
 for mod in ob.modifiers:
  if mod.type=='ARMATURE':mod.use_deform_preserve_volume=True
bpy.context.view_layer.update();deps=bpy.context.evaluated_depsgraph_get();meshes=[o for o in bpy.data.objects if o.type=='MESH'];bpy.ops.object.select_all(action='DESELECT');clones=[]
for original in meshes:
 mesh=bpy.data.meshes.new_from_object(original.evaluated_get(deps),preserve_all_data_layers=True,depsgraph=deps)
 ob=bpy.data.objects.new('Posed_'+original.name,mesh);bpy.context.collection.objects.link(ob);ob.matrix_world=original.matrix_world.copy();ob.select_set(True);clones.append(ob)
bpy.context.scene.render.engine='BLENDER_EEVEE'
for image in bpy.data.images:
 candidate=root/'assets/people/mpfb_corridor/textures'/Path(bpy.path.abspath(image.filepath)).name
 if candidate.is_file():image.filepath=str(candidate)
bpy.ops.wm.usd_export(filepath=str(out/'person.usdc'),selected_objects_only=True,export_animation=False,export_materials=True,export_textures=True,overwrite_textures=True,relative_paths=True,root_prim_path='/Person')
report={'source':str(source.relative_to(root)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'blender_version':bpy.app.version_string,'method':'Original MPFB rig: right clavicle rotated about its unchanged head around world Y, then original-length two-bone arm IK. Skin weights and all non-right-clavicle-descendant bones unchanged, including root and feet; original hand world orientation. Static evaluated mesh export.','clavicle_world_y_rotation_deg':args.clavicle_y_deg,'target_wrist_world_asset_m':list(target_world),'actual_wrist_world_asset_m':list(rig.matrix_world@hand.head),'wrist_error_m':error,'max_unchanged_bone_error_m':max_other,'before':before,'after':after,'arm_lengths_local_m':[l1,l2],'arm_lengths_world_m':[l1*rig.matrix_world.to_scale()[0],l2*rig.matrix_world.to_scale()[0]],'exported_meshes':[o.name for o in clones]}
(out/'pose.json').write_text(json.dumps(report,indent=2));print('POSE_RESULT',error,max_other,len(clones))
