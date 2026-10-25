"""Blender CPU: two-bone IK on the original MPFB rig, feet/root unchanged."""
import bpy,json,math,hashlib,argparse,sys
from pathlib import Path
from mathutils import Vector,Matrix
p=argparse.ArgumentParser();p.add_argument('--wrist',type=float,nargs=3,default=[-.59115082,.48153019,1.32]);p.add_argument('--asset-dir',default='assets/extensions/controls/human_reach_high');p.add_argument('--hand-orientation',choices=('source-world','follow-forearm'),default='source-world');p.add_argument('--arm-frame',choices=('legacy-aim','source-bend-plane'),default='legacy-aim');group=p.add_mutually_exclusive_group();group.add_argument('--elbow-up',action='store_true');group.add_argument('--elbow-down',action='store_true');group.add_argument('--elbow-direction',type=float,nargs=3);args=p.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
root=Path.cwd();source=root/'assets/people/mpfb_corridor/posed.blend';out=root/args.asset_dir
if any((out/name).exists() for name in ('person.usdc','pose.json','artifact_manifest.json')):raise FileExistsError('Refusing to overwrite a historical posed human: '+str(out))
out.mkdir(parents=True,exist_ok=True)
rig=bpy.data.objects['KitchenAdult.rig'];upper=rig.pose.bones['upperarm_r'];lower=rig.pose.bones['lowerarm_r'];hand=rig.pose.bones['hand_r']
before={b.name:[list(rig.matrix_world@b.head),list(rig.matrix_world@b.tail)] for b in rig.pose.bones}
head=upper.head.copy();elbow=lower.head.copy();wrist=hand.head.copy();l1=(elbow-head).length;l2=(wrist-elbow).length
target_world=Vector(args.wrist);target=rig.matrix_world.inverted()@target_world
d=(target-head).length
assert abs(l1-l2)<d<l1+l2
axis=(target-head).normalized();along=(l1*l1-l2*l2+d*d)/(2*d);radius=math.sqrt(max(0,l1*l1-along*along));bend=elbow-head-axis*(elbow-head).dot(axis);bend.normalize();goal_elbow=head+axis*along+bend*radius
if args.elbow_up or args.elbow_down or args.elbow_direction:
 up=rig.matrix_world.inverted().to_3x3()@Vector(args.elbow_direction or (0,0,-1 if args.elbow_down else 1));bend=up-axis*up.dot(axis);assert bend.length>1e-6;bend.normalize();goal_elbow=head+axis*along+bend*radius
hand_rotation=hand.matrix.to_3x3().copy()
source_relative_hand=lower.matrix.to_quaternion().inverted()@hand.matrix.to_quaternion()
source_wrist_bend=math.degrees((lower.tail-lower.head).angle(hand.tail-hand.head))
def aim(bone,point):
 h=bone.head.copy();q=(bone.tail-h).rotation_difference(point-h)
 bone.matrix=Matrix.Translation(h)@q.to_matrix().to_4x4()@Matrix.Translation(-h)@bone.matrix
 bpy.context.view_layer.update()
if args.arm_frame=='source-bend-plane':
 # Align both the upper-arm direction and the source elbow hinge plane.
 def frame(a,b):
  x=a.normalized();z=a.cross(b).normalized();y=z.cross(x)
  assert z.length>.99,'Degenerate elbow reference plane'
  return Matrix((x,y,z)).transposed()
 source_frame=frame(elbow-head,wrist-elbow)
 target_frame=frame(goal_elbow-head,target-goal_elbow)
 rotation=target_frame@source_frame.transposed()
 upper.matrix=Matrix.Translation(head)@rotation.to_4x4()@Matrix.Translation(-head)@upper.matrix
 bpy.context.view_layer.update()
else:aim(upper,goal_elbow)
aim(lower,target)
if args.hand_orientation=='follow-forearm':
 # Transport the source wrist frame with the forearm, not the world frame.
 hm=(lower.matrix.to_quaternion()@source_relative_hand).to_matrix().to_4x4()
else:hm=hand_rotation.to_4x4()
hm.translation=hand.head.copy();hand.matrix=hm;bpy.context.view_layer.update()
posed_relative_hand=lower.matrix.to_quaternion().inverted()@hand.matrix.to_quaternion()
wrist_structure={'mode':args.hand_orientation,'source_forearm_to_hand_deg':source_wrist_bend,
 'posed_forearm_to_hand_deg':math.degrees((lower.tail-lower.head).angle(hand.tail-hand.head)),
 'source_relative_hand_quaternion_wxyz':list(source_relative_hand),
 'posed_relative_hand_quaternion_wxyz':list(posed_relative_hand),
 'relative_orientation_delta_deg':math.degrees(source_relative_hand.rotation_difference(posed_relative_hand).angle),
 'scope':'Preserve the original rig wrist relationship; not a clinical range-of-motion certification.'}
if args.hand_orientation=='follow-forearm':
 assert wrist_structure['relative_orientation_delta_deg']<.001,wrist_structure
 assert abs(wrist_structure['posed_forearm_to_hand_deg']-source_wrist_bend)<.001,wrist_structure
after={b.name:[list(rig.matrix_world@b.head),list(rig.matrix_world@b.tail)] for b in rig.pose.bones}
affected={upper.name,*[b.name for b in upper.children_recursive]}
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
report={'source':str(source.relative_to(root)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'blender_version':bpy.app.version_string,'method':'Original armature two-bone IK; original skin weights and preserve-volume deformation; right shoulder pivot and arm lengths unchanged; same hand orientation; source root, feet and all other non-right-arm bones unchanged. Static evaluated mesh export.','target_wrist_world_asset_m':list(target_world),'actual_wrist_world_asset_m':list(rig.matrix_world@hand.head),'wrist_error_m':error,'max_unchanged_bone_error_m':max_other,'before':before,'after':after,'arm_lengths_local_m':[l1,l2],'arm_lengths_world_m':[l1*rig.matrix_world.to_scale()[0],l2*rig.matrix_world.to_scale()[0]],'exported_meshes':[o.name for o in clones]}
report['wrist_structure']=wrist_structure
report['arm_frame']=args.arm_frame
report['recipe_args']=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
if args.hand_orientation=='follow-forearm':report['method']=report['method'].replace('same hand orientation','source hand orientation relative to the newly posed forearm preserved')
if args.arm_frame=='source-bend-plane':report['method']+=' Shoulder frame maps the source elbow bend-plane normal to the target normal before hinge flexion; no independent arbitrary lower-arm swing.'
(out/'pose.json').write_text(json.dumps(report,indent=2));print('POSE_RESULT',error,max_other,len(clones))
