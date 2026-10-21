"""Blender worker for the reproducible MPFB obstacle library; run via build_library_people.py."""
import argparse, json, math, sys
from pathlib import Path
import bpy
from mathutils import Vector, Matrix
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from human_pose_helpers import refine_library_arms, correct_sportsuit_coverage
OUT=ROOT/'assets/obstacle_library/people'
SHARED=ROOT/'assets/people/mpfb_library'
EXTRA=OUT/'source_library'
parser=argparse.ArgumentParser()
parser.add_argument('--person',choices=['medium_shirt','compact_sport','broad_workwear'],required=True)
parser.add_argument('--pose',choices=['palm_down','raised_wrist'])
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
CONFIGS={
 'medium_shirt':dict(height=1.78,gender=1.,age=.38,muscle=.45,weight=.48,clothing='male_casualsuit01',hair='short02',skin='young_asian_male',label='中等体型 / 长袖衬衫'),
 'compact_sport':dict(height=1.62,gender=0.,age=.32,muscle=.30,weight=.38,clothing='female_sportsuit01',hair='ponytail01',skin='young_asian_female',label='较矮纤细体型 / 运动服'),
 'broad_workwear':dict(height=1.86,gender=1.,age=.55,muscle=.72,weight=.72,clothing='male_worksuit01',hair='short02',skin='young_asian_male',label='较高壮实体型 / 工装'),
}
cfg=CONFIGS[args.person]
folder=OUT/args.person;folder.mkdir(parents=True,exist_ok=True)
source=folder/'source.blend'
def asset(rel):
 p=EXTRA/rel
 return p if p.exists() else SHARED/rel
if not source.exists():
 import addon_utils
 addon_utils.enable('bl_ext.user_default.mpfb',default_set=True)
 from bl_ext.user_default.mpfb.services.humanservice import HumanService
 from bl_ext.user_default.mpfb.services.targetservice import TargetService
 bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
 macro=TargetService.get_default_macro_info_dict()
 macro.update(gender=cfg['gender'],age=cfg['age'],muscle=cfg['muscle'],weight=cfg['weight'],height=.5)
 macro['race']={'asian':1.,'caucasian':0.,'african':0.}
 body=HumanService.create_human(macro_detail_dict=macro);body.name='LibraryAdult'
 HumanService.set_character_skin(str(asset(f"skins/{cfg['skin']}/{cfg['skin']}.mhmat")),body,skin_type='MAKESKIN')
 HumanService.add_builtin_rig(body,'game_engine')
 items=[(f"clothes/{cfg['clothing']}/{cfg['clothing']}.mhclo",'Clothes'),('clothes/shoes01/shoes01.mhclo','Clothes'),(f"hair/{cfg['hair']}/{cfg['hair']}.mhclo",'Hair'),('eyes/high-poly/high-poly.mhclo','Eyes'),('eyebrows/eyebrow001/eyebrow001.mhclo','Eyebrows')]
 for rel,kind in items:
  obj=HumanService.add_mhclo_asset(str(asset(rel)),body,asset_type=kind,subdiv_levels=1)
  if kind=='Eyes':
   mat=bpy.data.materials.new('BrownEyes');mat.use_nodes=True
   shader=mat.node_tree.nodes.get('Principled BSDF');tex=mat.node_tree.nodes.new('ShaderNodeTexImage')
   tex.image=bpy.data.images.load(str(asset('eyes/materials/brown_eye.png')))
   mat.node_tree.links.new(tex.outputs['Color'],shader.inputs['Base Color']);mat.node_tree.links.new(tex.outputs['Alpha'],shader.inputs['Alpha'])
   obj.data.materials.clear();obj.data.materials.append(mat)
  print('LOADED',args.person,rel,flush=True)
 bpy.ops.wm.save_as_mainfile(filepath=str(source))
for pose in [args.pose] if args.pose else ['palm_down','raised_wrist']:
 bpy.ops.wm.open_mainfile(filepath=str(source))
 rig=next(o for o in bpy.data.objects if o.type=='ARMATURE');meshes=[o for o in bpy.data.objects if o.type=='MESH']
 for mat in list(bpy.data.materials):
  if not mat.use_nodes:continue
  images=[n.image for n in mat.node_tree.nodes if n.type=='TEX_IMAGE' and n.image and ('diffuse' in n.name.lower() or mat.name=='BrownEyes')]
  if not images:continue
  image=images[0];nodes=mat.node_tree.nodes;nodes.clear()
  output=nodes.new('ShaderNodeOutputMaterial');shader=nodes.new('ShaderNodeBsdfPrincipled');tex=nodes.new('ShaderNodeTexImage');tex.image=image
  mat.node_tree.links.new(tex.outputs['Color'],shader.inputs['Base Color']);mat.node_tree.links.new(shader.outputs['BSDF'],output.inputs['Surface'])
  shader.inputs['Roughness'].default_value=.65;shader.inputs['Specular IOR Level'].default_value=.25
  if cfg['hair'] in mat.name or 'eyebrow' in mat.name or mat.name=='BrownEyes':mat.node_tree.links.new(tex.outputs['Alpha'],shader.inputs['Alpha'])
 deps=bpy.context.evaluated_depsgraph_get()
 points=[o.matrix_world@v.co for o in meshes for v in o.evaluated_get(deps).data.vertices]
 zmin=min(v.z for v in points);zmax=max(v.z for v in points);scale=cfg['height']/(zmax-zmin)
 root=bpy.data.objects.new('PersonPlacement',None);bpy.context.collection.objects.link(root)
 for obj in list(bpy.context.scene.objects):
  if obj!=root and obj.parent is None:obj.parent=root
 root.scale=(scale,)*3;root.rotation_euler.z=math.radians(-35);root.location.z=-zmin*scale
 bpy.context.view_layer.update()
 def point(name):return rig.matrix_world@rig.pose.bones[name].head
 def rotate(name,rotation):
  bone=rig.pose.bones[name];pivot=point(name)
  world=Matrix.Translation(pivot)@rotation.to_4x4()@Matrix.Translation(-pivot)@rig.matrix_world@bone.matrix
  bone.matrix=rig.matrix_world.inverted()@world;bpy.context.view_layer.update()
 def aim(name,child,target):
  current=point(child)-point(name);direction=Vector(target)-point(name)
  rotate(name,current.normalized().rotation_difference(direction.normalized()).to_matrix())
 rotate('spine_02',Matrix.Rotation(math.radians(26),3,Vector((-.4,-.6,0)).normalized()))
 s=point('upperarm_r');el=point('lowerarm_r');w=point('hand_r');l1=(el-s).length;l2=(w-el).length
 requested=Vector((-.60,.49,1.08) if pose=='palm_down' else (-.54,.48,1.11))
 u=(requested-s).normalized();d=min((requested-s).length,(l1+l2)*.985);target=s+u*d
 a=(l1*l1-l2*l2+d*d)/(2*d);h=math.sqrt(max(0,l1*l1-a*a))
 hint=Vector((-.22,.40,1.12) if pose=='palm_down' else (-.45,.16,1.10))-s
 v=(hint-u*hint.dot(u)).normalized();elbow=s+u*a+v*h
 aim('upperarm_r','lowerarm_r',elbow);aim('lowerarm_r','hand_r',target)
 hand=rig.pose.bones['hand_r'];direction=rig.matrix_world.to_3x3()@(hand.tail-hand.head)
 finger_direction=Vector((-.95,.3,-.05) if pose=='palm_down' else (-.95,.3,.20)).normalized()
 rotate('hand_r',direction.normalized().rotation_difference(finger_direction).to_matrix())
 # Keep the free arm relaxed alongside the torso.
 aim('upperarm_l','lowerarm_l',Vector((.18,-.15,1.08*cfg['height']/1.78)))
 aim('lowerarm_l','hand_l',point('lowerarm_l')+Vector((-.03,-.02,-.3*cfg['height']/1.78)))
 anatomy=refine_library_arms(rig,root,pose)
 for obj in meshes:
  for modifier in obj.modifiers:
   if modifier.type=='ARMATURE':modifier.use_deform_preserve_volume=True
 bpy.context.view_layer.update()
 if args.person=='compact_sport':anatomy['clothing_coverage']=correct_sportsuit_coverage(rig,meshes)
 out=folder/pose;out.mkdir(parents=True,exist_ok=True)
 wrist=list(point('hand_r'));yaw=-30.;rotation=Matrix.Rotation(math.radians(yaw),3,'Z')
 # Align the reaching hand with the unchanged bottle-to-bowl transport corridor.
 world_wrist=Vector((2.368849299 + (.03 if pose=='raised_wrist' else 0.),-.433469691,wrist[2]));xy=world_wrist-rotation@Vector(wrist)
 metadata={'id':f'human_{args.person}_{pose}','person':args.person,'pose':pose,'label':cfg['label']+' / '+('掌心向下伸手' if pose=='palm_down' else '抬腕伸手'),'nominal_standing_height_m':cfg['height'],'macro_parameters':{k:cfg[k] for k in ['gender','age','muscle','weight']},'clothing':cfg['clothing'],'hair':cfg['hair'],'wrist_local':wrist,'wrist_world':list(world_wrist),'elbow_local':list(point('lowerarm_r')),'shoulder_local':list(point('upperarm_r')),'hand_tip_local':list(rig.matrix_world@hand.tail),'arm_segment_lengths_m':[l1,l2],'position':[xy.x,xy.y,0.],'yaw_deg':yaw,'source_asset':str(source.relative_to(ROOT))}
 metadata['anatomy']=anatomy
 metadata['skin_deformation']='dual_quaternion_preserve_volume'
 (out/'pose.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
 bpy.ops.wm.save_as_mainfile(filepath=str(out/'posed.blend'))
 deps=bpy.context.evaluated_depsgraph_get();baked=[]
 for obj in meshes:
  evaluated=obj.evaluated_get(deps);mesh=bpy.data.meshes.new_from_object(evaluated,depsgraph=deps);mesh.transform(obj.matrix_world)
  copy=bpy.data.objects.new('Visual_'+obj.name,mesh);bpy.context.collection.objects.link(copy);baked.append(copy)
 for obj in list(bpy.context.scene.objects):
  if obj not in baked:bpy.data.objects.remove(obj,do_unlink=True)
 bpy.ops.object.select_all(action='SELECT')
 bpy.ops.wm.usd_export(filepath=str(out/'person.usdc'),selected_objects_only=True,export_armatures=False,export_animation=False,export_materials=True,export_lights=False,export_cameras=False,convert_world_material=False,generate_preview_surface=True,export_textures=True,overwrite_textures=True,root_prim_path='/Person',relative_paths=True,export_custom_properties=False)
 print('EXPORTED',metadata['id'],json.dumps(metadata['position']),flush=True)
