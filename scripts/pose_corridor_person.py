"""Pose either character, keep editable rig, export a baked static USD in metres."""
import sys, math, json
from pathlib import Path
import bpy
from mathutils import Vector, Matrix
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from human_pose_helpers import refine_library_arms
kind='mpfb'
source_folder=ROOT/'assets/people/mpfb'
folder=ROOT/'assets/people/mpfb_corridor'
folder.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=str(source_folder/'source.blend'))
rig=next(o for o in bpy.data.objects if o.type=='ARMATURE')
meshes=[o for o in bpy.data.objects if o.type=='MESH']
if kind=='nvidia':
    texture=folder/'People/Characters/original_male_adult_medical_01/textures/male_adult_medical_01_BaseColor.png'
    image=bpy.data.images.load(str(texture))
    for obj in meshes:
        mat=bpy.data.materials.new(obj.name+'_Preview');mat.use_nodes=True
        shader=mat.node_tree.nodes.get('Principled BSDF')
        tex=mat.node_tree.nodes.new('ShaderNodeTexImage');tex.image=image
        mat.node_tree.links.new(tex.outputs['Color'],shader.inputs['Base Color'])
        shader.inputs['Roughness'].default_value=.65
        obj.data.materials.clear();obj.data.materials.append(mat)
else:
    # Use a portable opaque skin/cloth shader. Only hair and eyebrow cards need alpha.
    for mat in list(bpy.data.materials):
        if not mat.use_nodes:continue
        images=[n.image for n in mat.node_tree.nodes if n.type=='TEX_IMAGE' and n.image
                and ('diffuse' in n.name.lower() or mat.name=='BrownEyes')]
        if not images:continue
        image=images[0];nodes=mat.node_tree.nodes;nodes.clear()
        output=nodes.new('ShaderNodeOutputMaterial');shader=nodes.new('ShaderNodeBsdfPrincipled')
        tex=nodes.new('ShaderNodeTexImage');tex.image=image
        mat.node_tree.links.new(tex.outputs['Color'],shader.inputs['Base Color'])
        mat.node_tree.links.new(shader.outputs['BSDF'],output.inputs['Surface'])
        shader.inputs['Roughness'].default_value=.65
        shader.inputs['Specular IOR Level'].default_value=.25
        if 'short02' in mat.name or 'eyebrow' in mat.name or mat.name=='BrownEyes':
            mat.node_tree.links.new(tex.outputs['Alpha'],shader.inputs['Alpha'])

# Normalize actual visible geometry, excluding the rig's terminal head bone.
deps=bpy.context.evaluated_depsgraph_get()
points=[o.matrix_world@v.co for o in meshes for v in o.evaluated_get(deps).data.vertices]
zmin=min(v.z for v in points);zmax=max(v.z for v in points)
scale=1.78/(zmax-zmin)
root=bpy.data.objects.new('PersonPlacement',None);bpy.context.collection.objects.link(root)
for obj in list(bpy.context.scene.objects):
    if obj!=root and obj.parent is None:obj.parent=root
root.scale=(scale,)*3;root.rotation_euler.z=math.radians(-35)
root.location.z=-zmin*scale
bpy.context.view_layer.update()

names=({'upper':'R_Upperarm','lower':'R_Forearm','hand':'R_Hand',
        'restupper':'L_Upperarm','restlower':'L_Forearm','resthand':'L_Hand','spine':'Waist'}
       if kind=='nvidia' else
       {'upper':'upperarm_r','lower':'lowerarm_r','hand':'hand_r',
        'restupper':'upperarm_l','restlower':'lowerarm_l','resthand':'hand_l','spine':'spine_02'})
def point(name):return rig.matrix_world@rig.pose.bones[name].head
def rotate(name,rotation):
    bone=rig.pose.bones[name];pivot=point(name)
    world=Matrix.Translation(pivot)@rotation.to_4x4()@Matrix.Translation(-pivot)@rig.matrix_world@bone.matrix
    bone.matrix=rig.matrix_world.inverted()@world
    bpy.context.view_layer.update()
def aim(name,child,target):
    current=point(child)-point(name)
    direction=Vector(target)-point(name)
    rotate(name,current.normalized().rotation_difference(direction.normalized()).to_matrix())

rotate(names['spine'],Matrix.Rotation(math.radians(26),3,Vector((-.4,-.6,0)).normalized()))
s=point(names['upper']);el=point(names['lower']);w=point(names['hand'])
l1=(el-s).length;l2=(w-el).length
target=Vector((-.60,.49,1.08));u=(target-s).normalized()
d=min((target-s).length,(l1+l2)*.985);target=s+u*d
a=(l1*l1-l2*l2+d*d)/(2*d);height=math.sqrt(max(0,l1*l1-a*a))
hint=Vector((-.22,.40,1.12))-s;v=(hint-u*hint.dot(u)).normalized()
elbow=s+u*a+v*height
aim(names['upper'],names['lower'],elbow)
aim(names['lower'],names['hand'],target)
hand=rig.pose.bones[names['hand']]
direction=rig.matrix_world.to_3x3()@(hand.tail-hand.head)
rotate(names['hand'],direction.normalized().rotation_difference(Vector((-.95,.3,-.05)).normalized()).to_matrix())
aim(names['restupper'],names['restlower'],Vector((.18,-.15,1.08)))
aim(names['restlower'],names['resthand'],point(names['restlower'])+Vector((-.03,-.02,-.3)))
anatomy=refine_library_arms(rig,root,'palm_down')
for obj in meshes:
    for modifier in obj.modifiers:
        if modifier.type=='ARMATURE':modifier.use_deform_preserve_volume=True
bpy.context.view_layer.update()
print('REACH',kind,tuple(point(names['hand'])),flush=True)
pose_metadata={'intended_wrist_local':[-.60,.49,1.08], 'wrist_local':list(point(names['hand'])), 'elbow_local':list(point(names['lower'])), 'shoulder_local':list(point(names['upper'])), 'arm_segment_lengths_m':[l1,l2], 'source_asset':'assets/people/mpfb/source.blend'}
pose_metadata['anatomy']=anatomy
pose_metadata['skin_deformation']='dual_quaternion_preserve_volume'
pose_metadata['hand_tip_local']=list(rig.matrix_world@hand.tail)
(folder/'pose.json').write_text(json.dumps(pose_metadata,indent=2)+'\n')
bpy.ops.wm.save_as_mainfile(filepath=str(folder/'posed.blend'))

# Bake evaluated deformations, subdivision and masks; preserve UVs and materials.
deps=bpy.context.evaluated_depsgraph_get()
baked=[]
for obj in meshes:
    evaluated=obj.evaluated_get(deps)
    mesh=bpy.data.meshes.new_from_object(evaluated,depsgraph=deps)
    mesh.transform(obj.matrix_world)
    copy=bpy.data.objects.new('Visual_'+obj.name,mesh)
    bpy.context.collection.objects.link(copy);baked.append(copy)
for obj in list(bpy.context.scene.objects):
    if obj not in baked:bpy.data.objects.remove(obj,do_unlink=True)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.wm.usd_export(filepath=str(folder/'person.usdc'),selected_objects_only=True,
    export_armatures=False,export_animation=False,export_materials=True,
    export_lights=False,export_cameras=False,convert_world_material=False,
    generate_preview_surface=True,export_textures=True,overwrite_textures=True,
    root_prim_path='/Person',relative_paths=True,export_custom_properties=False)
print('EXPORTED',kind,flush=True)
