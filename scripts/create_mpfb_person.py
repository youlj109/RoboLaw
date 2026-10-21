"""Build an editable clothed MPFB character using selected CC0 system assets."""
import sys
from pathlib import Path
import bpy
ROOT=Path(__file__).resolve().parents[1]
import addon_utils
addon_utils.enable('bl_ext.user_default.mpfb',default_set=True)
from bl_ext.user_default.mpfb.services.humanservice import HumanService
from bl_ext.user_default.mpfb.services.targetservice import TargetService

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
macro=TargetService.get_default_macro_info_dict()
macro.update(gender=1.0,age=.38,muscle=.45,weight=.48,height=.5)
macro['race']={'asian':1.,'caucasian':0.,'african':0.}
body=HumanService.create_human(macro_detail_dict=macro)
body.name='KitchenAdult'
lib=ROOT/'assets/people/mpfb_library'
HumanService.set_character_skin(str(lib/'skins/young_asian_male/young_asian_male.mhmat'),body,skin_type='MAKESKIN')
rig=HumanService.add_builtin_rig(body,'game_engine')
for path,kind in [
    ('clothes/male_casualsuit01/male_casualsuit01.mhclo','Clothes'),
    ('clothes/shoes01/shoes01.mhclo','Clothes'),
    ('hair/short02/short02.mhclo','Hair'),
    ('eyes/high-poly/high-poly.mhclo','Eyes'),
    ('eyebrows/eyebrow001/eyebrow001.mhclo','Eyebrows')]:
    asset=HumanService.add_mhclo_asset(str(lib/path),body,asset_type=kind,subdiv_levels=1)
    print('LOADED',path,flush=True)
    if kind=='Eyes':
        # System eye geometry has no default material; build a simple textured surface.
        mat=bpy.data.materials.new('BrownEyes');mat.use_nodes=True
        shader=mat.node_tree.nodes.get('Principled BSDF')
        tex=mat.node_tree.nodes.new('ShaderNodeTexImage')
        tex.image=bpy.data.images.load(str(lib/'eyes/materials/brown_eye.png'))
        mat.node_tree.links.new(tex.outputs['Color'],shader.inputs['Base Color'])
        mat.node_tree.links.new(tex.outputs['Alpha'],shader.inputs['Alpha'])
        shader.inputs['Roughness'].default_value=.25
        asset.data.materials.clear();asset.data.materials.append(mat)
print('BONES',[(b.name,tuple(b.head),tuple(b.tail)) for b in rig.pose.bones],flush=True)
out=ROOT/'assets/people/mpfb';out.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=str(out/'source.blend'))
