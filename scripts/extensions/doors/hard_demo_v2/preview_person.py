"""Five CPU stills for character motion review, not a video render."""
import bpy,json,math
from pathlib import Path
from mathutils import Vector
out=Path('assets/extensions/doors/hard_demo_v2/person');meta=json.loads((out/'animation.json').read_text());F=Vector(meta['asset_forward']);R=Vector(meta['asset_right']);U=Vector((0,0,1));s=bpy.context.scene
s.render.engine='CYCLES';s.cycles.device='CPU';s.cycles.samples=8;s.cycles.use_denoising=True;s.render.threads_mode='FIXED';s.render.threads=4;s.render.resolution_x=480;s.render.resolution_y=640;s.render.resolution_percentage=100
s.world.color=(.35,.35,.35)
bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.004));plane=bpy.context.object;mat=bpy.data.materials.new('review_floor');mat.diffuse_color=(.32,.35,.38,1);plane.data.materials.append(mat)
camdata=bpy.data.cameras.new('ReviewCamera');cam=bpy.data.objects.new('ReviewCamera',camdata);bpy.context.collection.objects.link(cam);s.camera=cam;camdata.type='ORTHO';camdata.ortho_scale=2.1
for name,pos,power,size in [('key',F*3+R*2+U*4,550,4),('fill',F*2-R*3+U*2,250,3)]:
 ld=bpy.data.lights.new(name,'AREA');ld.energy=power;ld.shape='DISK';ld.size=size;lo=bpy.data.objects.new(name,ld);bpy.context.collection.objects.link(lo);lo.location=pos;lo.rotation_euler=(Vector((0,0,1))-pos).to_track_quat('-Z','Y').to_euler()
for frame in (0,12,58,66,102):
 s.frame_set(frame);travel=Vector(meta['samples'][frame]['root_translation_asset_m']);target=travel+U*.9;cam.location=target+F*3+R*1.8+U*.2;cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler();s.render.filepath=str(out/('pose_%03d.png'%frame));bpy.ops.render.render(write_still=True)
