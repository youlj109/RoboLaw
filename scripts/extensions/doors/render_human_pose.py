"""CPU visual check of the exported posed USD, without Isaac or GPU."""
import bpy,math
from pathlib import Path
from mathutils import Vector
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
bpy.ops.wm.usd_import(filepath=str(Path('assets/extensions/doors/human_reach_high/person.usdc').resolve()))
scene=bpy.context.scene;scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=16;scene.render.threads_mode='FIXED';scene.render.threads=4
scene.world.color=(.25,.25,.25)
bpy.ops.object.camera_add(location=(3,-4,2.2));cam=bpy.context.object;cam.rotation_euler=(Vector((-.15,.2,.85))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=2.0;scene.camera=cam
bpy.ops.object.light_add(type='AREA',location=(1,-2,4));bpy.context.object.data.energy=450;bpy.context.object.data.shape='DISK';bpy.context.object.data.size=4
scene.render.resolution_x=800;scene.render.resolution_y=800;scene.render.resolution_percentage=100;scene.render.filepath=str(Path('reports/avoidance_trials/doors/human_pose.png').resolve());scene.render.image_settings.file_format='PNG'
bpy.ops.render.render(write_still=True)
