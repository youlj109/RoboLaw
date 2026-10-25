"""CPU asset preview only: identical camera/light for old and revised wrists."""
import argparse
import sys
from pathlib import Path

import bpy
from mathutils import Vector

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--assets',nargs='+',required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args(sys.argv[sys.argv.index('--')+1:])
a.output.mkdir(parents=True,exist_ok=False)
for asset in a.assets:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(Path(asset).resolve()/'person.usdc'))
    scene=bpy.context.scene
    scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=12
    scene.render.threads_mode='FIXED';scene.render.threads=4
    scene.render.resolution_x=900;scene.render.resolution_y=700;scene.render.resolution_percentage=100
    scene.world=bpy.data.worlds.new('NeutralWorld');scene.world.use_nodes=True
    scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.35,.35,.35,1)
    scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.6
    for location,energy,size in (((-1.1,.8,2.5),180,1.5),((-.9,-.8,1.8),80,1.)):
        bpy.ops.object.light_add(type='AREA',location=location)
        light=bpy.context.object;light.data.energy=energy;light.data.shape='DISK';light.data.size=size
        light.rotation_euler=(Vector((-.5,.03,1.25))-light.location).to_track_quat('-Z','Y').to_euler()
    bpy.ops.object.camera_add(location=(-1.15,.7,1.6))
    camera=bpy.context.object
    camera.rotation_euler=(Vector((-.48,.04,1.27))-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.type='ORTHO';camera.data.ortho_scale=.68;scene.camera=camera
    scene.render.filepath=str((a.output/(Path(asset).name+'.png')).resolve())
    bpy.ops.render.render(write_still=True)
