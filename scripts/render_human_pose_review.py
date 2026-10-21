"""CPU-only Blender pose review; never changes the benchmark camera assets.

The two external views use the actual fixed DROID camera mounts and intrinsics.
They show the human alone in neutral clay, so these are geometry review images,
not Isaac Sim rollout frames. The optional orthographic views are diagnostics.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Quaternion, Vector

parser = argparse.ArgumentParser()
parser.add_argument('--blend', required=True)
parser.add_argument('--pose-json', required=True)
parser.add_argument('--cameras-json', required=True)
parser.add_argument('--output', required=True)
parser.add_argument('--tag', default='human')
parser.add_argument('--diagnostic', action='store_true')
args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=str(Path(args.blend).resolve()))
pose = json.loads(Path(args.pose_json).read_text())
cameras = json.loads(Path(args.cameras_json).read_text())
scene = bpy.context.scene
scene.render.engine = 'CYCLES'
scene.cycles.device = 'CPU'
scene.cycles.samples = 16
scene.world.color = (.18, .18, .18)
for obj in bpy.data.objects:
    if obj.type != 'MESH':
        continue
    mat = bpy.data.materials.new(obj.name + '_ReviewClay')
    body = obj.name.endswith('Adult') or obj.name in ('Human', 'Human.body')
    garment = any(s in obj.name.lower() for s in ('suit', 'shirt', 'pants'))
    color = (.55, .33, .19, 1) if body else ((.08, .24, .55, 1) if garment else (.08, .055, .04, 1))
    mat.diffuse_color = color
    mat.use_nodes = True
    mat.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = color
    mat.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = .7
    obj.data.materials.clear()
    obj.data.materials.append(mat)
for name, location, energy in [('Key', (-2, -3, 4), 550), ('Fill', (2, 3, 3), 400)]:
    light = bpy.data.lights.new(name, 'AREA')
    light.energy = energy
    light.size = 4
    obj = bpy.data.objects.new(name, light)
    bpy.context.collection.objects.link(obj)
    obj.location = location
camera = bpy.data.cameras.new('PoseReviewCamera')
camera.clip_start = .01
obj = bpy.data.objects.new('PoseReviewCamera', camera)
bpy.context.collection.objects.link(obj)
scene.camera = obj
placement_rotation = Matrix.Rotation(math.radians(pose.get('yaw_deg', -30)), 3, 'Z')
placement_translation = Vector(pose['position'])
base_rotation = Matrix.Rotation(math.pi / 2, 3, 'Z')
base_translation = Vector((2.2, -.93, .8))
for name in ('external_camera', 'external_camera_2'):
    config = cameras[name]
    offset = config['offset']
    x, y, z, w = offset['quaternion_xyzw']
    world_rotation = base_rotation @ Quaternion((w, x, y, z)).normalized().to_matrix()
    world_position = base_translation + base_rotation @ Vector(offset['position'])
    obj.location = placement_rotation.transposed() @ (world_position - placement_translation)
    obj.rotation_euler = (placement_rotation.transposed() @ world_rotation).to_euler()
    camera.type = 'PERSP'
    camera.sensor_fit = 'HORIZONTAL'
    camera.sensor_width = 36
    camera.lens = config['intrinsics'][0][0] * 36 / config['width']
    scene.render.resolution_x = config['width']
    scene.render.resolution_y = config['height']
    scene.render.resolution_percentage = 100
    scene.render.filepath = str(out / f'{args.tag}_{name}.png')
    bpy.ops.render.render(write_still=True)
if args.diagnostic:
    camera.type = 'ORTHO'
    camera.ortho_scale = 1.95
    scene.render.resolution_x = 720
    scene.render.resolution_y = 960
    for label, position in [('front', (-2, -3, 1.7)), ('back', (2, 3, 1.7))]:
        obj.location = position
        obj.rotation_euler = (Vector((-.16, .10, .82)) - obj.location).to_track_quat('-Z', 'Y').to_euler()
        scene.render.filepath = str(out / f'{args.tag}_{label}.png')
        bpy.ops.render.render(write_still=True)
