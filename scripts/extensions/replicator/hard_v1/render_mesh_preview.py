"""Render world-coordinate USD mesh exports with Blender Cycles, CPU only."""
import argparse
import gzip
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector


parser = argparse.ArgumentParser()
parser.add_argument("--geometry", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--eye", nargs=3, type=float, required=True)
parser.add_argument("--target", nargs=3, type=float, required=True)
parser.add_argument("--lens", type=float, default=42)
parser.add_argument("--samples", type=int, default=16)
parser.add_argument("--width", type=int, default=1100)
parser.add_argument("--height", type=int, default=850)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
opener = gzip.open if args.geometry.endswith(".gz") else open
with opener(args.geometry, "rt", encoding="utf-8") as stream:
    document = json.load(stream)
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
materials = {}


def get_material(spec, has_uv):
    key = json.dumps(spec, sort_keys=True) + str(has_uv)
    if key in materials:
        return materials[key]
    mat = bpy.data.materials.new("USD_preview_" + str(len(materials)))
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get("Principled BSDF")
    color = spec.get("texture_average", spec["color"]) if not has_uv else spec["color"]
    shader.inputs["Base Color"].default_value = (*color[:3], 1.)
    shader.inputs["Roughness"].default_value = spec.get("roughness", .5)
    shader.inputs["Metallic"].default_value = spec.get("metallic", 0.)
    opacity = spec.get("opacity", 1.)
    shader.inputs["Alpha"].default_value = opacity
    shader.inputs["IOR"].default_value = spec.get("ior", 1.45)
    if opacity < .99:
        shader.inputs["Transmission Weight"].default_value = .65
    if has_uv and spec.get("texture"):
        tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
        try:
            tex.image = bpy.data.images.load(spec["texture"], check_existing=True)
            mat.node_tree.links.new(tex.outputs["Color"], shader.inputs["Base Color"])
        except RuntimeError:
            pass
    materials[key] = mat
    return mat


for item in document["objects"]:
    if item["kind"] == "curve":
        data = bpy.data.curves.new(item["path"], "CURVE")
        data.dimensions = "3D"
        data.bevel_depth = item["widths"][0] / 2.
        data.bevel_resolution = 2
        offset = 0
        for count in item["counts"]:
            spline = data.splines.new("POLY")
            spline.points.add(count - 1)
            for p, co in zip(spline.points, item["points"][offset:offset+count]):
                p.co = (*co, 1.)
            spline.use_cyclic_u = item["periodic"]
            offset += count
    else:
        data = bpy.data.meshes.new(item["path"])
        data.from_pydata(item["points"], [], item["faces"])
        data.update()
        if item.get("loop_uv") and len(item["loop_uv"]) == len(data.loops):
            uv = data.uv_layers.new(name="USD_st")
            uv.data.foreach_set("uv", [v for pair in item["loop_uv"] for v in pair])
    obj = bpy.data.objects.new(item["path"], data)
    scene.collection.objects.link(obj)
    data.materials.append(get_material(item["material"], bool(item.get("loop_uv"))))

scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = args.samples
scene.cycles.use_denoising = True
scene.render.threads_mode = "FIXED"
scene.render.threads = 4
scene.render.resolution_x, scene.render.resolution_y = args.width, args.height
scene.render.resolution_percentage = 100
scene.view_settings.exposure = -.65
scene.world = bpy.data.worlds.new("CPU_preview_world")
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs[0].default_value = (.64,.68,.75,1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value = .4
camera_data = bpy.data.cameras.new("PreviewCamera")
camera = bpy.data.objects.new("PreviewCamera", camera_data)
scene.collection.objects.link(camera)
camera.location = args.eye
camera.rotation_euler = (Vector(args.target) - camera.location).to_track_quat("-Z", "Y").to_euler()
camera_data.lens = args.lens
camera_data.clip_start = .02
scene.camera = camera
for name, loc, energy, size in [("Key", (1.5,-.5,2.5), 250, 2), ("Fill", (2.5,.6,2.2), 160, 1.5)]:
    light = bpy.data.lights.new(name, "AREA")
    light.energy, light.shape, light.size = energy, "DISK", size
    obj = bpy.data.objects.new(name, light)
    scene.collection.objects.link(obj)
    obj.location = loc
    obj.rotation_euler = (Vector(args.target) - obj.location).to_track_quat("-Z", "Y").to_euler()
scene.render.image_settings.file_format = "PNG"
scene.render.filepath = str(Path(args.output).resolve())
Path(args.output).resolve().parent.mkdir(parents=True, exist_ok=True)
bpy.ops.render.render(write_still=True)
provenance = {"source_scene": document["source_scene"], "source_scene_sha256": document["source_sha256"], "output": scene.render.filepath, "renderer": "Blender " + bpy.app.version_string + " Cycles CPU; four threads", "scope": document["scope"], "counts": document["counts"], "unsupported_visible_geometry": document["skipped_types"], "camera_eye": args.eye, "camera_target": args.target, "lens_mm": args.lens, "samples": args.samples, "geometry_moved_or_hidden_for_preview": False, "physics_stepped": False, "trajectories_generated": False}
Path(args.output).with_suffix(".preview.json").write_text(json.dumps(provenance, indent=2) + "\n")
print("CPU_MESH_PREVIEW", scene.render.filepath, document["counts"])
