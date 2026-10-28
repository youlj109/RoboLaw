"""Render compact USD animation exports with Cycles CPU; never simulate robotics."""
import argparse
import gzip
import json
from pathlib import Path
import sys
import time
import bpy
from mathutils import Matrix, Vector

parser=argparse.ArgumentParser()
parser.add_argument("--animation",required=True)
parser.add_argument("--output-dir",required=True)
parser.add_argument("--samples",type=int,default=4)
parser.add_argument("--width",type=int,default=960)
parser.add_argument("--height",type=int,default=720)
parser.add_argument("--threads",type=int,default=6)
parser.add_argument("--indices",nargs="*",type=int,help="Optional representative frames; earlier updates are still applied")
args=parser.parse_args(sys.argv[sys.argv.index("--")+1:])
if not 1 <= args.threads <= 6:
    raise ValueError("This preview renderer is limited to six CPU threads")
opener=gzip.open if args.animation.endswith(".gz") else open
with opener(args.animation,"rt",encoding="utf-8") as stream:
    document=json.load(stream)
bpy.ops.wm.read_factory_settings(use_empty=True)
scene=bpy.context.scene
materials={}


def material(spec,has_uv):
    key=json.dumps(spec,sort_keys=True)+str(has_uv)
    if key in materials:return materials[key]
    mat=bpy.data.materials.new("USD_material_"+str(len(materials)));mat.use_nodes=True
    shader=mat.node_tree.nodes.get("Principled BSDF")
    color=spec.get("texture_average",spec["color"]) if not has_uv else spec["color"]
    shader.inputs["Base Color"].default_value=(*color[:3],1.)
    shader.inputs["Roughness"].default_value=spec.get("roughness",.5)
    shader.inputs["Metallic"].default_value=spec.get("metallic",0.)
    opacity=spec.get("opacity",1.)
    shader.inputs["Alpha"].default_value=opacity
    shader.inputs["IOR"].default_value=spec.get("ior",1.45)
    if opacity<.99:shader.inputs["Transmission Weight"].default_value=.65
    if has_uv and spec.get("texture"):
        tex=mat.node_tree.nodes.new("ShaderNodeTexImage")
        try:
            tex.image=bpy.data.images.load(spec["texture"],check_existing=True)
            mat.node_tree.links.new(tex.outputs["Color"],shader.inputs["Base Color"])
        except RuntimeError:pass
    materials[key]=mat
    return mat


objects={}
for item in document["objects"]:
    if item["kind"]=="curve":
        data=bpy.data.curves.new(item["path"],"CURVE");data.dimensions="3D"
        data.bevel_depth=item["widths"][0]/2.;data.bevel_resolution=2
        offset=0
        for count in item["counts"]:
            spline=data.splines.new("POLY");spline.points.add(count-1)
            for point,co in zip(spline.points,item["points"][offset:offset+count]):point.co=(*co,1.)
            spline.use_cyclic_u=item["periodic"];offset+=count
    else:
        data=bpy.data.meshes.new(item["path"])
        data.from_pydata(item["points"],[],item["faces"]);data.update()
        if '/human/' in item['path']:
            for polygon in data.polygons:
                polygon.use_smooth = True
        if item.get("loop_uv") and len(item["loop_uv"])==len(data.loops):
            uv=data.uv_layers.new(name="USD_st");uv.data.foreach_set("uv",[v for pair in item["loop_uv"] for v in pair])
    obj=bpy.data.objects.new(item["path"],data);scene.collection.objects.link(obj)
    data.materials.append(material(item["material"],bool(item.get("loop_uv"))))
    obj.hide_render=not item["visible"];objects[item["id"]]=obj
scene.render.engine="CYCLES";scene.cycles.device="CPU"
scene.cycles.samples=args.samples;scene.cycles.use_denoising=True
scene.cycles.max_bounces=5;scene.cycles.transparent_max_bounces=6
scene.render.threads_mode="FIXED";scene.render.threads=args.threads
scene.render.use_persistent_data=True
scene.render.resolution_x=args.width;scene.render.resolution_y=args.height
scene.render.resolution_percentage=100;scene.render.fps=document["fps"]
scene.view_settings.exposure=-.65
scene.world=bpy.data.worlds.new("CPU_demo_world");scene.world.use_nodes=True
scene.world.node_tree.nodes["Background"].inputs[0].default_value=(.64,.68,.75,1.)
scene.world.node_tree.nodes["Background"].inputs[1].default_value=.4
camera_data=bpy.data.cameras.new("Authored_USD_camera")
camera=bpy.data.objects.new("Authored_USD_camera",camera_data);scene.collection.objects.link(camera)
scene.camera=camera;camera_data.clip_start=.015;camera_data.sensor_fit="HORIZONTAL"
first_matrix=Matrix(document["frames"][0]["camera"]["matrix"]).transposed()
first_eye=first_matrix.translation
forward=first_matrix.to_3x3()@Vector((0,0,-1))
right=first_matrix.to_3x3()@Vector((1,0,0))
light_target=first_eye+forward*1.0
# Lighting follows the source camera's scene location, including kitchens away
# from the original G origin. It changes shading only, never object placement.
for name,loc,energy,size in [("Key",first_eye+Vector((0,0,1.1))-right*.5,180,2.),("Fill",first_eye+right*.8+Vector((0,0,.6)),100,1.5)]:
    light=bpy.data.lights.new(name,"AREA");light.energy=energy;light.shape="DISK";light.size=size
    obj=bpy.data.objects.new(name,light);scene.collection.objects.link(obj);obj.location=loc
    obj.rotation_euler=(light_target-obj.location).to_track_quat("-Z","Y").to_euler()
out=Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
scene.render.image_settings.file_format="PNG"
scene.render.image_settings.color_mode="RGB"
timings=[];selected=set(args.indices) if args.indices is not None else None
for frame in document["frames"]:
    for update in frame["updates"]:
        obj=objects[update["id"]]
        if "visible" in update:obj.hide_render=not update["visible"]
        if "matrix" in update:obj.matrix_world=Matrix(update["matrix"]).transposed()
        if "points" in update:
            if obj.type=="MESH":
                obj.data.vertices.foreach_set("co",[v for point in update["points"] for v in point]);obj.data.update()
            else:
                offset=0
                for spline in obj.data.splines:
                    for point,co in zip(spline.points,update["points"][offset:offset+len(spline.points)]):point.co=(*co,1.)
                    offset+=len(spline.points)
    camera.matrix_world=Matrix(frame["camera"]["matrix"]).transposed()
    camera_data.lens=frame["camera"]["focal_length"]
    camera_data.sensor_width=frame["camera"]["horizontal_aperture"]
    if selected is not None and frame["frame"] not in selected:continue
    scene.frame_set(frame["frame"])
    scene.render.filepath=str(out/f"frame_{frame['frame']:06d}.png")
    started=time.monotonic();bpy.ops.render.render(write_still=True)
    elapsed=time.monotonic()-started;timings.append({"frame":frame["frame"],"render_seconds":elapsed})
    print("CPU_DEMO_FRAME",frame["frame"],round(elapsed,3),flush=True)
report={"source_scene":document["source_scene"],"source_scene_sha256":document["source_sha256"],"scope":document["scope"],"renderer":"Blender "+bpy.app.version_string+" Cycles CPU","threads":args.threads,"samples":args.samples,"fps":document["fps"],"frame_count":document["frame_count"],"resolution":[args.width,args.height],"counts":document["counts"],"unsupported_visible_geometry":document["skipped_types"],"camera_relative_lighting":True,"geometry_changed_beyond_authored_USD_samples":False,"physics_stepped":False,"robot_trajectories_generated":False,"timings":timings}
(out.parent/(out.name+".render.json")).write_text(json.dumps(report,indent=2)+"\n")
print("CPU_DEMO_COMPLETE",str(out),flush=True)
