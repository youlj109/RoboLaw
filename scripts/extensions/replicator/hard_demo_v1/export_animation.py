"""Compact actual USD time-sample export for the CPU demonstration renderer.

World-space base geometry is stored once. Later frames contain only changing
transforms, visibility and authored mesh/curve point samples. Hidden collision
meshes stay excluded; geometry that becomes visible later remains available.
"""
from __future__ import annotations
import argparse
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path

for _option in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_option] = "4"
import numpy as np
from pxr import Usd, UsdGeom

_preview_path = Path(__file__).resolve().parents[1] / "hard_v1/export_preview.py"
_spec = importlib.util.spec_from_file_location("world_preview", _preview_path)
preview = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(preview)


def varying_ancestors(prim, visibility=False):
    while prim and not prim.IsPseudoRoot():
        if visibility:
            attr = prim.GetAttribute("visibility")
            if attr and attr.ValueMightBeTimeVarying():
                return True
        elif prim.IsA(UsdGeom.Xformable):
            if any(op.GetAttr().ValueMightBeTimeVarying() for op in UsdGeom.Xformable(prim).GetOrderedXformOps()):
                return True
        prim = prim.GetParent()
    return False


def export(scene_path, output, camera_path, fps=12, frames=96):
    scene_path = Path(scene_path).resolve()
    stage = Usd.Stage.Open(str(scene_path))
    if not stage or stage.GetCompositionErrors():
        raise ValueError("A complete USD scene is required")
    camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
    if not camera:
        raise ValueError(f"Missing animated camera {camera_path}")
    times = [stage.GetStartTimeCode() + index * stage.GetTimeCodesPerSecond() / fps for index in range(frames)]
    caches = [UsdGeom.XformCache(Usd.TimeCode(time)) for time in times]
    result = {"schema_version": 1, "source_scene": str(scene_path), "source_sha256": hashlib.sha256(scene_path.read_bytes()).hexdigest(), "scope": "Actual source USD scene time samples, CPU shaded demonstration only; no robot control, trajectories or physics acceptance", "fps": fps, "frame_count": frames, "time_codes": times, "objects": [], "frames": [{"frame": index, "time_code": time, "updates": []} for index,time in enumerate(times)], "skipped_types": {}}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputePurpose() in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy):
            continue
        visibility_varies = varying_ancestors(prim, visibility=True)
        visible = [imageable.ComputeVisibility(Usd.TimeCode(t)) != UsdGeom.Tokens.invisible for t in times] if visibility_varies else [imageable.ComputeVisibility(Usd.TimeCode(times[0])) != UsdGeom.Tokens.invisible] * frames
        if not any(visible):
            continue
        is_curve = prim.IsA(UsdGeom.BasisCurves)
        if is_curve:
            curve = UsdGeom.BasisCurves(prim)
            points = np.asarray(curve.GetPointsAttr().Get(times[0]), dtype=float)
            extra = {"counts": list(curve.GetCurveVertexCountsAttr().Get(times[0])), "widths": list(curve.GetWidthsAttr().Get(times[0]) or [.001]), "periodic": curve.GetWrapAttr().Get() == UsdGeom.Tokens.periodic}
            faces = None
        elif prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            if mesh.GetFaceVertexCountsAttr().ValueMightBeTimeVarying() or mesh.GetFaceVertexIndicesAttr().ValueMightBeTimeVarying():
                raise ValueError(f"Topology animation is unsupported: {prim.GetPath()}")
            points, faces, extra = preview.mesh_data(prim)
            points = np.asarray(mesh.GetPointsAttr().Get(times[0]), dtype=float)
        else:
            data = preview.primitive_data(prim)
            if data is None:
                kind = prim.GetTypeName()
                result["skipped_types"][kind] = result["skipped_types"].get(kind,0)+1
                continue
            if any(prim.GetAttribute(name) and prim.GetAttribute(name).ValueMightBeTimeVarying() for name in ("radius", "height", "size")):
                raise ValueError(f"Animate xform scale instead of primitive dimensions: {prim.GetPath()}")
            points, faces, extra = data
        transform = np.asarray(caches[0].GetLocalToWorldTransform(prim))
        world = np.column_stack([points,np.ones(len(points))]) @ transform
        obj = {"id": len(result["objects"]), "path": str(prim.GetPath()), "kind": "curve" if is_curve else "mesh", "points": np.round(world[:,:3],8).tolist(), "material": preview.material(prim), "visible": visible[0], **extra}
        if faces is not None:
            if np.linalg.det(transform[:3,:3]) < 0:
                faces = [list(reversed(face)) for face in faces]
                if "loop_uv" in obj:
                    old_uv, obj["loop_uv"], offset = obj["loop_uv"], [], 0
                    for face in faces:
                        obj["loop_uv"].extend(reversed(old_uv[offset:offset+len(face)]))
                        offset += len(face)
            obj["faces"] = faces
        result["objects"].append(obj)
        transform_varies = varying_ancestors(prim)
        point_attr = prim.GetAttribute("points")
        points_vary = bool(point_attr and point_attr.ValueMightBeTimeVarying())
        inverse_base = np.linalg.inv(transform) if transform_varies else None
        old_matrix, old_points = np.eye(4), points
        for index in range(1,frames):
            update = {"id": obj["id"]}
            if visible[index] != visible[index-1]:
                update["visible"] = visible[index]
            if transform_varies:
                delta = inverse_base @ np.asarray(caches[index].GetLocalToWorldTransform(prim))
                if not np.allclose(delta,old_matrix,atol=1e-10,rtol=0):
                    update["matrix"] = delta.tolist()
                    old_matrix = delta
            if points_vary:
                current_points = np.asarray(point_attr.Get(times[index]),dtype=float)
                if current_points.shape != old_points.shape:
                    raise ValueError(f"Point count changed: {prim.GetPath()}")
                if not np.array_equal(current_points,old_points):
                    update["points"] = np.round((np.column_stack([current_points,np.ones(len(current_points))]) @ transform)[:,:3],8).tolist()
                    old_points = current_points
            if len(update)>1:
                result["frames"][index]["updates"].append(update)
    for index,time in enumerate(times):
        result["frames"][index]["camera"] = {"matrix": np.asarray(caches[index].GetLocalToWorldTransform(camera.GetPrim())).tolist(), "focal_length": float(camera.GetFocalLengthAttr().Get(time)), "horizontal_aperture": float(camera.GetHorizontalApertureAttr().Get(time)), "vertical_aperture": float(camera.GetVerticalApertureAttr().Get(time))}
    result["counts"] = {"objects":len(result["objects"]), "base_vertices":sum(len(obj["points"]) for obj in result["objects"]), "base_faces":sum(len(obj.get("faces",[])) for obj in result["objects"]), "animated_objects":len({u["id"] for frame in result["frames"] for u in frame["updates"]}), "frame_updates":sum(len(frame["updates"]) for frame in result["frames"])}
    output = Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    opener = gzip.open if output.suffix == ".gz" else open
    compression_options = {"compresslevel": 1} if output.suffix == ".gz" else {}
    with opener(output,"wt",encoding="utf-8",**compression_options) as stream:
        json.dump(result,stream,separators=(",",":"))
    summary = {key:result[key] for key in ("source_scene","source_sha256","scope","fps","frame_count","counts","skipped_types")}
    output.with_suffix(".summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--scene",required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--camera",default="/World/hard_demo/camera")
    parser.add_argument("--fps",type=int,default=12)
    parser.add_argument("--frames",type=int,default=96)
    args=parser.parse_args()
    export(args.scene,args.output,args.camera,args.fps,args.frames)
