"""Export actual visible composed USD geometry in world coordinates for CPU preview.

This deliberately avoids Blender's USD importer. No source geometry is moved,
hidden, enlarged, or substituted. Materials are approximate preview materials;
MDL defaults without accessible colors receive an explicitly reported fallback.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path

# Keep the CPU-only preview preparation bounded on the shared workstation.
for _thread_option in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_thread_option] = "4"
import numpy as np
from pxr import Usd, UsdGeom, UsdShade


def material(prim):
    result = {"color": [.64, .66, .68], "roughness": .55, "metallic": 0., "opacity": 1., "source": "neutral fallback"}
    display = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get() if prim.IsA(UsdGeom.Gprim) else None
    if display:
        result.update(color=list(display[0]), source="authored displayColor")
    bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not bound:
        return result
    result["material_path"] = str(bound.GetPath())
    # Blender USD exports base-color textures through a connected UVTexture,
    # rather than the MDL diffuse_texture input used by the kitchen assets.
    for shader in Usd.PrimRange(bound.GetPrim()):
        color_input = shader.GetAttribute('inputs:diffuseColor')
        if not color_input:
            continue
        for connection in color_input.GetConnections():
            texture_prim = prim.GetStage().GetPrimAtPath(connection.GetPrimPath())
            file_attr = texture_prim.GetAttribute('inputs:file') if texture_prim else None
            texture = file_attr.Get() if file_attr else None
            if texture and texture.resolvedPath and Path(texture.resolvedPath).is_file():
                result['texture'] = texture.resolvedPath
                result['source'] = 'authored USD PreviewSurface diffuse texture'
                try:
                    from PIL import Image
                    im = Image.open(texture.resolvedPath).convert('RGB').resize((1, 1))
                    result['texture_average'] = [v / 255. for v in im.getpixel((0, 0))]
                except (ImportError, OSError):
                    pass
    for shader in Usd.PrimRange(bound.GetPrim()):
        for key in ("diffuseColor", "diffuse_color_constant", "diffuse_tint", "base_color", "albedo"):
            value = shader.GetAttribute("inputs:" + key).Get()
            if value is not None and hasattr(value, "__len__") and len(value) >= 3:
                result.update(color=list(value)[:3], source="authored material color")
                break
        for key, names in {"roughness": ("roughness", "reflection_roughness_constant"), "metallic": ("metallic", "metallic_constant"), "opacity": ("opacity",), "ior": ("ior",)}.items():
            for name in names:
                value = shader.GetAttribute("inputs:" + name).Get()
                if value is not None:
                    try:
                        result[key] = float(value)
                    except (ValueError, TypeError):
                        pass
                    break
        for name in ("diffuse_texture", "base_color_texture"):
            texture = shader.GetAttribute("inputs:" + name).Get()
            if texture is not None and texture.resolvedPath and Path(texture.resolvedPath).is_file():
                result["texture"] = texture.resolvedPath
                result["source"] = "authored texture (UV when available, mean color fallback)"
                try:
                    from PIL import Image
                    im = Image.open(texture.resolvedPath).convert("RGB").resize((1, 1))
                    result["texture_average"] = [channel / 255. for channel in im.getpixel((0, 0))]
                except (ImportError, OSError):
                    pass
                break
    return result


def mesh_data(prim):
    mesh = UsdGeom.Mesh(prim)
    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
    counts, indices = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
    holes = set(mesh.GetHoleIndicesAttr().Get() or [])
    faces, face_loop_indices, offset = [], [], 0
    for face_index, count in enumerate(counts):
        if face_index not in holes:
            face = list(indices[offset:offset + count])
            loops = list(range(offset, offset + count))
            if mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded:
                face.reverse()
                loops.reverse()
            faces.append(face)
            face_loop_indices.extend(loops)
        offset += count
    extra = {}
    for name in ("st", "st0", "UVMap", "uv"):
        uv = UsdGeom.PrimvarsAPI(prim).GetPrimvar(name)
        if uv and uv.HasValue():
            values = uv.ComputeFlattened()
            if values is not None:
                interpolation = uv.GetInterpolation()
                if interpolation == UsdGeom.Tokens.faceVarying:
                    extra["loop_uv"] = [list(values[index])[:2] for index in face_loop_indices]
                elif interpolation in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
                    extra["loop_uv"] = [list(values[index])[:2] for face in faces for index in face]
            break
    return points, faces, extra


def primitive_data(prim, sides=32):
    kind = prim.GetTypeName()
    if kind == "Cube":
        half = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2.
        points = [(x * half, y * half, z * half) for x, y, z in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        return np.asarray(points), [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]], {}
    if kind in ("Cylinder", "Cone"):
        schema = UsdGeom.Cylinder(prim) if kind == "Cylinder" else UsdGeom.Cone(prim)
        radius, height, axis = schema.GetRadiusAttr().Get(), schema.GetHeightAttr().Get(), schema.GetAxisAttr().Get()
        points = []
        for z in (-height / 2., height / 2.):
            r = radius if kind == "Cylinder" or z < 0 else 0.
            for i in range(sides):
                p = [r * math.cos(2 * math.pi * i / sides), r * math.sin(2 * math.pi * i / sides), z]
                points.append([p[2],p[0],p[1]] if axis == "X" else [p[1],p[2],p[0]] if axis == "Y" else p)
        faces = [list(reversed(range(sides))), list(range(sides, sides * 2))]
        faces.extend([i,(i+1)%sides,(i+1)%sides+sides,i+sides] for i in range(sides))
        return np.asarray(points), faces, {}
    if kind == "Sphere":
        radius = UsdGeom.Sphere(prim).GetRadiusAttr().Get()
        points = [[radius * math.sin(math.pi*j/16) * math.cos(2*math.pi*i/sides), radius * math.sin(math.pi*j/16) * math.sin(2*math.pi*i/sides), radius * math.cos(math.pi*j/16)] for j in range(17) for i in range(sides)]
        faces = [[j*sides+i,j*sides+(i+1)%sides,(j+1)*sides+(i+1)%sides,(j+1)*sides+i] for j in range(16) for i in range(sides)]
        return np.asarray(points), faces, {}
    return None


def export(scene_path, output):
    stage = Usd.Stage.Open(str(Path(scene_path).resolve()))
    if not stage or stage.GetCompositionErrors():
        raise ValueError("Scene composition must be complete before preview export")
    cache = UsdGeom.XformCache()
    result = {"source_scene": str(Path(scene_path).resolve()), "source_sha256": hashlib.sha256(Path(scene_path).read_bytes()).hexdigest(), "scope": "Actual visible USD geometry; CPU preview shading approximation, not physics or RTX acceptance", "objects": [], "skipped_types": {}}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Imageable):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible or imageable.ComputePurpose() in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy):
            continue
        transform = np.asarray(cache.GetLocalToWorldTransform(prim))
        if prim.IsA(UsdGeom.BasisCurves):
            curve = UsdGeom.BasisCurves(prim)
            points = np.asarray(curve.GetPointsAttr().Get(), dtype=float)
            world_points = np.column_stack([points, np.ones(len(points))]) @ transform
            result["objects"].append({"path": str(prim.GetPath()), "kind": "curve", "points": world_points[:,:3].tolist(), "counts": list(curve.GetCurveVertexCountsAttr().Get()), "widths": list(curve.GetWidthsAttr().Get() or [.001]), "periodic": curve.GetWrapAttr().Get() == UsdGeom.Tokens.periodic, "material": material(prim)})
            continue
        data = mesh_data(prim) if prim.IsA(UsdGeom.Mesh) else primitive_data(prim)
        if data is None:
            if prim.IsA(UsdGeom.Gprim):
                kind = prim.GetTypeName()
                result["skipped_types"][kind] = result["skipped_types"].get(kind, 0) + 1
            continue
        points, faces, extra = data
        world_points = np.column_stack([points, np.ones(len(points))]) @ transform
        if np.linalg.det(transform[:3,:3]) < 0:
            # Keep the authored winding under reflected USD transforms.
            faces = [list(reversed(face)) for face in faces]
            if "loop_uv" in extra:
                old_uv, extra["loop_uv"], offset = extra["loop_uv"], [], 0
                for face in faces:
                    extra["loop_uv"].extend(reversed(old_uv[offset:offset+len(face)]))
                    offset += len(face)
        result["objects"].append({"path": str(prim.GetPath()), "kind": "mesh", "points": np.round(world_points[:,:3], 8).tolist(), "faces": faces, "material": material(prim), **extra})
    result["counts"] = {"objects": len(result["objects"]), "vertices": sum(len(o["points"]) for o in result["objects"]), "faces": sum(len(o.get("faces", [])) for o in result["objects"])}
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    opener = gzip.open if output.suffix == ".gz" else open
    with opener(output, "wt", encoding="utf-8") as stream:
        json.dump(result, stream, separators=(",", ":"))
    print(json.dumps({"output": str(output), "counts": result["counts"], "unsupported_visible_geometry": result["skipped_types"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    export(args.scene, args.output)
