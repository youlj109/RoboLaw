"""Traceable local repair for the two passive toaster doneness knobs.

The imported mesh has a slightly eccentric automatically computed mass centre
and an X drive on a revolute joint.  This adapter models a balanced passive knob
and authors a real angular damper.  It never sets runtime joint state, adds a
position servo, changes collision geometry, or changes the cancel-button task.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

RECIPE_VERSION = "balanced-passive-knobs-v2"
ANGULAR_DAMPING_NM_S_PER_RAD = 0.05


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def repaired_kitchen(source: Path, output_dir: Path) -> tuple[Path, dict]:
    """Write a small local USD override and its provenance, retaining the source.

    The knob mass centre is the collision-volume centroid projected onto the
    existing joint axis.  Only its perpendicular offset changes; mass/inertia
    remain subject to the original USD's mass and PhysX inertia computation.
    This is an idealized balanced knob, not a measured manufacturer calibration.
    """
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    source = Path(source).resolve()
    source_hash = _hash(source)
    recipe_hash = _hash(__file__)
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"toaster_{RECIPE_VERSION}_{source_hash[:16]}_{recipe_hash[:12]}.usda"
    manifest = output.with_suffix(".json")
    if output.exists() and manifest.exists():
        info = json.loads(manifest.read_text())
        if info["overlay_sha256"] == _hash(output):
            return output, info
        raise ValueError(f"Changed cached physics override: {output}")

    # The override uses a sublayer so relative texture references remain anchored
    # to the original asset. The generated absolute source path is never shipped.
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath("/world"))
    info = {"recipe": RECIPE_VERSION, "source": str(source), "source_sha256": source_hash,
            "recipe_sha256": recipe_hash,
            "model": "Balanced passive knob with viscous angular damping; no positional servo",
            "angular_damping_nm_s_per_rad": ANGULAR_DAMPING_NM_S_PER_RAD,
            "usd_angular_damping_nm_s_per_deg": ANGULAR_DAMPING_NM_S_PER_RAD * math.pi / 180.,
            "angular_stiffness_nm_per_rad": 0.0, "knobs": []}
    xf = UsdGeom.XformCache()
    for side in ("L", "R"):
        path = f"/world/toaster_main_group/Toaster015_side{side}_knob_doneness"
        body = stage.GetPrimAtPath(path)
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{path}/side{side}_knob_doneness_joint"))
        if not body or not joint:
            raise ValueError(f"Missing source toaster knob {side}")
        anchor = np.array(joint.GetLocalPos1Attr().Get(), dtype=float)
        rotation = joint.GetLocalRot1Attr().Get()
        axis = np.array(rotation.Transform(Gf.Vec3f(1, 0, 0)), dtype=float)
        axis /= np.linalg.norm(axis)
        body_inverse = xf.GetLocalToWorldTransform(body).GetInverse()
        volumes, centres = [], []
        for prim in Usd.PrimRange(body):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                continue
            transform = xf.GetLocalToWorldTransform(prim) * body_inverse
            scale_volume = abs(transform.GetDeterminant())
            if prim.IsA(UsdGeom.Cube):
                volume = UsdGeom.Cube(prim).GetSizeAttr().Get() ** 3 * scale_volume
            elif prim.IsA(UsdGeom.Cylinder):
                shape = UsdGeom.Cylinder(prim)
                volume = math.pi * shape.GetRadiusAttr().Get() ** 2 * shape.GetHeightAttr().Get() * scale_volume
            else:
                raise ValueError(f"Unsupported knob mass geometry: {prim.GetPath()}")
            volumes.append(volume)
            centres.append(np.array(transform.Transform(Gf.Vec3d(0)), dtype=float))
        centroid = np.average(centres, weights=volumes, axis=0)
        balanced = anchor + axis * np.dot(centroid - anchor, axis)
        mass_api = UsdPhysics.MassAPI.Apply(body)
        source_com_attribute = str(mass_api.GetCenterOfMassAttr().Get())
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*balanced))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        # USD angular drives use degrees; the runtime PhysX tensor uses radians.
        # See pinned IsaacLab sim/schemas/schemas.py: set_joint_drive_properties.
        drive.CreateDampingAttr(ANGULAR_DAMPING_NM_S_PER_RAD * math.pi / 180.)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateMaxForceAttr(10000.0)  # same cap as the original X drive
        info["knobs"].append({"joint": str(joint.GetPath()), "body": path,
                              "source_collision_centroid_local_m": centroid.tolist(),
                              "joint_anchor_body_local_m": anchor.tolist(),
                              "joint_axis_body_local": axis.tolist(),
                              "body_local_to_world_row_vector_matrix": np.asarray(xf.GetLocalToWorldTransform(body)).tolist(),
                              "source_com_attribute": source_com_attribute,
                              "balanced_com_local_m": balanced.tolist(),
                              "com_perpendicular_correction_m": float(np.linalg.norm(balanced-centroid)),
                              "source_mass_kg": mass_api.GetMassAttr().Get(),
                              "joint_limits_degrees": [joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()]})
    stage.GetRootLayer().Save()
    info["overlay_sha256"] = _hash(output)
    manifest.write_text(json.dumps(info, indent=2) + "\n")
    return output, info
