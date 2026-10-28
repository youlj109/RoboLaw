"""Versioned clear-water shading for the hard demo's visual-only stream."""
from __future__ import annotations

import json
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdShade

from .water_mechanism import CONFIG, DIRECTORY, ROOT, sha


def build(config_path=CONFIG):
    """Keep the original stream geometry and physics; replace its blue shader."""
    config_path = Path(config_path)
    config = json.loads(config_path.read_text())
    original = ROOT / config["visual_asset"]
    output = DIRECTORY / "clear_water_v2" / "water_visual.usda"
    if output.exists():
        raise FileExistsError(f"Preserve existing appearance revision: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [os.path.relpath(original, output.parent)]
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Asset"))

    material = UsdShade.Material.Define(stage, "/Asset/Looks/ClearWater")
    shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/Shader")
    shader.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateVolumeOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    inputs = (
        ("glass_color", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1.)),
        ("glass_ior", Sdf.ValueTypeNames.Float, 1.333),
        ("frosting_roughness", Sdf.ValueTypeNames.Float, .02),
        ("reflection_color", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1.)),
        ("thin_walled", Sdf.ValueTypeNames.Bool, False),
        ("depth", Sdf.ValueTypeNames.Float, .001),
        ("enable_opacity", Sdf.ValueTypeNames.Bool, False),
        ("cutout_opacity", Sdf.ValueTypeNames.Float, 1.),
    )
    for name, kind, value in inputs:
        shader.CreateInput(name, kind).Set(value)
    for name in ("Stream", "ContactWater"):
        prim = stage.OverridePrim("/Asset/FlowVisual/" + name)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    stage.GetRootLayer().Save()

    config["visual_asset"] = str(output.relative_to(ROOT))
    config["asset_sha256"][config["visual_asset"]] = sha(output)
    config["water_appearance"] = dict(
        revision="clear_water_v2", shader="OmniGlass.mdl::OmniGlass",
        ior=1.333, roughness=.02, tint_rgb=[1., 1., 1.],
        transmission="Physical dielectric refraction; no blue diffuse/metallic coating",
        stream_geometry_unchanged=True, visual_only=True,
        parent_asset=str(original.relative_to(ROOT)), parent_sha256=sha(original),
        renderer_settings={"/rtx/translucency/enabled": True, "/rtx/reflections/enabled": True},
    )
    config_path.write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n")
    return config["water_appearance"]


if __name__ == "__main__":
    print(json.dumps(build(), indent=2, ensure_ascii=False))
