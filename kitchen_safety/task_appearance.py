"""Exact-path visual augmentation of an offline reconstruction stage.

Only supported light and shader inputs are writable. No geometry, physical
material, opacity, texture connection, camera, or safety attribute is changed.
Recipes and actual readbacks belong to the derived render, never the raw run.
"""
from copy import deepcopy
import hashlib
import math
import platform
import random

from kitchen_safety.task_contract import digest


SHADER_INPUTS = {
    'UsdPreviewSurface': {'base_color_rgb': 'diffuseColor', 'roughness': 'roughness'},
    'OmniPBR': {'base_color_rgb': 'diffuse_color_constant', 'roughness': 'reflection_roughness_constant'},
}


def shader_kind(shader):
    identifier = shader.GetIdAttr().Get()
    if identifier in SHADER_INPUTS:
        return identifier
    source = shader.GetPrim().GetAttribute('info:mdl:sourceAsset')
    asset = source.Get() if source else None
    if asset and str(asset.path).replace('\\', '/').rsplit('/', 1)[-1] == 'OmniPBR.mdl':
        return 'OmniPBR'
    return None


def plain(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    return [float(v) for v in value]


def usable_attribute(attr):
    return bool(attr) and attr.Get() is not None and not attr.GetConnections() and not attr.GetNumTimeSamples()


def visible_material_bindings(stage):
    from pxr import Usd, UsdGeom, UsdShade
    result = {}
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputeVisibility() == 'invisible' or imageable.ComputePurpose() == 'guide':
            continue
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if material:
            result.setdefault(str(material.GetPath()), []).append(str(prim.GetPath()))
    return result


def active_material(shader):
    """Resolve the surface used by the native RTX renderer, preferring MDL."""
    from pxr import UsdShade
    parent = shader.GetPrim().GetParent()
    while parent and not parent.IsA(UsdShade.Material):
        parent = parent.GetParent()
    if not parent:
        return None
    material = UsdShade.Material(parent)
    mdl = material.GetSurfaceOutput('mdl')
    context = 'mdl' if mdl and mdl.HasConnectedSource() else ''
    source, _, _ = material.ComputeSurfaceSource(context)
    if not source or source.GetPath() != shader.GetPath():
        return None
    return str(material.GetPath())


def directly_effective_input(shader, semantic):
    # A texture can override an OmniPBR constant. Do not silently disconnect
    # maps or claim that changing an unused constant changes the rendered look.
    if shader_kind(shader) != 'OmniPBR':
        return True
    names = ['diffuse_texture'] if semantic == 'base_color_rgb' else ['reflection_roughness_texture', 'ORM_texture']
    if semantic == 'roughness':
        influence = shader.GetInput('reflection_roughness_texture_influence')
        if influence and influence.Get() == 0:
            return True
    for name in names:
        item = shader.GetInput(name)
        if item and (item.GetConnectedSource() or item.Get()):
            return False
    return True


def catalog(stage):
    """List exact available inputs; discovery never authorizes changing them."""
    from pxr import UsdLux, UsdShade
    result = dict(lights=[], shaders=[], unsupported_shaders=0)
    bindings = visible_material_bindings(stage)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdLux.LightAPI):
            light = UsdLux.LightAPI(prim)
            attr = light.GetIntensityAttr()
            result['lights'].append(dict(path=str(prim.GetPath()), intensity=plain(attr.Get()),
                intensity_writable=usable_attribute(attr)))
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim); kind = shader_kind(shader)
        if kind is None:
            result['unsupported_shaders'] += 1
            continue
        inputs = {}; material = active_material(shader)
        for semantic, name in SHADER_INPUTS[kind].items():
            item = shader.GetInput(name); attr = item.GetAttr() if item else None
            inputs[semantic] = dict(input=name, value=plain(attr.Get()) if attr else None,
                writable=usable_attribute(attr) and directly_effective_input(shader, semantic)
                    and bool(bindings.get(material)))
        result['shaders'].append(dict(path=str(prim.GetPath()), shader_kind=kind, inputs=inputs,
            active_material=material, affected_visible_geometry=bindings.get(material, [])))
    return result


def _interval(value, low, high=None):
    if not isinstance(value, list) or len(value) != 2 or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError('Expected a finite uniform [min, max] interval')
    if value[0] < low or value[0] > value[1] or (high is not None and value[1] > high):
        raise ValueError('Appearance interval is outside its declared domain')
    return value


def apply_offline_appearance(stage, contract, config, seed):
    from pxr import Gf, Usd, UsdLux, UsdShade
    if set(config) - {'schema_version', 'task_sha256', 'semantics', 'lights', 'shaders'}:
        raise ValueError('Unknown appearance fields; physical parameters are unsupported')
    if config.get('schema_version') != 'robolaw-native-appearance-1' or config.get('semantics') != 'visual_only':
        raise ValueError('Offline appearance requires an explicit visual-only recipe')
    if config.get('task_sha256') != digest(contract):
        raise ValueError('Appearance recipe belongs to another task')
    if type(seed) is not int or seed < 0:
        raise ValueError('Appearance requires an explicit nonnegative integer seed')
    lights, shaders = config.get('lights', []), config.get('shaders', [])
    if not isinstance(lights, list) or not isinstance(shaders, list) or not (lights or shaders):
        raise ValueError('Appearance recipe must select nonempty exact paths')
    layer = stage.GetSessionLayer()
    if 'robolawOfflineAppearance' in layer.customLayerData:
        raise ValueError('Appearance already applied; open a fresh reconstruction stage')
    edits = []; seen = set(); selected_paths = set(); bindings = visible_material_bindings(stage)

    def draw(path, semantic, interval, low, high=None):
        bounds = _interval(interval, low, high)
        stream = hashlib.sha256(f'{seed}:{path}:{semantic}'.encode()).digest()
        return random.Random(int.from_bytes(stream, 'big')).uniform(*bounds)

    def add(attr, value, semantic, path, allow_schema_default=False):
        if not attr or attr.GetConnections() or attr.GetNumTimeSamples() or (attr.Get() is None and not allow_schema_default):
            raise ValueError('Input absent, connected, or animated: ' + str(attr.GetPath() if attr else path))
        key = str(attr.GetPath())
        if key in seen:
            raise ValueError('Duplicate appearance attribute: ' + key)
        seen.add(key)
        edits.append((attr, value, dict(path=path, attribute=key, semantic=semantic,
            baseline=plain(attr.Get()), requested=plain(value))))

    def select(row, allowed):
        if not isinstance(row, dict) or set(row) - allowed or 'path' not in row:
            raise ValueError('Malformed exact-path appearance target')
        path = row['path']
        if not isinstance(path, str) or not path.startswith('/') or any(x in path for x in '*{}') or path in selected_paths:
            raise ValueError('Appearance paths must be unique exact absolute USD prim paths')
        selected_paths.add(path)
        prim = stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError('Appearance target absent from this reconstruction: ' + path)
        return path, prim

    for row in lights:
        path, prim = select(row, {'path', 'intensity_multiplier', 'temperature_k'})
        if not prim.HasAPI(UsdLux.LightAPI) or len(row) == 1:
            raise ValueError('Light target must declare supported changes')
        light = UsdLux.LightAPI(prim)
        if 'intensity_multiplier' in row:
            attr = light.GetIntensityAttr(); baseline = attr.Get()
            if baseline is None or not math.isfinite(float(baseline)) or baseline < 0:
                raise ValueError('Invalid original light intensity')
            value = float(baseline) * draw(path, 'intensity_multiplier', row['intensity_multiplier'], 0)
            add(attr, value, 'intensity', path)
        if 'temperature_k' in row:
            value = draw(path, 'temperature_k', row['temperature_k'], 1000, 10000)
            add(light.GetColorTemperatureAttr(), value, 'temperature_k', path, True)
            add(light.GetEnableColorTemperatureAttr(), True, 'temperature_enabled', path, True)
    for row in shaders:
        path, prim = select(row, {'path', 'base_color_rgb', 'roughness'})
        if not prim.IsA(UsdShade.Shader) or len(row) == 1:
            raise ValueError('Shader target must declare supported changes')
        shader = UsdShade.Shader(prim); kind = shader_kind(shader)
        if kind not in SHADER_INPUTS:
            raise ValueError('Unsupported shader implementation: ' + path)
        material = active_material(shader)
        if not bindings.get(material):
            raise ValueError('Shader is not the active surface of visible bound geometry: ' + path)
        for semantic in ('roughness', 'base_color_rgb'):
            if semantic not in row:
                continue
            if not directly_effective_input(shader, semantic):
                raise ValueError('A texture overrides this shader constant: ' + path + ':' + semantic)
            item = shader.GetInput(SHADER_INPUTS[kind][semantic])
            if not item:
                raise ValueError('Shader input missing: ' + path + ':' + semantic)
            if semantic == 'roughness':
                value = draw(path, semantic, row[semantic], 0, 1)
            else:
                if not isinstance(row[semantic], list) or len(row[semantic]) != 3:
                    raise ValueError('RGB must provide three explicit uniform intervals')
                value = Gf.Vec3f(*(draw(path, semantic + str(i), bounds, 0, 1) for i, bounds in enumerate(row[semantic])))
            add(item.GetAttr(), value, semantic, path)
            edits[-1][2].update(active_material=material, affected_visible_geometry=bindings[material])
    # Validate all targets before writing anything. The root layer is untouched.
    report = dict(schema_version='robolaw-native-appearance-report-1', semantics='visual_only',
        seed=seed, generator='python.random.Random/MT19937 with SHA256 per-attribute streams',
        python_version=platform.python_version(), config=deepcopy(config), config_sha256=digest(config),
        changes=[], physical_material_randomization=False, source_online_observations=False,
        geometry_or_camera_changes=False)
    with Usd.EditContext(stage, layer):
        for attr, value, record in edits:
            if not attr.Set(value):
                raise ValueError('USD appearance write failed: ' + record['attribute'])
            record['actual'] = plain(attr.Get())
            report['changes'].append(record)
    layer.customLayerData = dict(layer.customLayerData, robolawOfflineAppearance=report['config_sha256'])
    semantics = {r['semantic'] for r in report['changes']}
    report['dimensions_applied'] = dict(lighting=bool(semantics & {'intensity', 'temperature_k'}),
        roughness='roughness' in semantics, visual_material_color='base_color_rgb' in semantics)
    report['appearance_sha256'] = digest(report)
    return report
