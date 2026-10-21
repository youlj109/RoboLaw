"""Deterministic episode sampling and session-only visual randomization.

No Isaac/Kit imports are needed to sample episodes.  Geometry parameters have
one owner: the sampled layout, shared by the scene, planners and data record.
"""
import copy
import json
import math
from pathlib import Path

import numpy as np

from kitchen_safety.obstacle_library import resolve_layout

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'configs/augmentation_nonhuman10.json'
# Stable catalog indices, independent of the selected batch subset/order.
NONHUMAN_IDS = (
    'property_tall_glass', 'property_wine_glass', 'property_glass_bottle',
    'property_ceramic_mug', 'property_vase', 'property_tablet_stand',
    'robot_steel_clamp', 'robot_projecting_crossbar', 'robot_narrow_opening',
    'robot_sharp_equipment_housing',
)
# Append new IDs: changing the first ten indices would change existing seeds.
HUMAN_IDS = (
    'human_default_mpfb',
    'human_medium_shirt_palm_down', 'human_medium_shirt_raised_wrist',
    'human_compact_sport_palm_down', 'human_compact_sport_raised_wrist',
    'human_broad_workwear_palm_down', 'human_broad_workwear_raised_wrist',
)
SUPPORTED_IDS = NONHUMAN_IDS + HUMAN_IDS


def _number(value, name, low=0.0, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number')
    if not math.isfinite(value) or value < low or (high is not None and value > high):
        raise ValueError(f'{name} outside [{low}, {high}]')
    return float(value)


def validate_randomization_config(config):
    """Validate ordinary JSON parameters without normalizing/mutating them."""
    if not isinstance(config, dict) or config.get('version') != 1:
        raise ValueError('Randomization config must have version 1')
    try:
        for group in ('objects', 'obstacle'):
            _number(config[group]['xy_m'], group + '.xy_m')
            _number(config[group]['yaw_deg'], group + '.yaw_deg', high=180.0)
        for identifier, scale in config.get('range_scale_by_id', {}).items():
            if identifier not in SUPPORTED_IDS:
                raise ValueError(f'Unsupported range override: {identifier}')
            _number(scale, 'range_scale_by_id.' + identifier)
        for group, name, low, high in (
            ('lighting', 'intensity_multiplier', 0.0, None),
            ('lighting', 'temperature_k', 1000.0, 10000.0),
            ('materials', 'color_multiplier', 0.0, None),
            ('materials', 'roughness', 0.0, 1.0),
        ):
            interval = config[group][name]
            if not isinstance(interval, (list, tuple)) or len(interval) != 2:
                raise ValueError(f'{group}.{name} must contain two numbers')
            a, b = [_number(v, f'{group}.{name}', low, high) for v in interval]
            if a > b:
                raise ValueError(f'{group}.{name} must be increasing')
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError(f'Malformed randomization config: {exc}') from exc
    return config


def load_randomization_config(path=DEFAULT_CONFIG):
    return validate_randomization_config(json.loads(Path(path).read_text()))


def sample_episode(obstacle_id, config, seed, attempt):
    """Sample once; retry index is zero-based and does not depend on successes.

    Positions retain their authored support Z and only receive planar yaw.
    This function samples parameters; the collector still validates actual
    support, initial intersections and risk relevance in the spawned scene.
    """
    validate_randomization_config(config)
    if obstacle_id not in SUPPORTED_IDS:
        raise ValueError(f'Unsupported augmentation obstacle: {obstacle_id}')
    for name, value in (('seed', seed), ('attempt', attempt)):
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or not 0 <= value < 2**32:
            raise ValueError(f'{name} must be an integer in [0, 2**32)')
    obstacle_index = SUPPORTED_IDS.index(obstacle_id)
    seeds = {
        stream: int(np.random.SeedSequence([int(seed), obstacle_index, int(attempt), i]).generate_state(1)[0])
        for i, stream in enumerate(('geometry', 'appearance', 'planner'))
    }
    geometry_rng = np.random.default_rng(seeds['geometry'])
    appearance_rng = np.random.default_rng(seeds['appearance'])
    layout = copy.deepcopy(resolve_layout(obstacle_id))
    scale = config.get('range_scale_by_id', {}).get(obstacle_id, 1.0)
    objects = config['objects']
    for name in ('mustard', 'bowl'):
        position = layout[name + '_position']
        position[:2] = (np.asarray(position[:2]) + geometry_rng.uniform(-objects['xy_m'] * scale,
                         objects['xy_m'] * scale, 2)).tolist()
        layout[name + '_yaw_deg'] = float(geometry_rng.uniform(-objects['yaw_deg'] * scale,
                                                              objects['yaw_deg'] * scale))
    obstacle = layout['obstacle']
    obstacle['position'][:2] = (np.asarray(obstacle['position'][:2]) + geometry_rng.uniform(
        -config['obstacle']['xy_m'] * scale, config['obstacle']['xy_m'] * scale, 2)).tolist()
    obstacle['yaw_deg'] = float(obstacle.get('yaw_deg', 0.0) + geometry_rng.uniform(
        -config['obstacle']['yaw_deg'] * scale, config['obstacle']['yaw_deg'] * scale))
    if obstacle['category'] == 'property':
        layout['property'].update(glass_position=list(obstacle['position']), glass_yaw_deg=obstacle['yaw_deg'])
    elif obstacle['category'] == 'human':
        layout['human'].update(position=list(obstacle['position']), yaw_deg=obstacle['yaw_deg'])
        # The catalog's world keypoints describe its authored placement. Keep
        # local anatomy fixed and report world keypoints at this sampled pose.
        yaw = math.radians(obstacle['yaw_deg'])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        for name, point in list(obstacle.get('keypoints', {}).items()):
            if not name.endswith('_local'):
                continue
            x, y, z = point
            obstacle['keypoints'][name[:-6] + '_world'] = [
                obstacle['position'][0] + cosine * x - sine * y,
                obstacle['position'][1] + sine * x + cosine * y,
                obstacle['position'][2] + z,
            ]
    else:
        layout['robot'].update(barrier_position=list(obstacle['position']), barrier_yaw_deg=obstacle['yaw_deg'])
    appearance = {'lights': {}, 'materials': {}}
    for path in ('/World/Light', '/World/DirectionalLight'):
        appearance['lights'][path] = {
            'intensity_multiplier': float(appearance_rng.uniform(*config['lighting']['intensity_multiplier'])),
            'temperature_k': float(appearance_rng.uniform(*config['lighting']['temperature_k'])),
        }
    for group in ('kitchen_wall', 'kitchen_counter', 'kitchen_cabinet', 'bowl', 'obstacle'):
        appearance['materials'][group] = {
            'color_multiplier': appearance_rng.uniform(*config['materials']['color_multiplier'], 3).tolist(),
            'roughness': float(appearance_rng.uniform(*config['materials']['roughness'])),
        }
    return {'version': 1, 'obstacle_id': obstacle_id, 'master_seed': int(seed),
            'attempt': int(attempt), 'seeds': seeds, 'range_scale': scale,
            'layout': layout, 'appearance': appearance}


def _material_group(path, material_name):
    """Allow only known surface classes; never tint the bottle label atlas."""
    parts = path.lower().split('/')
    name = material_name.lower()
    if 'kitchen' in parts:
        if name.startswith('wall'):
            return 'kitchen_wall'
        if name.startswith('counter') and name.endswith('_top'):
            return 'kitchen_counter'
        if ((name.startswith(('cab_', 'stack_')) and 'main_group' in name)
                or name.startswith('m_cabinetdoorpanel')):
            return 'kitchen_cabinet'
        return None
    if 'bowl' in parts:
        return 'bowl'
    if 'glass' in parts or 'barrier' in parts:
        if any(word in name for word in ('label', 'glass', 'crystal', 'water', 'wine',
                                         'safety', 'warning', 'screen', 'display',
                                         'control', 'button')):
            return None
        return 'obstacle'
    return None


def apply_appearance(stage, spec, *, edit_layer=None):
    """Apply before first RGB, using the USD session layer by default.

    Returns actual authored attribute values, not just intended parameters.
    Texture/opacity/metallic/physics properties and all camera attributes are
    left untouched.  The mustard's shared bottle/label texture is preserved.
    By default the caller should invoke once per newly spawned scene. A
    persistent renderer can supply its own layer in the stage's layer stack;
    clear only that layer before each episode so multipliers are applied to
    the original composed values. An outer EditContext alone is insufficient
    because this function explicitly selects its edit target.
    """
    from pxr import Gf, Sdf, Usd, UsdLux, UsdShade

    target_layer = stage.GetSessionLayer() if edit_layer is None else edit_layer
    report = {'edit_layer': 'session' if edit_layer is None else 'caller_supplied',
              'lights': [], 'materials': [], 'skipped': [],
              'protected': ['mustard shared label texture', 'transparent glass and liquids',
                            'labels, displays, control panels and safety markings']}
    appearance = spec['appearance']
    with Usd.EditContext(stage, target_layer):
        for path, parameters in appearance['lights'].items():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.HasAPI(UsdLux.LightAPI):
                raise ValueError(f'Expected episode light missing: {path}')
            light = UsdLux.LightAPI(prim)
            baseline = float(light.GetIntensityAttr().Get())
            light.CreateIntensityAttr(baseline * parameters['intensity_multiplier'])
            light.CreateEnableColorTemperatureAttr(True)
            light.CreateColorTemperatureAttr(parameters['temperature_k'])
            report['lights'].append({'path': path, 'baseline_intensity': baseline,
                'intensity_multiplier': parameters['intensity_multiplier'],
                'intensity': float(light.GetIntensityAttr().Get()),
                'temperature_k': float(light.GetColorTemperatureAttr().Get()),
                'enable_color_temperature': bool(light.GetEnableColorTemperatureAttr().Get())})
        for prim in stage.Traverse():
            if not prim.IsA(UsdShade.Shader):
                continue
            parent = prim.GetParent()
            while parent and not parent.IsA(UsdShade.Material):
                parent = parent.GetParent()
            if not parent:
                continue
            group = _material_group(str(prim.GetPath()), parent.GetName())
            if group is None:
                continue
            shader = UsdShade.Shader(prim)
            parameters = appearance['materials'][group]
            shader_id = shader.GetIdAttr().Get()
            # Some kitchen MDL shaders use sourceAsset rather than info:id.
            is_omnipbr = shader_id == 'OmniPBR' or (shader_id is None and
                bool(shader.GetInput('diffuse_texture')) and bool(shader.GetInput('ORM_texture')))
            is_preview = shader_id == 'UsdPreviewSurface'
            if not (is_omnipbr or is_preview):
                report['skipped'].append({'path': str(prim.GetPath()), 'reason': 'unsupported_shader'})
                continue
            transparent = False
            for name in ('opacity', 'opacity_constant', 'cutout_opacity'):
                value = shader.GetInput(name)
                if value and (value.GetConnectedSource() or (value.Get() is not None and float(value.Get()) < 0.999)):
                    transparent = True
            for name in ('transmission', 'transmission_weight'):
                value = shader.GetInput(name)
                if value and (value.GetConnectedSource() or (value.Get() is not None and float(value.Get()) > 0.0)):
                    transparent = True
            if transparent:
                report['skipped'].append({'path': str(prim.GetPath()), 'reason': 'transparent'})
                continue
            color_name = 'diffuse_tint' if is_omnipbr else 'diffuseColor'
            roughness_name = 'reflection_roughness_constant' if is_omnipbr else 'roughness'
            color_input = shader.GetInput(color_name)
            if color_input and color_input.GetConnectedSource():
                report['skipped'].append({'path': str(prim.GetPath()), 'reason': 'connected_color'})
                continue
            baseline_color = color_input.Get() if color_input else None
            if baseline_color is None:
                baseline_color = (1.0, 1.0, 1.0)
            color = [max(0.0, min(1.0, float(c) * m))
                     for c, m in zip(baseline_color, parameters['color_multiplier'])]
            shader.CreateInput(color_name, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput(roughness_name, Sdf.ValueTypeNames.Float).Set(parameters['roughness'])
            # Kitchen door ORM maps otherwise override the visual roughness constant.
            if is_omnipbr and shader.GetInput('reflection_roughness_texture_influence'):
                shader.GetInput('reflection_roughness_texture_influence').Set(0.0)
            report['materials'].append({'path': str(prim.GetPath()), 'group': group,
                'color_input': color_name, 'baseline_color': [float(v) for v in baseline_color],
                'color': [float(v) for v in shader.GetInput(color_name).Get()],
                'roughness_input': roughness_name,
                'roughness': float(shader.GetInput(roughness_name).Get())})
    report['material_counts'] = {group: sum(m['group'] == group for m in report['materials'])
                                 for group in appearance['materials']}
    return report
