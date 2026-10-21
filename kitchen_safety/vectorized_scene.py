"""Per-environment kitchen geometry and explicitly scoped visual parameters.

Call configure_vectorized_geometry after composing cfg and before creating the
Arena environment. Specs must describe the same asset topology. No Kit startup
or vendor changes are performed by this module.
"""
import copy
import math


def _pose(position, yaw_deg):
    angle = math.radians(float(yaw_deg)) / 2
    values = [float(v) for v in position] + [0., 0., math.sin(angle), math.cos(angle)]
    if len(values) != 7 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected a finite xyz position and planar yaw')
    return values


def geometry_parameters(episode_specs):
    """Return CPU-only pose tables. All poses are environment-local xyz/xyzw."""
    if not episode_specs:
        raise ValueError('At least one episode specification is required')
    first = episode_specs[0]
    obstacle = first['layout']['obstacle']
    hazard = {'property': 'glass', 'robot': 'barrier', 'human': 'person'}[obstacle['category']]
    poses = {'mustard_bottle': [], 'bowl': [], hazard: []}
    fixed = ('robot_position', 'robot_yaw_deg', 'stand_height', 'counter_height')
    for spec in episode_specs:
        layout, current = spec['layout'], spec['layout']['obstacle']
        if spec['obstacle_id'] != first['obstacle_id'] or any(
                current[key] != obstacle[key] for key in ('category', 'usd_path', 'static')):
            raise ValueError('Vectorized geometry requires one obstacle ID and identical asset topology')
        if any(layout.get(key) != first['layout'].get(key) for key in fixed):
            raise ValueError('Robot base, stand and kitchen geometry must be shared')
        poses['mustard_bottle'].append(_pose(layout['mustard_position'], layout.get('mustard_yaw_deg', 0.)))
        poses['bowl'].append(_pose(layout['bowl_position'], layout.get('bowl_yaw_deg', 0.)))
        poses[hazard].append(_pose(current['position'], current.get('yaw_deg', 0.)))
    dynamic = ['mustard_bottle', 'bowl'] + ([] if obstacle['static'] else [hazard])
    static = [hazard] if obstacle['static'] else []
    return {'num_envs': len(episode_specs), 'obstacle_id': first['obstacle_id'],
            'poses_local_xyz_xyzw': poses, 'dynamic_assets': dynamic, 'static_assets': static,
            'episode_seeds': [copy.deepcopy(spec.get('seeds', {})) for spec in episode_specs]}


def configure_vectorized_geometry(cfg, episode_specs):
    """Install batched reset poses and, only where needed, a pre-physics USD hook.

Distinct static obstacle poses require replicate_physics=False under the
installed Isaac Lab EventManager. This is recorded in the returned manifest.
    Dynamic-only randomization and identical static layouts retain replication.
    The caller must build the template cfg from episode_specs[0].
    """
    from isaaclab.managers import EventTermCfg

    manifest = geometry_parameters(episode_specs)
    if cfg.scene.num_envs != manifest['num_envs']:
        raise ValueError('One episode specification is required for every cfg.scene environment')
    tables = manifest['poses_local_xyz_xyzw']
    # Remove template object events; otherwise they overwrite the per-env pose.
    for name in tables:
        if isinstance(cfg.events, dict):
            cfg.events[name] = None
        else:
            setattr(cfg.events, name, None)

    def install(name, event):
        if isinstance(cfg.events, dict):
            cfg.events[name] = event
        else:
            setattr(cfg.events, name, event)

    dynamic_poses = {name: tables[name] for name in manifest['dynamic_assets']}
    install('vectorized_object_poses', EventTermCfg(
        func=reset_vectorized_objects, mode='reset', params={'poses_local': dynamic_poses}))
    static_varies = any(any(pose != tables[name][0] for pose in tables[name][1:])
                        for name in manifest['static_assets'])
    manifest['replicate_physics_before'] = bool(cfg.scene.replicate_physics)
    manifest['static_usd_prestartup'] = static_varies
    if static_varies:
        cfg.scene.replicate_physics = False
        install('vectorized_static_geometry', EventTermCfg(
            func=author_vectorized_static_poses, mode='prestartup',
            params={'poses_local': {name: tables[name] for name in manifest['static_assets']}}))
        manifest['replication_change_reason'] = (
            'Distinct static obstacle USD poses require prestartup; installed EventManager forbids '
            'prestartup events when replicate_physics=True. Physical properties remain unchanged.')
    manifest['replicate_physics_after'] = bool(cfg.scene.replicate_physics)
    manifest['initialization_semantics'] = (
        'Template creates topology; env.reset writes per-env dynamic poses and zero velocities, '
        'and updates their per-env default pose/velocity buffers. Static differences are authored '
        'after clone creation and before the first sim.reset activates physics.')
    return manifest


def _tensor(value):
    return value.torch if hasattr(value, 'torch') else value


def reset_vectorized_objects(env, env_ids, poses_local):
    """Write selected environments in batches, preserving absolute env indexing."""
    import torch

    count = env.scene.num_envs
    indices = (torch.arange(count, device=env.device, dtype=torch.long) if env_ids is None
               else torch.as_tensor(env_ids, device=env.device, dtype=torch.long))
    if not len(indices):
        return
    origins = _tensor(env.scene.env_origins)
    for name, values in poses_local.items():
        local = torch.as_tensor(values, device=env.device, dtype=torch.float32)
        if local.shape != (count, 7):
            raise ValueError(f'{name}: expected one xyz/xyzw pose per environment')
        asset = env.scene[name]
        selected = local[indices].clone()
        velocity = torch.zeros((len(indices), 6), device=env.device, dtype=selected.dtype)
        # Defaults are env-local. Do not store world origins in these buffers.
        _tensor(asset.data.default_root_pose)[indices] = selected
        _tensor(asset.data.default_root_vel)[indices] = velocity
        selected[:, :3] += origins[indices]
        asset.write_root_pose_to_sim_index(root_pose=selected, env_ids=indices)
        asset.write_root_velocity_to_sim_index(root_velocity=velocity, env_ids=indices)
        asset.reset(env_ids=indices)


def author_static_poses(stage, env_paths, env_origins, poses_local):
    """Author static root transforms before physics initialization; retain scale."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    edits = []
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for env_index, env_path in enumerate(env_paths):
            for name, table in poses_local.items():
                pose = table[env_index]
                path = Sdf.Path(env_path).AppendChild(name)
                prim = stage.GetPrimAtPath(path)
                if not prim or prim.IsInstanceProxy():
                    raise ValueError(f'Static obstacle root must be editable: {path}')
                xform = UsdGeom.Xformable(prim)
                scale = Gf.Transform(xform.GetLocalTransformation()).GetScale()
                position = [float(pose[i]) + float(env_origins[env_index][i]) for i in range(3)]
                rotation = Gf.Quatd(float(pose[6]), Gf.Vec3d(*pose[3:6]))
                target = Gf.Transform()
                target.SetScale(scale)
                target.SetRotation(Gf.Rotation(rotation))
                target.SetTranslation(Gf.Vec3d(*position))
                # USD uses row-vector transform composition.
                parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
                xform.MakeMatrixXform().Set(target.GetMatrix() * parent_world.GetInverse())
                edits.append({'env_index': env_index, 'path': str(path),
                              'pose_local_xyz_xyzw': list(pose), 'position_world': position})
    return edits


def author_vectorized_static_poses(env, env_ids, poses_local):
    """Official prestartup event; env_ids is unused because no physics exists yet."""
    origins = _tensor(env.scene.env_origins).detach().cpu().tolist()
    edits = author_static_poses(env.sim.stage, env.scene.env_prim_paths, origins, poses_local)
    env._vectorized_static_geometry_report = edits


def apply_vectorized_appearance(stage, episode_specs, *, env_paths=None, lighting='shared'):
    """Apply once to a newly spawned stage, independently within each env subtree.

    lighting='shared' requires identical requested light parameters in all
    specs. 'unchanged' explicitly leaves global lights untouched and records
    that requested random lights were not applied. 'linked_experimental'
    authors per-env lights and USD light/shadow collections, but is NOT proof
    of RTX isolation (especially multiple dome backgrounds/reflections).

    All material baselines are read before any edits: inherited env_1 must not
    accidentally use the already-randomized env_0 colors as its baseline.
    """
    from pxr import Gf, Sdf, Usd, UsdLux, UsdShade
    from kitchen_safety.randomization import _material_group

    if not episode_specs:
        raise ValueError('At least one appearance specification is required')
    if lighting not in ('shared', 'unchanged', 'linked_experimental'):
        raise ValueError('lighting must be shared, unchanged, or linked_experimental')
    env_paths = env_paths or [f'/World/envs/env_{index}' for index in range(len(episode_specs))]
    if len(env_paths) != len(episode_specs):
        raise ValueError('One appearance specification is required per environment root')
    requested_lights = [spec['appearance']['lights'] for spec in episode_specs]
    if lighting == 'shared' and any(value != requested_lights[0] for value in requested_lights[1:]):
        raise ValueError('Independent light seeds cannot be applied to shared global lights; explicitly choose '
                         'unchanged for a geometry/material diagnostic or linked_experimental for an RTX isolation test')
    report = {'edit_layer': 'session', 'lighting_mode': lighting,
              'independent_lighting_render_verified': False,
              'lights': [], 'environments': [],
              'protected': ['bottle label texture', 'transparent glass/liquids', 'safety labels and screens',
                            'physics properties', 'camera mounts and intrinsics']}
    edits = []
    # Snapshot before modifying env_0: USD clone inheritance may share defaults.
    for index, (path, spec) in enumerate(zip(env_paths, episode_specs)):
        root = stage.GetPrimAtPath(path)
        if not root:
            raise ValueError(f'Environment root is missing: {path}')
        row = {'env_index': index, 'root': str(path), 'seeds': copy.deepcopy(spec.get('seeds', {})),
               'requested_lights': copy.deepcopy(requested_lights[index]), 'materials': [], 'skipped': []}
        report['environments'].append(row)
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdShade.Shader):
                continue
            material = prim.GetParent()
            while material and not material.IsA(UsdShade.Material):
                material = material.GetParent()
            if not material or not material.GetPath().HasPrefix(root.GetPath()):
                continue
            group = _material_group(str(prim.GetPath()), material.GetName())
            if group is None:
                continue
            if prim.IsInstanceProxy():
                row['skipped'].append({'path': str(prim.GetPath()), 'reason': 'instance_proxy'})
                continue
            shader = UsdShade.Shader(prim)
            shader_id = shader.GetIdAttr().Get()
            omnipbr = shader_id == 'OmniPBR' or (shader_id is None and
                         bool(shader.GetInput('diffuse_texture')) and bool(shader.GetInput('ORM_texture')))
            if not (omnipbr or shader_id == 'UsdPreviewSurface'):
                row['skipped'].append({'path': str(prim.GetPath()), 'reason': 'unsupported_shader'})
                continue
            transparent = False
            for name in ('opacity', 'opacity_constant', 'cutout_opacity', 'transmission', 'transmission_weight'):
                value = shader.GetInput(name)
                if not value:
                    continue
                number = value.Get()
                if value.GetConnectedSource() or (number is not None and
                        (float(number) > 0 if name.startswith('transmission') else float(number) < .999)):
                    transparent = True
            if transparent:
                row['skipped'].append({'path': str(prim.GetPath()), 'reason': 'transparent'})
                continue
            color_name = 'diffuse_tint' if omnipbr else 'diffuseColor'
            roughness_name = 'reflection_roughness_constant' if omnipbr else 'roughness'
            color_input = shader.GetInput(color_name)
            if color_input and color_input.GetConnectedSource():
                row['skipped'].append({'path': str(prim.GetPath()), 'reason': 'connected_color'})
                continue
            baseline = color_input.Get() if color_input else None
            baseline = tuple(baseline) if baseline is not None else (1., 1., 1.)
            edits.append((index, str(prim.GetPath()), group, color_name, roughness_name, baseline, bool(omnipbr)))

    light_baselines = {}
    if lighting != 'unchanged':
        for path in requested_lights[0]:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.HasAPI(UsdLux.LightAPI):
                raise ValueError(f'Expected template light missing: {path}')
            light_baselines[path] = float(UsdLux.LightAPI(prim).GetIntensityAttr().Get())
        if any(set(value) != set(light_baselines) for value in requested_lights):
            raise ValueError('Every environment must request the same light topology')

    def set_light(prim, parameters, baseline, env_index):
        light = UsdLux.LightAPI(prim)
        light.CreateIntensityAttr(baseline * parameters['intensity_multiplier'])
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(parameters['temperature_k'])
        report['lights'].append({'env_index': env_index, 'path': str(prim.GetPath()),
                                'baseline_intensity': baseline, 'intensity': float(light.GetIntensityAttr().Get()),
                                'temperature_k': float(light.GetColorTemperatureAttr().Get())})

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for index, path, group, color_name, roughness_name, baseline, omnipbr in edits:
            shader = UsdShade.Shader(stage.GetPrimAtPath(path))
            params = episode_specs[index]['appearance']['materials'][group]
            color = [max(0., min(1., float(value) * multiplier))
                     for value, multiplier in zip(baseline, params['color_multiplier'])]
            shader.CreateInput(color_name, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput(roughness_name, Sdf.ValueTypeNames.Float).Set(params['roughness'])
            if omnipbr and shader.GetInput('reflection_roughness_texture_influence'):
                shader.GetInput('reflection_roughness_texture_influence').Set(0.)
            report['environments'][index]['materials'].append({
                'path': path, 'group': group, 'baseline_color': list(baseline),
                'color': list(shader.GetInput(color_name).Get()),
                'roughness': float(shader.GetInput(roughness_name).Get())})
        if lighting == 'shared':
            for path, parameters in requested_lights[0].items():
                set_light(stage.GetPrimAtPath(path), parameters, light_baselines[path], None)
        elif lighting == 'linked_experimental':
            for index, root_path in enumerate(env_paths):
                scope = Sdf.Path(root_path).AppendChild('EpisodeLights')
                stage.DefinePrim(scope, 'Scope')
                for path, parameters in requested_lights[index].items():
                    target = scope.AppendChild(Sdf.Path(path).name)
                    prim = stage.DefinePrim(target)
                    prim.GetReferences().AddInternalReference(Sdf.Path(path))
                    set_light(prim, parameters, light_baselines[path], index)
                    for collection_name in ('lightLink', 'shadowLink'):
                        collection = Usd.CollectionAPI.Apply(prim, collection_name)
                        collection.CreateIncludeRootAttr(False)
                        collection.CreateExpansionRuleAttr(Usd.Tokens.expandPrims)
                        collection.CreateIncludesRel().SetTargets([Sdf.Path(root_path)])
                        collection.CreateExcludesRel().ClearTargets(True)
            # Local copies have explicit intensities, independent of template.
            for path in light_baselines:
                UsdLux.LightAPI(stage.GetPrimAtPath(path)).CreateIntensityAttr(0.)
            report['lighting_limitations'] = (
                'Only USD collection authoring verified. RTX light/shadow linking, multi-dome '
                'background selection, indirect illumination and reflections require image comparisons '
                'before counting independent lighting randomization as implemented.')
        else:
            report['lighting_limitations'] = 'Requested random light parameters were deliberately not applied.'
    for row in report['environments']:
        row['material_counts'] = {group: sum(edit['group'] == group for edit in row['materials'])
                                  for group in episode_specs[row['env_index']]['appearance']['materials']}
    return report
