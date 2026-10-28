"""Banana-layout-only 128/8 candidate; no integration or task success implied.

The predeclared 128/8 case passed a 10 s probe; 255/16 failed. This is not a
monotonic convergence result. A new 45 s neutral and actual task execution are
required before accepting this physical profile. The certified shared 64/4
recipe is inherited unchanged and is never modified by this module.
"""
import copy
import hashlib
import json
from pathlib import Path

from kitchen_safety.extensions.replicator import background_physics as base

ROOT = Path(__file__).resolve().parents[3]
RECIPE = 'banana-g-shape-kettle-solver128-8-candidate-v1'
TASK = 'kitchen_bench_replicator_g_shape_banana_bowl'
LAYOUT_PATH = ROOT/'configs/extensions/replicator/g_shape_banana_bowl_source_layout.json'
LAYOUT_SHA256 = '7a032245402428e2ec49305a96370a86301d836685812c6d95fae4704c66dfb1'
SOURCE_SHA256 = '3c502506532233850196fabe24f03dc488245de72951ce069e1e84f536a9fa01'
SOLVER_ATTRIBUTES = {'physxRigidBody:solverPositionIterationCount': 128,
                     'physxRigidBody:solverVelocityIterationCount': 8}
DIAGNOSTICS = (
    (128, 8, True, '90546cd1a286b174b09673469f25638bdf821b26bec56ee4d83af6df3a805cb3',
     '29a505003a64be13ad0b656322f1664175dfd3d7a869344235243b6ed35675f0'),
    (255, 16, False, 'e775986afdee26695d7e81d01d2414ed4f8a8a31cc586dfaae8ad9c7652ee17c',
     '31c033311e02e6f039d424fae8ec6709d611d78e550decfa17f1c5498ff1fc8d'),
)
MATRIX_PATH = 'reports/replicator_execution_20261023/g_banana/banana_solver_matrix_independent01.json'
MATRIX_SHA256 = '691104a40021a6ab55c9690d0523e4abc06fb8bd715b5ed93dfb9f4aabbcd505'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_local_authoring(stage, inherited_overlay):
    """The strong leaf may contain only four API additions/eight solver values."""
    from pxr import Sdf
    layer = stage.GetRootLayer()
    if list(layer.subLayerPaths) != [str(Path(inherited_overlay).resolve())] or \
            any(offset != Sdf.LayerOffset() for offset in layer.subLayerOffsets):
        raise ValueError('Banana leaf must inherit exactly the unchanged base overlay')
    found, apis = {}, set()

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            if str(spec.owner.path) not in base.KETTLE_BODIES or spec.name not in SOLVER_ATTRIBUTES or \
                    spec.typeName != Sdf.ValueTypeNames.Int or \
                    set(spec.ListInfoKeys()) - {'typeName', 'custom', 'variability', 'default'}:
                raise ValueError('Unexpected banana leaf attribute: ' + str(path))
            found[str(path)] = int(spec.default)
        elif isinstance(spec, Sdf.RelationshipSpec):
            raise ValueError('Banana leaf may not change relationships or joints')
        elif isinstance(spec, Sdf.PrimSpec):
            if path == Sdf.Path.absoluteRootPath:
                if set(spec.ListInfoKeys()) - {'subLayers', 'subLayerOffsets', 'defaultPrim'} or layer.defaultPrim != 'World':
                    raise ValueError('Unexpected banana leaf root metadata')
                return
            if set(spec.ListInfoKeys()) - {'specifier', 'apiSchemas'} or spec.specifier != Sdf.SpecifierOver:
                raise ValueError('Unexpected banana leaf prim metadata/type: ' + str(path))
            if spec.HasInfo('apiSchemas'):
                operation = spec.GetInfo('apiSchemas')
                if str(path) not in base.KETTLE_BODIES or operation.GetAddedOrExplicitItems() != ['PhysxRigidBodyAPI'] or \
                        operation.isExplicit or operation.deletedItems or operation.orderedItems:
                    raise ValueError('Unexpected banana leaf API schema')
                apis.add(str(path))

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    expected = {body+'.'+key: value for body in base.KETTLE_BODIES for key, value in SOLVER_ATTRIBUTES.items()}
    if found != expected or apis != set(base.KETTLE_BODIES):
        raise ValueError('Missing/changed/extra banana solver authoring')
    return dict(passed=True, solver_attributes=found, physx_api_added_bodies=sorted(apis),
                inherited_overlay=str(Path(inherited_overlay).resolve()),
                scope='Only four Kettle PhysxRigidBodyAPI additions and eight solver iteration values; no mass, COM, inertia, collision, xform, drive, joint or relationship edits.')


def create_leaf(inherited_overlay, output):
    """Author a fresh local layer. Used separately by CPU preservation tests."""
    from pxr import Sdf, Usd, UsdPhysics
    inherited_overlay, output = Path(inherited_overlay).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    original = Usd.Stage.Open(str(inherited_overlay))
    base.validate_local_authoring(original)
    actual = {str(prim.GetPath()) for prim in original.Traverse()
              if str(prim.GetPath()).startswith('/World/KettleA01/') and prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    if actual != set(base.KETTLE_BODIES):
        raise ValueError('The full four-body Kettle source inventory is required')
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [str(inherited_overlay)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    for path in base.KETTLE_BODIES:
        prim = stage.GetPrimAtPath(path)
        prim.AddAppliedSchema('PhysxRigidBodyAPI')
        for key, value in SOLVER_ATTRIBUTES.items():
            prim.CreateAttribute(key, Sdf.ValueTypeNames.Int, custom=False).Set(value)
    validation = validate_local_authoring(stage, inherited_overlay)
    stage.GetRootLayer().Save()
    return validation


def mounted_banana_kitchen(source, output_dir, *, layout):
    """Return (USD path, provenance), restricted to the measured banana layout.

    ``layout`` may add a hazard but may not change any original source task
    geometry/placement. The full safety policy is checked by the environment.
    Reports remain local artifacts: their exact hashes are recorded here; this
    leaf neither invents missing reports nor treats 10 s evidence as acceptance.
    """
    from pxr import Usd
    source, output_dir = Path(source).resolve(), Path(output_dir).resolve()
    original_layout = {key: value for key, value in layout.items() if key != 'obstacle'}
    if sha(LAYOUT_PATH) != LAYOUT_SHA256 or original_layout != json.loads(LAYOUT_PATH.read_text()) or \
            original_layout.get('task_id') != TASK:
        raise ValueError('Banana candidate is restricted to the measured canonical source layout')
    if source != (ROOT/original_layout['kitchen_usd']).resolve() or sha(source) != SOURCE_SHA256:
        raise ValueError('Banana candidate original kitchen source identity differs')
    inherited, profile = base.mounted_kitchen(source, output_dir)
    inherited = inherited.resolve()
    recipe_path, base_recipe_path = Path(__file__).resolve(), Path(base.__file__).resolve()
    recipe_sha = sha(recipe_path)
    path = output_dir/f'{RECIPE}_{SOURCE_SHA256[:16]}_{recipe_sha[:12]}.usda'
    metadata = path.with_suffix('.json')
    if path.exists() != metadata.exists():
        raise ValueError('Incomplete banana candidate cache; preserve it and use a fresh directory')
    if not path.exists():
        create_leaf(inherited, path)
    stage = Usd.Stage.Open(str(path))
    validation = validate_local_authoring(stage, inherited)
    info = copy.deepcopy(profile)
    info.update(recipe=RECIPE, recipe_source_path=str(recipe_path), recipe_sha256=recipe_sha,
        overlay_path=str(path), overlay_sha256=sha(path),
        base_recipe_source_path=str(base_recipe_path), base_recipe_sha256=sha(base_recipe_path),
        base_overlay_path=str(inherited), base_overlay_sha256=sha(inherited), base_profile=profile,
        local_authoring_validation=validation, inherited_local_authoring_validation=profile['local_authoring_validation'],
        candidate_status='10-second probe candidate; longer neutral and actual collection still required',
        physical_profile_certified=False, task_scope=TASK, source_layout_sha256=LAYOUT_SHA256,
        preserved=['All source geometry, collision, mass, COM and inertia',
                   'Source initial poses, gravity, internal joints and original door mobility',
                   'The inherited refrigerator housing installation', 'Original robot/task geometry and task predicates'],
        runtime_state_writes=False)
    info['kettle_solver'] = dict(position_iterations=128, velocity_iterations=8, bodies=list(base.KETTLE_BODIES),
        source_attributes=profile['kettle_solver']['source_attributes'],
        inherited_solver_attributes=profile['local_authoring_validation']['solver_attributes'],
        world_stability_thresholds=dict(translation_m=.002, rotation_deg=1.),
        selection_rule='First passing entry of predeclared ordered candidates [(128,8),(255,16)]; both outcomes retained.',
        selection_scope='One canonical G banana layout, seed42, CPU simulation, 2 physics/2 torch threads, 10 seconds/1201 samples. 128/8 passed, 255/16 failed: effects are nonmonotonic, not a convergence proof. Requires a new 45-second neutral and actual complete collection.',
        diagnostics=[dict(position_iterations=pos, velocity_iterations=vel, passed=passed,
            directory=f'reports/replicator_execution_20261023/g_banana/kettle_solver_{pos}_{vel}_01',
            report_sha256=report_sha, physics_states_sha256=raw_sha,
            predeclared_plan_sha256='9612c02af58abfbf07b45bcbf8ae5aa13f9210c33016f7a79ea896e90c08771d')
            for pos, vel, passed, report_sha, raw_sha in DIAGNOSTICS],
        selected_diagnostic_index=0, independent_reconstruction=dict(path=MATRIX_PATH, sha256=MATRIX_SHA256),
        retained_failed_canonical_neutral=dict(directory='reports/replicator_execution_20261023/g_banana/neutral_v2_03',
            independent_review_sha256='8220d913d0e5450e27674ecb3c6dbd04d8d59fcd25601937539a8ad038f4a1a6'))
    if metadata.exists():
        if json.loads(metadata.read_text()) != info:
            raise ValueError('Cached banana candidate profile/evidence/hash differs')
    else:
        metadata.write_text(json.dumps(info, indent=2)+'\n')
    return path, info
