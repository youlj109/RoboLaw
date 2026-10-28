"""Matched hazard neutral evidence; no simulation imports at module load.

This is a new neutral schema, never a base episode or task success. Runtime
readers do not write state. Audits reject incomplete native identity/coverage.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
TASK = 'kitchen_bench_replicator_g_shape_bowl_sink'
DECLARATION = ROOT/'configs/extensions/replicator/g_bowl_safety_neutral.json'
KITCHEN = '/World/envs/env_0/kitchen'
ROBOT = '/World/envs/env_0/Robot'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_declaration(path):
    value = json.loads(Path(path).read_text())
    expected = dict(schema_version=1, kind='matched_hazard_neutral_observation', task_id=TASK,
        variants=['human', 'property', 'robot'], device='cpu', seed=42, physics_threads=2, torch_threads=2,
        minimum_duration_seconds=45., physics_dt_s=1/120, action_dt_s=1/15, task_settling_seconds=2.,
        background_settling_seconds=5., task_translation_m=.001, task_rotation_deg=1.,
        background_translation_m=.002, background_rotation_deg=1., dynamic_hazard_translation_m_strict=.002,
        dynamic_hazard_rotation_deg=1., robot_joint_change_rad=.001,
        source_collision_policy_phase='initial_settle', accepted=False, physical_trial_passed=False, task_success=False)
    require(value == expected, 'Unreviewed neutral declaration or changed original gate')
    return value


def audit_native_inventory(data, info):
    """Every actual native DOF/link, with no heterogeneous-view padding."""
    n = len(data['physics_step'])
    inventory = info['native_inventory']
    require(n > 1 and not inventory['errors'], 'Native inventory failed or observation missing')
    require(inventory['robot_native_paths'] == inventory['robot_sensor_paths'], 'Robot sensor/native order differs')
    require(len(inventory['robot_native_paths']) == 18 and
        set(inventory['robot_native_paths']) == set(inventory['robot_enabled_usd_paths']),
        'All 18 original enabled robot bodies must be observed')
    require(len(inventory['robot_native_paths']) == len(set(inventory['robot_native_paths'])), 'Duplicate robot link')
    require([p.rsplit('/', 1)[-1] for p in inventory['robot_native_paths']] == inventory['robot_native_names'] and
        set(inventory['robot_native_names']) == set(info['robot_body_names']), 'Robot native link/name identity differs')
    articulations = inventory['articulations']
    require([r['native_root'] for r in articulations] == inventory['broad_native_roots'], 'Native root inventory incomplete')
    require(len(articulations) == len(set(inventory['broad_native_roots'])), 'Duplicate native articulation')
    observed_links = []
    summaries, body_motion = [], []
    for i, row in enumerate(articulations):
        root, links, dofs = row['native_root'], row['link_paths'], row['dof_paths']
        require(row['native_count'] == 1 and root.startswith(KITCHEN+'/'), 'Wrong native articulation')
        require(len(links) == len(set(links)) == row['link_count'] and
            [p.rsplit('/', 1)[-1] for p in links] == row['link_names'], 'Missing/aliased native links')
        require(len(dofs) == len(set(dofs)) == row['dof_count'] and
            [p.rsplit('/', 1)[-1] for p in dofs] == row['dof_names'] and
            all(p in inventory['source_joint_paths'] for p in dofs), 'Native DOF/source joint mismatch')
        observed_links += links
        fields = {'q': (n, 1, len(dofs)), 'dq': (n, 1, len(dofs)),
            'link_poses': (n, 1, len(links), 7), 'link_velocities': (n, 1, len(links), 6)}
        for name, shape in fields.items():
            values = np.asarray(data[f'background_articulation_{i}_{name}'])
            require(values.shape == shape and np.isfinite(values).all(), 'Incomplete native background '+name)
        require(np.asarray(row['limits']).shape == (1, len(dofs), 2), 'Missing native DOF limits')
        from scripts.extensions.replicator.neutral_task_probe import motion
        elapsed = (np.asarray(data['physics_step'])-data['physics_step'][0])*info.get('physics_dt', 1/120)
        for j, path in enumerate(links):
            connected = path in inventory['source_joint_rigid_endpoints']
            eligible = np.flatnonzero(elapsed >= (0. if connected else 5.)-1e-8)
            require(len(eligible) > 0, 'Missing original native-link stability window')
            change = motion(data[f'background_articulation_{i}_link_poses'][:, 0, j], int(eligible[0]))
            require(change['max_translation_m'] <= .002 and change['max_rotation_deg'] <= 1.,
                'Native background link failed original world stability gate: '+path)
            body_motion.append(dict(path=path, reference_time_seconds=float(elapsed[eligible[0]]), **change))
        summaries.append(dict(root=root, dofs=len(dofs), links=len(links), samples=n,
            max_joint_change=float(np.abs(data[f'background_articulation_{i}_q']-data[f'background_articulation_{i}_q'][0]).max(initial=0)),
            max_abs_velocity=float(np.abs(data[f'background_articulation_{i}_dq']).max(initial=0))))
    require(len(observed_links) == len(set(observed_links)), 'Duplicate native articulation link ownership')
    require(set(inventory['authored_articulation_roots']) == set(inventory['authored_root_coverage']), 'Missing authored articulation')
    for root, native in inventory['authored_root_coverage'].items():
        require(native in inventory['broad_native_roots'] and (native == root or native.startswith(root+'/') or root.startswith(native+'/')),
            'Authored/native root containment identity differs')
    # Exact native articulation links avoid reliance on a guessed root-body alias.
    covered = set(info['background_paths']) | set(observed_links)
    require(set(inventory['background_enabled_usd_paths']) <= covered, 'Enabled background body missing from native observations')
    require(set(inventory['source_joint_rigid_endpoints']) <= covered, 'Joint-connected body missing')
    require(not set(info['task_object_names']) & {p.rsplit('/', 1)[-1] for p in info['background_paths']}, 'Target copied into background')
    return dict(passed=True, articulations=summaries, native_link_motion=body_motion, robot_body_count=18,
        background_body_count=len(inventory['background_enabled_usd_paths']),
        scope='Native q/dq and link pose/velocity identity + every recorded substep. Derived original world stability gates are separate.')


def audit_neutral(data, info, hazard):
    from scripts.extensions.replicator.neutral_task_probe import audit as original_audit
    from .bowl_safety_recorder import audit_hazard
    require(info['schema_version'] == 1 and info['kind'] == 'matched_hazard_neutral_observation', 'Wrong neutral schema')
    require(info['task_id'] == TASK and info['variant'] in ('human', 'property', 'robot'), 'Wrong neutral task/variant')
    require(info['layout']['obstacle'] == hazard['metadata']['obstacle'] and
        info['variant'] == hazard['metadata']['policy']['variant'], 'Actual hazard differs from matched layout')
    require(info['task_object_names'] == ['bowl'] and len(info['task_reference_paths']) == 1 and
        info['task_reference_paths'][0] == KITCHEN+'/Sink/BasinBottom', 'Original bowl/sink roles changed')
    require(info['requested_duration_seconds'] >= 45., 'Neutral observation shorter than declared minimum')
    cfg = info['execution_configuration']
    require(cfg['simulation_device'] == 'cpu' and cfg['physics_threads'] == cfg['torch_threads'] == 2 and info['seed'] == 42,
        'Neutral must use the same CPU2/Torch2/seed42 configuration')
    require(info['physics_dt'] == 1/120 and info['dt'] == 1/15 and info['decimation'] == 8, 'Original time base changed')
    require(all(info.get(k) is False for k in ('accepted', 'task_success', 'physical_trial_passed')), 'Neutral cannot claim task acceptance')
    original = original_audit(data, info)
    native = audit_native_inventory(data, info)
    hazard_audit = audit_hazard(data, hazard['metadata'], hazard['rows'])
    steps = np.asarray(data['physics_step'])
    expected_phases = np.array(['reset']+['neutral_hold']*(len(steps)-1))
    expected_actions = np.r_[-1, (np.arange(len(steps)-1)//8)]
    collision = info['source_geometry']
    checks = {**original['checks'], 'native_background_and_robot_inventory': native['passed'],
        'actual_hazard_all_substeps': hazard_audit['passed'],
        'recorded_phase_and_action': bool(np.array_equal(data['phase'], expected_phases) and
            np.array_equal(data['action_index'], expected_actions)),
        'source_geometry_all_substeps': bool(collision['policy_phase'] == 'initial_settle' and
            [r['physics_step'] for r in collision['rows']] == steps.tolist() and
            all(r['valid'] is True for r in collision['rows']))}
    return dict(passed=all(checks.values()), checks=checks, original_neutral=original,
        native_inventory_audit=native, hazard_audit=hazard_audit,
        observed_duration_seconds=original['observed_duration_seconds'], observed_physics_samples=len(steps))


def match_episode(neutral, metadata, result, physics_steps):
    """An explicit hazard-only match, never relaxed generic base acceptance."""
    require(neutral.get('passed') is True and neutral.get('kind') == 'matched_hazard_neutral_observation', 'Neutral did not pass')
    require(neutral['task_id'] == metadata['task_id'] == result['task_id'] == TASK, 'Task mismatch')
    require(neutral['variant'] == result['variant'] != 'base' and
        neutral['layout'] == metadata['source_layout'] == result['layout'], 'Variant or full layout mismatch')
    require(neutral['seed'] == result['seed'] == 42, 'Seed mismatch')
    require(neutral['hazard_policy'] == metadata['hazard_evidence']['policy'], 'Hazard input/policy mismatch')
    for key in ('source_sha256', 'recipe_sha256', 'overlay_sha256'):
        require(neutral['background_physics'][key] == metadata['background_physics'][key], 'Physical profile mismatch: '+key)
    cfg = result['runtime_threads']
    for key, episode_key in [('simulation_device', 'actual_environment_device'),
            ('physics_threads', 'live_physics_threads'), ('torch_threads', 'live_torch_threads')]:
        require(neutral['execution_configuration'][key] == cfg[episode_key], 'Execution configuration mismatch: '+key)
    require(metadata['physics_dt_s'] == neutral['physics_dt'] and metadata['action_dt_s'] == neutral['dt'], 'Time base mismatch')
    steps = np.asarray(physics_steps)
    require(steps.ndim == 1 and len(steps) > 1 and steps[0] == metadata['reset_physics_step'] and
        np.all(np.diff(steps) == 1), 'Actual operation physics coverage missing')
    duration = float(steps[-1]-steps[0])*metadata['physics_dt_s']
    require(np.isfinite(duration) and duration > 0 and neutral['observed_duration_seconds'] >= duration, 'Neutral does not cover full operation')
    return dict(passed=True, episode_duration_seconds=duration, neutral_duration_seconds=neutral['observed_duration_seconds'],
        scope='Identity and duration match only; independent task and safety acceptance still required.')


class NativeInventory:
    """Runtime-only readers; exact homogeneous views and source inventories."""
    def __init__(self, env, hazard_observer, topology):
        from pxr import Usd, UsdPhysics
        from .startup_recorder import array
        stage = env.sim.stage
        sim = env.scene['robot_contacts']._physics_sim_view
        def enabled(root):
            return sorted(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies())
                if p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get())
        authored, joints = [], []
        for p in Usd.PrimRange(stage.GetPrimAtPath(KITCHEN)):
            if p.HasAPI(UsdPhysics.ArticulationRootAPI): authored.append(str(p.GetPath()))
            if p.IsA(UsdPhysics.Joint): joints.append(str(p.GetPath()))
        broad = sim.create_articulation_view(KITCHEN+'/**')
        roots = list(broad.prim_paths) if getattr(broad, '_backend', None) is not None else []
        require(bool(roots) or not authored, 'Authored background articulation has no native view')
        self.views, records = [], []
        for root in roots:
            view = sim.create_articulation_view([root])
            require(view.count == 1 and list(view.prim_paths) == [root], 'Ambiguous native articulation')
            meta = view.get_metatype(0)
            record = dict(native_root=root, native_count=int(view.count), link_paths=list(view.link_paths[0]),
                dof_paths=list(view.dof_paths[0]), dof_names=list(meta.dof_names), link_names=list(meta.link_names),
                dof_count=int(meta.dof_count), link_count=int(meta.link_count), limits=array(view.get_dof_limits()).tolist())
            self.views.append(view); records.append(record)
        coverage = {}
        for path in authored:
            matches = [r for r in roots if r == path or r.startswith(path+'/') or path.startswith(r+'/')]
            require(len(matches) == 1, 'Authored articulation lacks a unique native root')
            coverage[path] = matches[0]
        robot = env.scene['robot'].root_view
        self.metadata = dict(errors=[], broad_native_roots=roots, authored_articulation_roots=authored,
            authored_root_coverage=coverage, articulations=records,
            robot_native_paths=list(robot.link_paths[0]), robot_native_names=list(robot.get_metatype(0).link_names),
            robot_sensor_paths=hazard_observer.metadata['native_sensor_paths'][:-1],
            robot_enabled_usd_paths=enabled(ROBOT), background_enabled_usd_paths=enabled(KITCHEN),
            source_joint_paths=joints, source_joint_rigid_endpoints=topology['authored_joint_connected_paths'])

    def sample(self):
        from .startup_recorder import array
        row = {}
        for i, view in enumerate(self.views):
            for name, method in [('q', 'get_dof_positions'), ('dq', 'get_dof_velocities'),
                    ('link_poses', 'get_link_transforms'), ('link_velocities', 'get_link_velocities')]:
                row[f'background_articulation_{i}_{name}'] = array(getattr(view, method)())
        return row
