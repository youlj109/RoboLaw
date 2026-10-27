"""Source-shape counterfactual screen of frozen visual candidates, never playback."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def summarize(samples):
    if not samples:
        raise ValueError('No counterfactual samples; empty evidence cannot pass')
    failures = [row for row in samples if not row['valid']]
    return dict(samples=len(samples), violations=len(failures),
                requires_adjustment=bool(failures),
                first_violation=failures[0] if failures else None,
                minimum=min(samples, key=lambda row: row['distance_m']))


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--current', type=Path, default=Path(
        'docs/extensions/controls/scene_review/quality_recheck02.json'))
    parser.add_argument('--physical', type=Path, default=Path(
        'docs/extensions/controls/scene_review/physical_final_20261027.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenes', nargs='+')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    import numpy as np
    from pxr import Gf, Usd, UsdGeom
    from scipy.spatial.transform import Rotation
    from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
    from kitchen_safety.extensions.controls.execution_contract import controller_signature
    from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
    from kitchen_safety.planning import DroidKinematics
    from scripts.extensions.controls.paired_planning import verify_asset_manifests
    from scripts.extensions.controls.render_first_violation import label_robot_hulls

    current = json.loads(args.current.read_text())['scenes']
    physical = {row['scene']: row for row in json.loads(args.physical.read_text())['scenes']}
    if args.scenes:
        missing = set(args.scenes)-{row['scene'] for row in current}
        if missing:
            raise ValueError('Unknown scenes: '+str(sorted(missing)))
        current = [row for row in current if row['scene'] in args.scenes]
    summary = dict(scope='Diagnostic counterfactual of recorded successful baseline versus current '
        'authored hazard shapes. Robot collision hulls and authored mesh triangles/round primitives '
        'are used, not PhysX cooked-shape identity. Action-rate samples, not continuous collision '
        'detection. No unsafe playback; no multi-seed or full benchmark acceptance.',
        safety_benchmark_accepted=False, scenes=[])
    for candidate in current:
        sid = candidate['scene']
        print('SCREEN_START', sid, flush=True)
        report = dict(scene=sid, status='failed')
        try:
            config_path = ROOT/candidate['visual_config']
            config = json.loads(config_path.read_text())
            digest = hashlib.sha256(config_path.read_bytes()).hexdigest()
            if digest != candidate['visual_config_sha256']:
                raise ValueError('Visual config changed after review')
            old = physical[sid]
            baseline = ROOT/old['baseline']
            result = json.loads((baseline/'result.json').read_text())
            records = json.loads((baseline/'records.json').read_text())
            source_config = json.loads((baseline/'config.json').read_text())
            if not result.get('accepted') or result['task_id'] != config['task_id']:
                raise ValueError('Reference is not a successful same-task physical baseline')
            if not np.allclose(result['initial_robot_pose'], config['robot_pose_xyzw'], atol=1e-7, rtol=0):
                raise ValueError('Reference robot pose differs')
            if result['source_sha256'] != config['source_sha256']:
                raise ValueError('Reference source task differs')
            variant = old['variant']
            key = dict(human='person', property='glass', robot='barrier')[variant]
            spec = config['variants'][variant]
            stage = Usd.Stage.CreateInMemory()
            prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/'+key)
            prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
            prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
            prim.AddRotateZOp().Set(spec['yaw_deg'])
            exact = static_collision_shapes(stage, str(prim.GetPath()))
            pose = config['robot_pose_xyzw']
            kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:]).as_euler('xyz', degrees=True)[2])
            world = ControlsCollisionWorld(stage, kin, env_root='/World/envs/env_0')
            names = result['robot_joint_names']
            ids = [names.index('panda_joint'+str(i)) for i in range(1, 8)]
            wrist = None
            pose_file = (ROOT/spec['asset']).parent/'pose.json'
            if variant == 'human' and pose_file.exists():
                local = json.loads(pose_file.read_text()).get('actual_wrist_world_asset_m')
                if local is not None:
                    wrist = (Rotation.from_euler('z', spec['yaw_deg'], degrees=True).apply(local)
                             +np.asarray(spec['position'])).tolist()
            samples = []
            for index, row in enumerate(records):
                for when in ('before', 'after'):
                    state = row[when]
                    raw = world.shapes(np.asarray(state['q'])[ids], dict(zip(names, state['q'])))
                    labeled = label_robot_hulls(raw, [body for body, _, _ in world.robot_hulls])
                    shapes = [s for s in labeled if s[0] not in ('arm', 'gripper')]
                    measurement = minimum_distance(shapes, exact, world.execution_clearances[key])
                    samples.append(dict(action_index=index, state=when, phase=row['phase'],
                        time_s=(index+(when == 'after'))*result['dt'],
                        human_wrist_world_m=wrist, **measurement))
            diagnosis = summarize(samples)
            report.update(status='complete', visual_config=candidate['visual_config'],
                visual_config_sha256=digest, baseline=old['baseline'],
                baseline_records_sha256=hashlib.sha256((baseline/'records.json').read_bytes()).hexdigest(),
                source_asset_sha256=hashlib.sha256((ROOT/spec['asset']).read_bytes()).hexdigest(),
                asset_manifests=verify_asset_manifests(config),
                reference_controller_matches_visual=controller_signature(source_config) == controller_signature(config),
                baseline_selection='Previously accepted physical reference, not selected by risk score. '
                    'If controller differs, this is diagnostic only, not a matched final pair.',
                hazard_pose='Nominal reviewed visual pose; dynamic property settling must be checked separately.',
                source_collider_count=len(exact), diagnosis=diagnosis)
            save(args.output/(sid+'.json'), dict(report, measurements=samples))
            print('SCREEN_DONE', sid, diagnosis['requires_adjustment'],
                  diagnosis['minimum']['distance_m'], flush=True)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            report['error'] = f'{type(exc).__name__}: {exc}'
            save(args.output/(sid+'.json'), report)
        summary['scenes'].append(report)
        summary['updated_utc'] = datetime.now(timezone.utc).isoformat()
        save(args.output/'summary.json', summary)


if __name__ == '__main__':
    main()
