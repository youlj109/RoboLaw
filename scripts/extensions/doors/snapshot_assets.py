"""Hash resolved USD dependencies used by this C01 delivery, including rebuilt human bytes."""
import argparse
import hashlib
import json
from pathlib import Path
from pxr import UsdUtils
from kitchen_safety.extensions.doors.environment import ROOT, TASK


def file_hash(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def label(path):
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mdl-root', type=Path, help='Explicit native Kit core/Base MDL directory; CPU USD lacks the Kit MDL resolver')
    p.add_argument('episodes', nargs='+', type=Path)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refusing to overwrite provenance evidence')
    roots = {ROOT/'assets/kitchen/scene.usd', Path.home()/'.cache/isaaclab_arena/usd/robot_on_stand/doors_droid_on_stand_0.8_1.08x0.91032.usd'}
    placements = {}
    for ep in a.episodes:
        m = json.loads((ep/'metadata.json').read_text())
        spec = m['variant_spec']
        if spec:
            roots.add(ROOT/spec['asset'])
            if spec.get('support'):
                roots.add(ROOT/spec['support']['asset'])
        placements[str(ep)] = {'variant': m['variant'], 'robot_base': m['robot_base'],
            'robot_yaw_deg': m['robot_yaw_deg'], 'variant_spec': spec,
            'actual_hazard_initial_pose': m.get('actual_hazard_initial_pose')}
    resolved = {TASK, ROOT/'assets/people/mpfb_corridor/posed.blend'}
    resolved.update(path for path in (ROOT/'assets/curobo/droid').rglob('*') if path.is_file())
    unresolved = []
    for root in sorted(roots):
        layers, assets, missing = UsdUtils.ComputeAllDependencies(str(root))
        resolved.add(root)
        resolved.update(Path(layer.realPath) for layer in layers if layer.realPath)
        resolved.update(Path(asset) for asset in assets)
        unresolved.extend(str(x) for x in missing)
    native_mdl = {}
    remaining = []
    for name in sorted(set(unresolved)):
        candidate = a.mdl_root/name if a.mdl_root and Path(name).name == name and name.endswith('.mdl') else None
        if candidate and candidate.is_file():
            resolved.add(candidate)
            native_mdl[name] = str(candidate)
        else:
            remaining.append(name)
    absent = sorted(str(path) for path in resolved if not path.is_file())
    files = {label(path): {'sha256': file_hash(path), 'bytes': path.stat().st_size}
             for path in sorted(resolved) if path.is_file()}
    report = {'resolved_dependencies_present': not remaining and not absent,
        'source_task_sha256': file_hash(TASK), 'usd_roots': [label(p) for p in sorted(roots)],
        'files': files, 'cpu_usd_unresolved_paths': sorted(set(unresolved)),
        'native_kit_mdl_lookup': native_mdl, 'unresolved_usd_paths': remaining, 'missing_files': absent,
        'placements': placements,
        'model_boundary': 'FCL uses source triangle meshes, actual articulated robot collision hulls and exact supported hazard primitives for necessity. Live planning retains conservative envelopes/AABBs. These models are not claimed identical to PhysX cooked shapes. Human export byte difference is disclosed in artifact_manifest_lwx.json; the original manifest is not changed.'}
    a.output.write_text(json.dumps(report, indent=2))
    print('ASSET_SNAPSHOT', len(files), 'files', 'missing', len(absent), 'unresolved', len(remaining), flush=True)
    return 0 if report['resolved_dependencies_present'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
