"""Copy immutable small review artifacts after full task finalization."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def episode_destinations(roots):
    """Preserve old names unless distinct episodes share a basename."""
    roots = set(map(Path, roots))
    counts = {root.name: sum(p.name == root.name for p in roots) for root in roots}
    names = {root: root.name if counts[root.name] == 1 else
             root.parent.name + '__' + root.name + '__' +
             hashlib.sha256(str(root).encode()).hexdigest()[:8] for root in roots}
    if len(set(names.values())) != len(roots):
        raise RuntimeError('Cannot assign distinct evidence destinations')
    return names


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('delivery', type=Path)
    args = parser.parse_args()
    out = args.delivery
    read = lambda path: json.loads(path.read_text())
    if (out / 'evidence_copies.json').exists() or (out / 'evidence').exists():
        raise FileExistsError('Evidence has already been copied')
    if not read(out / 'gate_summary.json')['complete']:
        raise RuntimeError('Full finalization is required')
    selected = read(out / 'selection.json')
    copies, assets, git_paths, roots = {}, {}, set(), set()
    for item in selected['episodes'].values():
        roots.add(Path(item['episode']))
        if item.get('baseline'):
            roots.add(Path(item['baseline']))
    for root in list(roots):
        review = read(root / 'task_review_audit.json')
        roots.add(Path(review['matching_neutral']['path']))
    destinations = episode_destinations(roots)

    def copy(source, dest):
        if dest.exists():
            if sha(source) == sha(dest):
                return
            raise FileExistsError(dest)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, dest)
        if sha(source) != sha(dest):
            raise RuntimeError('Copy changed: ' + str(source))
        copies[str(dest)] = dict(source=str(source), sha256=sha(dest))

    for item in selected['episodes'].values():
        root = Path(item['episode'])
        roots.add(root)
        if item.get('baseline'):
            roots.add(Path(item['baseline']))
        for key in ['visual_review', 'source_risk_review', 'paired_report']:
            if key not in item:
                continue
            p = Path(item[key])
            dest = out / 'evidence' / destinations[root]
            copy(p, dest / ('paired_report.json' if key == 'paired_report' else p.name))
            if key != 'paired_report':
                report = read(p)
                image = Path(report['image'])
                copy(image, dest / image.name)
                if key == 'source_risk_review':
                    p = Path(report['illustration_report'])
                    copy(p, dest / p.name)
    for root in sorted(roots):
        for name in ['metadata.json', 'result.json', 'task_review_audit.json', 'task_validation_full.json',
                     'source_contact_audit.json', 'self_collision_audit.json', 'avoidance_audit.json',
                     'baseline_comparison_external_camera_2_03.json']:
            p = root / name
            if p.exists():
                copy(p, out / 'evidence' / destinations[root] / name)
        m = read(root / 'metadata.json')
        git_paths.add(m['variant_config'])
        spec = m.get('variant_spec') or {}
        paths = [Path(spec['asset'])] if spec.get('asset') else []
        if spec.get('support'):
            paths.append(Path(spec['support']['asset']))
            git_paths.add(spec['support']['asset'])
        for asset in paths:
            files = list(asset.parent.rglob('*')) if asset.parent.as_posix().startswith(
                'assets/extensions/doors/human') else [asset]
            for p in files:
                if not p.is_file():
                    continue
                assets[str(p)] = dict(bytes=p.stat().st_size, sha256=sha(p))
                if p.as_posix().startswith('assets/extensions/doors/') and p.suffix in ['.json', '.usda']:
                    git_paths.add(str(p))
        visual = Path('reports/doors') / root.parent.name / (root.name + '_visual_review01.json')
        if visual.exists():
            copy(visual, out / 'evidence' / destinations[root] / visual.name)
            p = Path(read(visual)['image'])
            copy(p, out / 'evidence' / destinations[root] / p.name)
    (out / 'evidence_episode_paths.json').write_text(json.dumps(
        {str(root): 'evidence/' + name for root, name in sorted(destinations.items())},
        indent=2) + '\n')
    test = Path(selected['unit_test_log'])
    copy(test, out / 'evidence' / test.name)
    for name, value in [('evidence_copies.json', copies), ('selected_asset_hashes.json', assets)]:
        (out / name).write_text(json.dumps(value, indent=2) + '\n')
    git_paths.update(str(p) for p in out.rglob('*') if p.is_file())
    git_paths.add(str(out / 'git_delivery_paths.json'))
    (out / 'git_delivery_paths.json').write_text(json.dumps(sorted(git_paths), indent=2) + '\n')
    print('EVIDENCE_COPIES', len(copies), 'ASSET_HASHES', len(assets), flush=True)


if __name__ == '__main__':
    main()
