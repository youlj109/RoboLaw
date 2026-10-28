"""Bind one explicitly authorized visual arm revision, not a visual verdict."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex


def prepare(source, asset, output):
    manifest = asset / 'artifact_manifest.json'
    if output.exists() or manifest.exists():
        raise FileExistsError('Historical revision output must not be overwritten')
    pose = json.loads((asset / 'pose.json').read_text())
    audit = json.loads((asset / 'rebuild_geometry_audit.json').read_text())
    required = ('six_exported_meshes', 'feet_unchanged', 'foot_vertex_count_unchanged',
                'non_arm_bones_unchanged', 'wrist_target_reached',
                'source_wrist_frame_preserved', 'source_wrist_bend_preserved')
    if (audit.get('passed') is not True or
            any(audit.get('checks', {}).get(key) is not True for key in required) or
            pose.get('wrist_structure', {}).get('mode') != 'follow-forearm'):
        raise ValueError('Require an independently audited source-relative wrist and preserved feet')
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    if (audit.get('source_sha256') != pose['source_sha256'] or
            sha(Path(pose['source'])) != pose['source_sha256'] or
            sha(asset / 'person.usdc') != audit.get('export_sha256')):
        raise ValueError('Geometry audit does not bind the current source/export')
    files = [asset / name for name in ('person.usdc', 'collidable.usda', 'pose.json',
                                     'rebuild_geometry_audit.json')]
    files += sorted((asset / 'textures').glob('*'))
    binding = dict(source=pose['source'], source_sha256=pose['source_sha256'],
        tested_blender=pose['blender_version'], geometry_provenance=pose['method'],
        generated_files={str(path): sha(path) for path in files},
        wrist_structure=pose['wrist_structure'],
        rebuild='Blender original posed.blend; repose_human_arm.py ' +
                shlex.join(pose['recipe_args']) + '; wrap_reposed_human.py; audit_human_export_blender.py',
        scope='Asset provenance only, not a visual, planning or safety acceptance.')
    config = json.loads(source.read_text())
    config['variants']['human']['asset'] = str(asset / 'collidable.usda')
    config['visual_revision'] = dict(source=str(source), source_sha256=sha(source), variant='human',
        reason='User authorized a new visual right-arm revision after fixed-layout contact failures; '
               'keep body root, feet, original bone lengths and forearm-relative wrist frame.',
        scope='Unreviewed visual candidate; original task, robot and other variants unchanged.',
        accepted_for_collection=False, warning='No earlier physical or safety acceptance transfers.')
    manifest.write_text(json.dumps(binding, indent=2) + '\n')
    config['human_pose_revision'] = dict(manifest=str(manifest), manifest_sha256=sha(manifest),
        scope='New right-arm pose requires fresh rendered visual review, freeze and all acceptance gates.')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + '\n')
    return config


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--asset-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.asset_dir, args.output)
    print('UNREVIEWED_ARM_REVISION', args.output)
