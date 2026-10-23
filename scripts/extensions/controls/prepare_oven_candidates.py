"""Bind rebuilt source assets and produce an explicitly unaccepted candidate."""
import hashlib
import json
import argparse
from pathlib import Path


def main():
    root=Path(__file__).resolve().parents[3]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--asset-dir',default='assets/extensions/controls/human_oven_reach_low')
    p.add_argument('--config',default='configs/extensions/controls/temperature_augmented01.json')
    p.add_argument('--output',default='configs/extensions/controls/temperature_augmented02.json')
    a=p.parse_args()
    asset=root/a.asset_dir
    output=root/a.output
    if output.exists() or (asset/'artifact_manifest.json').exists():
        raise FileExistsError('Candidate or manifest already exists')
    pose=json.loads((asset/'pose.json').read_text())
    audit=json.loads((asset/'rebuild_geometry_audit.json').read_text())
    if not audit['passed']:raise ValueError('Rebuilt human geometry audit failed')
    files=[asset/'person.usdc',asset/'collidable.usda',asset/'pose.json',*sorted((asset/'textures').glob('*'))]
    manifest={'source':pose['source'],'source_sha256':pose['source_sha256'],
        'tested_blender':pose['blender_version'], 'geometry_provenance':pose['method'],
        'rebuild_geometry_audit':'rebuild_geometry_audit.json',
        'generated_files':{str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
        'rebuild':f'Blender original posed.blend; repose_human_arm.py --wrist {pose["target_wrist_world_asset_m"]} --asset-dir {a.asset_dir}; retain recorded pose method/elbow branch; then wrap_reposed_human.py and audit_human_export_blender.py'}
    (asset/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    config=json.loads((root/a.config).read_text())
    config['variants']['human']['asset']=str((asset/'collidable.usda').relative_to(root))
    config['variants']['property']['asset']='assets/extensions/controls/oven_glass_floor_lamp_low.usda'
    config['inspection_stage']='reports/controls_temperature_zxw_20261023/inspection03/stage.usda'
    config['augmentation_design']+=' Lower wrist and source-authored reading lamp candidate; no acceptance claimed before independent gates.'
    output.write_text(json.dumps(config,indent=2)+'\n')


if __name__=='__main__':main()
