"""Bind a fresh wrist-aligned human to both original button-task configs."""
import argparse
import hashlib
import json
import shlex
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--asset-dir',type=Path,required=True)
    p.add_argument('--revision',required=True)
    a=p.parse_args()
    asset=a.asset_dir
    pose=json.loads((asset/'pose.json').read_text())
    audit=json.loads((asset/'rebuild_geometry_audit.json').read_text())
    if not audit['passed'] or pose.get('wrist_structure',{}).get('mode')!='follow-forearm':
        raise ValueError('An independently audited forearm-relative wrist is required')
    configs={'left':'configs/extensions/controls/repaired_left_button.json',
             'right':'configs/extensions/controls/right_button_final_candidate.json'}
    outputs={key:Path('configs/extensions/controls')/(key+'_button_'+a.revision+'.json') for key in configs}
    manifest=asset/'artifact_manifest.json'
    if any(path.exists() for path in [manifest,*outputs.values()]):
        raise FileExistsError('Refusing to replace previous hand-revision evidence')
    files=[asset/'person.usdc',asset/'collidable.usda',asset/'pose.json',asset/'rebuild_geometry_audit.json',
           *sorted((asset/'textures').glob('*'))]
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    binding={'source':pose['source'],'source_sha256':pose['source_sha256'],
        'tested_blender':pose['blender_version'],'geometry_provenance':pose['method'],
        'rebuild_geometry_audit':'rebuild_geometry_audit.json',
        'generated_files':{str(path):sha(path) for path in files},
        'rebuild':'Blender 4.0.2 original posed.blend; repose_human_arm.py '+shlex.join(pose.get('recipe_args',[]))+'; wrap_reposed_human.py; audit_human_export_blender.py',
        'wrist_structure':pose['wrist_structure']}
    manifest.write_text(json.dumps(binding,indent=2)+'\n')
    for key,source in configs.items():
        config=json.loads(Path(source).read_text())
        config['variants']['human']['asset']=str(asset/'collidable.usda')
        config['human_pose_revision']={'previous_config':source,'previous_config_sha256':sha(Path(source)),
            'manifest':str(manifest),'manifest_sha256':sha(manifest),
            'scope':'Human right-arm pose revised using the source rig. Root/feet, original task, robot controller and safety distances are unchanged. New source-geometry, paired planning and physical human execution are required.'}
        outputs[key].write_text(json.dumps(config,indent=2)+'\n')
        print(outputs[key])


if __name__=='__main__':main()
