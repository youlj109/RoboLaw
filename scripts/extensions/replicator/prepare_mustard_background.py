#!/usr/bin/env python3
"""Compose/inspect the disclosed source-local background overlay on CPU only."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from kitchen_safety.extensions.replicator.mustard_family_background import family_mounted_kitchen


def main():
    from pxr import Usd,UsdPhysics
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preparation',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=False)
    report=dict(passed=False,physics_launched=False,neutral_validated=False,mobility_validated=False,execution_ready=False)
    try:
        prepared=load_preparation(args.preparation);source=ROOT/prepared['layout']['kitchen_usd']
        original=Usd.Stage.Open(str(source))
        before={str(Path(layer.realPath).resolve()):sha(layer.realPath) for layer in original.GetUsedLayers() if Path(layer.realPath).is_file()}
        overlay,info=family_mounted_kitchen(source,args.output_dir)
        stage=Usd.Stage.Open(str(overlay))
        def joints(value):
            return {str(p.GetPath()):dict(type=p.GetTypeName(),body0=list(map(str,UsdPhysics.Joint(p).GetBody0Rel().GetTargets())),
                body1=list(map(str,UsdPhysics.Joint(p).GetBody1Rel().GetTargets()))) for p in value.Traverse() if p.IsA(UsdPhysics.Joint)}
        source_joints,composed_joints=joints(original),joints(stage)
        original_preserved=all(composed_joints.get(name)==value for name,value in source_joints.items())
        extra=set(composed_joints)-set(source_joints)
        unchanged=all(sha(path)==digest for path,digest in before.items())
        passed=original_preserved and extra=={info['mount_joint']} and unchanged and info['local_authoring_validation']['passed']
        report.update(task_id=prepared['task_id'],preparation_sha256=sha(args.preparation),source_layers_sha256=before,
            original_layers_unchanged=unchanged,original_joint_inventory=source_joints,
            original_joint_inventory_preserved=original_preserved,added_joint_paths=sorted(extra),
            overlay_path=str(overlay),overlay_sha256=sha(overlay),overlay_provenance=info,passed=bool(passed),
            scope='USD composition and strict local-authoring inventory only. No physical stability, door mobility, rendered-camera or task execution claim.')
    except Exception as error:report['error']=f'{type(error).__name__}: {error}'
    (args.output_dir/'composition_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output_dir),passed=report['passed'],error=report.get('error'),physics_launched=False)),flush=True)
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
