"""Package existing raw episodes without rewriting or duplicating physical claims."""
import argparse
import hashlib
import json
import os
from pathlib import Path

RAW_FILES=['config.json','records.json','contacts.json','mechanism_audit.json',
    'plans.json','trajectory.npz','audit.json','validation.json','result.json',
    'cameras.json','images.h5','external_camera.mp4','external_camera_2.mp4',
    'wrist_camera.mp4','all_cameras.mp4']


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def package(sources,output):
    if output.exists():raise FileExistsError(output)
    task_ids=set()
    for role,source in sources.items():
        result=json.loads((source/'result.json').read_text())
        if result.get('accepted') is not True or result['variant']!=role:
            raise ValueError('Rejected or mismatched source '+str(source))
        task_ids.add(result['task_id'])
        for name in RAW_FILES:
            if not (source/name).is_file():raise FileNotFoundError(source/name)
    if len(task_ids)!=1:raise ValueError('Mixed original task IDs')
    output.mkdir(parents=True)
    manifest={'task_id':next(iter(task_ids)),'physical_episode_count':len(sources),
        'accepted':False,'episodes':{},
        'scope':'Hardlinked raw evidence from the same physical episodes, not additional demonstrations. Independent acceptance files are deliberately not carried over; all final gates must rerun. Historical source directories are retained unchanged.'}
    for role,source in sources.items():
        destination=output/role
        destination.mkdir()
        files={}
        for name in RAW_FILES:
            os.link(source/name,destination/name)
            files[name]=sha(destination/name)
        manifest['episodes'][role]={'source':str(source),'packaged':str(destination),'raw_sha256':files}
    (output/'raw_packaging_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for role in ('base','human','property','robot'):p.add_argument('--'+role,type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    report=package({role:getattr(a,role) for role in ('base','human','property','robot')},a.output)
    print(json.dumps({'task_id':report['task_id'],'output':str(a.output),'accepted':False,'physical_episode_count':4},indent=2))
