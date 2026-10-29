"""Read-only JSON/source checks for a separately versioned added-hazard pose.

Runtime binding gate for a new, frozen hazard pose. The independent auditor also
checks the composed USD transforms; original configurations remain immutable.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

SOURCE_KEYS=('scene_id','task_id','variant_id','result_path','result_sha256','config_path','config_sha256','source_scene_path','source_scene_sha256')
ALLOWED=('obstacle.position','obstacle.yaw_deg')


def validate_scene_revision(root, reference, *, expected_scene_id=None, current=None, digest=None):
    root=Path(root).resolve()
    def require(value,message):
        if not value:raise ValueError(message)
    def local(value):
        path=(root/str(value).removeprefix('/workspace/')).resolve()
        require(path.is_relative_to(root),'Revision path outside checkout');return path
    def sha(path):
        if digest:return digest(path)
        with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
    def bound(ref):
        require(isinstance(ref,dict) and isinstance(ref.get('path'),str) and isinstance(ref.get('sha256'),str),'Invalid revision binding')
        path=local(ref['path']);require(path.is_file() and sha(path)==ref['sha256'],'Revision source SHA differs: '+str(path));return path
    def read(ref):return json.loads(bound(ref).read_text())
    document=read(reference)
    require(document.get('kind')=='hazard_pose_revision' and document.get('retry_round')==7,'Unknown scene revision kind/round')
    sid=document['scene_id'];require(expected_scene_id is None or sid==expected_scene_id,'Scene revision identity differs')
    previous=read(document['prior_manifest']);rows={r['scene_id']:r for r in previous['entries']}
    require(previous.get('retry_round')==6 and previous.get('stage')=='complete' and sid in rows,'Revision must follow completed round6')
    original=rows[sid]
    require(original['execution_status']=='planning_failed' and original.get('executed') is False and original.get('attempts')==0,'Revision parent is not an unresolved unexecuted candidate')
    require(document['prior_row_source']=={k:original[k] for k in SOURCE_KEYS},'Revision changed historical source snapshot')
    for name,prefix in [('old_config','config'),('old_source_scene','source_scene')]:
        require(document[name]=={'path':original[prefix+'_path'],'sha256':original[prefix+'_sha256']},'Revision original binding differs: '+name)
    old,new=read(document['old_config']),read(document['new_config'])
    old_stage,new_stage=bound(document['old_source_scene']),bound(document['new_source_scene'])
    require(local(document['old_config']['path'])!=local(document['new_config']['path']) and old_stage!=new_stage,'Revision must use new files')
    a,b=deepcopy(old),deepcopy(new);changed=[]
    for field in ALLOWED:
        key=field.split('.')[1]
        require(key in a['obstacle'] and key in b['obstacle'],'Missing explicit hazard pose field')
        if a['obstacle'][key]!=b['obstacle'][key]:changed.append(field)
        a['obstacle'].pop(key);b['obstacle'].pop(key)
    require(a==b,'Revision changed fields beyond added-hazard position/yaw')
    require(isinstance(document['changed_fields'],list) and len(set(document['changed_fields']))==len(document['changed_fields'])
        and set(changed)==set(document['changed_fields']) and changed,'Revision changed_fields differs from actual pose delta')
    for config in (old,new):
        pos=config['obstacle']['position'];yaw=config['obstacle']['yaw_deg']
        require(isinstance(pos,list) and len(pos)==3 and all(type(v) in (int,float) and math.isfinite(v) for v in pos)
            and type(yaw) in (int,float) and math.isfinite(yaw),'Invalid finite hazard pose')
    if current is not None:
        require(current.get('scene_revision')==reference and current.get('layout_changed') is True,'Current row lacks its exact scene revision')
        for name,prefix in [('new_config','config'),('new_source_scene','source_scene')]:
            require(document[name]=={'path':current[prefix+'_path'],'sha256':current[prefix+'_sha256']},'Current row source differs from revision: '+name)
        for key in ('scene_id','task_id','variant_id','result_path','result_sha256'):
            require(current[key]==original[key],'Revision changed original task/planning identity: '+key)
    original_plan=read({'path':original['result_path'],'sha256':original['result_sha256']})
    composition=document['new_composition']
    require(composition['source_paths_sha256']==original_plan['composition']['source_paths_sha256'],'Revision replaced source assets')
    require(composition['layout_sha256']==document['new_config']['sha256'] and composition['scene_sha256']==document['new_source_scene']['sha256'],'Revision composition layout/stage digest differs')
    for path,value in composition['source_paths_sha256'].items():bound({'path':path,'sha256':value})
    return dict(document=document,old_config=old,new_config=new,original_row=original,original_plan=original_plan,
                old_source_scene=old_stage,new_source_scene=new_stage,changed_fields=changed)
