"""Read-only binding gate for round9 retaining the exact selected round8 layout."""
import hashlib
import json
from pathlib import Path
from scripts.extensions.replicator.scene_revision_review_round8 import validate_scene_revision,SOURCE_KEYS


def validate_retained_scene_revision(root,reference,previous_manifest,*,expected_scene_id=None,current=None,digest=None):
    root=Path(root).resolve()
    def require(value,message):
        if not value:raise ValueError(message)
    def local(value):
        path=(root/str(value).removeprefix('/workspace/')).resolve()
        require(path.is_relative_to(root),'Retained layout path outside checkout');return path
    path=local(previous_manifest['path'])
    with path.open('rb') as stream:actual=digest(path) if digest else hashlib.file_digest(stream,'sha256').hexdigest()
    require(actual==previous_manifest['sha256'],'Retained layout previous manifest SHA differs')
    previous=json.loads(path.read_text());rows=previous['entries'];indexed={r['scene_id']:r for r in rows}
    require(len(indexed)==len(rows) and previous.get('retry_round')==8 and previous.get('stage')=='complete','Retained layout needs completed round8')
    checked=validate_scene_revision(root,reference,expected_scene_id=expected_scene_id,current=current,digest=digest)
    sid=checked['document']['scene_id'];prior=indexed[sid]
    require(prior.get('scene_revision')==reference and prior.get('layout_changed') is True,'Retained layout does not match round8 selected revision')
    status=prior.get('execution_status')
    require(status in ('failed','planning_failed') and prior.get('task_success') is False,'Retained layout scene is not unresolved')
    if status=='planning_failed':
        require(prior.get('attempts')==0 and prior.get('executed') is False and not prior.get('execution_result_path'),'Prior failed planning invented an episode')
    else:
        require(prior.get('attempts')==1 and prior.get('executed') is True and prior.get('execution_result_path') and prior.get('execution_result_sha256'),'Prior actual failure lacks its episode')
    if current is not None:
        for key in SOURCE_KEYS:
            require(current.get(key)==prior.get(key),'Retained round9 changed a selected scene source: '+key)
        for key,value in dict(layout_changed=True,scene_settings_changed_this_round=False,retained_scene_revision=True,scene_settings_changed_once=True).items():
            require(current.get(key) is value,'Retained layout scope differs: '+key)
    return dict(checked,retained_scene_revision=True,scene_settings_changed_this_round=False,
        retained_from_manifest=previous_manifest,retained_from_row=prior)
