"""Read-only round12 retention gate; only the unresolved human is selected."""
import hashlib
import json
from collections import Counter
from pathlib import Path
from scripts.extensions.replicator.retained_scene_revision_review_round11 import validate_retained_scene_revision as validate_round11_retention
from scripts.extensions.replicator.scene_revision_review_round8 import SOURCE_KEYS

PREVIOUS_COHORT_SIZE = 2


def validate_retained_scene_revision(root,reference,previous_manifest,*,expected_scene_id=None,current=None,digest=None):
    root=Path(root).resolve()
    def require(value,message):
        if not value:raise ValueError(message)
    def local(value):
        path=(root/str(value).removeprefix('/workspace/')).resolve()
        require(path.is_relative_to(root),'Retained layout path outside checkout');return path
    def load(binding):
        path=local(binding['path'])
        if digest:actual=digest(path)
        else:
            with path.open('rb') as stream:actual=hashlib.file_digest(stream,'sha256').hexdigest()
        require(actual==binding['sha256'],'Retained round12 prior source SHA differs')
        return json.loads(path.read_text())
    previous=load(previous_manifest);rows=previous['entries'];indexed={r['scene_id']:r for r in rows}
    require(len(indexed)==len(rows)==PREVIOUS_COHORT_SIZE and previous.get('retry_round')==11 and previous.get('stage')=='complete'
            and Counter(r['execution_status'] for r in rows)=={'success':1,'planning_failed':1},
            'Retained round12 needs completed round11 with one native success and one planning failure')
    for row in rows:
        if row['execution_status']=='success':
            result=load(dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']))
            require(row.get('planning_status')=='generated' and row.get('attempts')==1 and row.get('executed') is True and row.get('task_success') is True
                    and result.get('status')=='complete' and result.get('execution_status')=='success'
                    and result.get('executed') is True and result.get('task_success') is True and result.get('attempts')==1,
                    'Round11 successful native episode is not bound')
            for key in ('scene_id','task_id','variant_id','config_path','config_sha256','trajectory_path','trajectory_sha256','scene_revision','retry_plan_path','retry_plan_sha256'):
                require(result.get(key)==row.get(key),'Round11 successful result differs from its manifest: '+key)
        else:
            require(row.get('planning_status')=='failed' and row.get('attempts')==0 and row.get('executed') is False
                    and row.get('task_success') is False and not row.get('execution_result_path') and not row.get('execution_result_sha256'),
                    'Round11 failed planning invented an actual episode')
    sid=expected_scene_id
    require(sid in indexed and indexed[sid]['execution_status']=='planning_failed','Round12 must select the single unresolved scene, never the previous success')
    prior=indexed[sid]
    require(prior.get('scene_revision')==reference,'Round12 changed selected revision')
    checked=validate_round11_retention(root,reference,previous['source_previous_retry_manifest'],
        expected_scene_id=sid,current=prior,digest=digest)
    if current is not None:
        for key in SOURCE_KEYS:
            require(current.get(key)==prior.get(key),'Retained round12 changed a selected scene source: '+key)
        for key,value in dict(layout_changed=True,scene_settings_changed_this_round=False,retained_scene_revision=True,scene_settings_changed_once=True).items():
            require(current.get(key) is value,'Retained round12 layout scope differs: '+key)
    return dict(checked,retained_from_manifest=previous_manifest,retained_from_row=prior,
        previous_retention_manifest=previous['source_previous_retry_manifest'])
