"""Read-only round11 retention gate; round10 failures did not create episodes."""
import hashlib
import json
from pathlib import Path
from scripts.extensions.replicator.retained_scene_revision_review_round10 import validate_retained_scene_revision as validate_round10_retention
from scripts.extensions.replicator.scene_revision_review_round8 import SOURCE_KEYS

COHORT_SIZE = 2


def validate_retained_scene_revision(root,reference,previous_manifest,*,expected_scene_id=None,current=None,digest=None):
    root=Path(root).resolve()
    def require(value,message):
        if not value:raise ValueError(message)
    def local(value):
        path=(root/str(value).removeprefix('/workspace/')).resolve()
        require(path.is_relative_to(root),'Retained layout path outside checkout');return path
    path=local(previous_manifest['path'])
    if digest:actual=digest(path)
    else:
        with path.open('rb') as stream:actual=hashlib.file_digest(stream,'sha256').hexdigest()
    require(actual==previous_manifest['sha256'],'Retained round11 prior manifest SHA differs')
    previous=json.loads(path.read_text());rows=previous['entries'];indexed={r['scene_id']:r for r in rows}
    require(len(indexed)==len(rows)==COHORT_SIZE and previous.get('retry_round')==10 and previous.get('stage')=='complete',
            'Retained round11 needs completed two-scene round10')
    for row in rows:
        require(row.get('execution_status')=='planning_failed' and row.get('planning_status')=='failed'
                and row.get('attempts')==0 and row.get('executed') is False and row.get('task_success') is False
                and not row.get('execution_result_path') and not row.get('execution_result_sha256'),
                'Round10 failed planning invented an actual episode')
    sid=expected_scene_id
    require(sid in indexed,'Retained round11 scene missing from prior cohort')
    prior=indexed[sid]
    require(prior.get('scene_revision')==reference,'Round11 changed selected revision')
    checked=validate_round10_retention(root,reference,previous['source_previous_retry_manifest'],
        expected_scene_id=sid,current=prior,digest=digest)
    if current is not None:
        for key in SOURCE_KEYS:
            require(current.get(key)==prior.get(key),'Retained round11 changed a selected scene source: '+key)
        for key,value in dict(layout_changed=True,scene_settings_changed_this_round=False,retained_scene_revision=True,scene_settings_changed_once=True).items():
            require(current.get(key) is value,'Retained round11 layout scope differs: '+key)
    return dict(checked,retained_from_manifest=previous_manifest,retained_from_row=prior,
        previous_retention_manifest=previous['source_previous_retry_manifest'])
