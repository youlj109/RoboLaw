#!/usr/bin/env python3
"""Clarify existing raw ESDF metadata; no GPU call or acceptance change."""
from copy import deepcopy
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[3]

def binding(path):return dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())

def normalize_esdf(record):
    value=deepcopy(record)
    for sample in value['samples']:
        if 'max_signed_penetration_m' not in sample:continue
        sample['max_raw_signed_center_distance_m']=sample.pop('max_signed_penetration_m')
        sample['raw_positive_center_indices']=sample.pop('positive_signed_sphere_indices')
        for pair in sample['world_object_pairs']:
            center=pair.pop('gpu_signed_penetration_m');radius=pair['sphere_world_xyzr'][3]
            pair['gpu_signed_center_distance_m']=center
            pair['gpu_sphere_surface_clearance_m']=-center-radius
            pair['gpu_cpu_surface_clearance_difference_m']=pair['gpu_sphere_surface_clearance_m']-pair['cpu_fcl_surface_distance_m']
            pair['interpretation']='compute_esdf raw distance refers to sphere center; sphere surface clearance subtracts radius. Actual world/self constraints and zero-activation costs are separate collision gates.'
        sample['minimum_reported_pair_surface_clearance_m']=min((p['gpu_sphere_surface_clearance_m'] for p in sample['world_object_pairs']),default=None)
    if 'positive_signed_samples' in value['summaries']:
        value['summaries']['raw_positive_center_samples']=value['summaries'].pop('positive_signed_samples')
    value['esdf_scope']='Raw max may include zero sentinel/disabled spheres. Reported nearby pairs have radius-corrected surface clearance; actual constraints determine collision. Pair enumeration was limited to center distance <40mm and is not a complete nearest-surface search.'
    return value

if __name__=='__main__':
    path=ROOT/'reports/replicator_physical_retry_round9_20261029/cpu_curobo_static_agreement.json'
    backup=path.with_name('cpu_curobo_static_agreement.raw_center_draft.json')
    if backup.exists():raise FileExistsError(backup)
    backup.write_bytes(path.read_bytes());report=json.loads(backup.read_text())
    report['entries']=[normalize_esdf(e) for e in report['entries']]
    report.update(raw_query_report=binding(backup),esdf_semantics_postprocessor=binding(Path(__file__)),
        correction_scope='Metadata-only clarification of center versus surface distances. No q, FK value, world/self constraint, zero-activation cost, source geometry or success claim was changed. No new GPU calls.')
    path.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(binding(path)))
