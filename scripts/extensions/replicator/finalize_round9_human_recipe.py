#!/usr/bin/env python3
"""Finalize scope metadata after the bounded round9 human CPU checks."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round9_human_cpu import SID,CONFIG,OUT
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe

pack=read(CONFIG);r=pack['recipes'][SID]
evidence=read(geo.bound(r['evidence']))
assert evidence['passed'] and evidence['selected_name']=='vertical_then_reverse_to_prior_transfer'
r['cpu_screen_scope']={
    'lower_endpoint_with_full_robot_and_payload':True,
    'source_full_bowl_destination':True,
    'source_gripper_release_angle_samples':17,
    'retreat_endpoints':2,
    'ideal_retreat_pose_samples':22,
    'ideal_lower_segment_checked':False,
    'actual_curobo_path_checked':False,
    'physical_support_checked':False,
}
r['ideal_segment_samples_checked_phases']=['retreat']
r['remaining_cpu_warnings']=[
    'The lower endpoint uses one collision-free CPU IK branch; default cuRobo may choose a different branch.',
    'Transfer-to-lower continuous ideal or actual path was not checked; only the new lower endpoint and release geometry were checked.',
    'Retreat 22-pose screening is finite ideal geometry and does not guarantee the actual cuRobo curve or physical success.',
]
r['finalizer_source']=ref(Path(__file__))
old=read(geo.bound(r['previous_candidate_planning_result']))
original=read(geo.bound(read(OUT/'candidates.json')['source_original_planning']))
layout=read(geo.bound(dict(path=r['config_path'],sha256=r['config_sha256'])))
assert all(r['target_poses_world'][p]==old['targets'][p] for p in ['pregrasp','approach','lift','transfer'])
assert r['nominal_attachment']==old['nominal_attachment']
assert r['target_poses_world']['retreat']==old['targets']['transfer']
validate_recipe(r,original,layout)
save(CONFIG,pack)
print(ref(CONFIG))
