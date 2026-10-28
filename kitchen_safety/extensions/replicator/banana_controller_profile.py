"""Source-bound G banana control candidate; never a physical-success claim."""
import hashlib
import json
from pathlib import Path

from .task_motion import top_down_grasp

ROOT = Path(__file__).resolve().parents[3]
TASK = 'kitchen_bench_replicator_g_shape_banana_bowl'
LAYOUT = ROOT/'configs/extensions/replicator/g_shape_banana_bowl_source_layout.json'
PROFILE = ROOT/'configs/extensions/replicator/g_shape_banana_bowl_controller.json'


def read_profile(path=PROFILE, layout_path=LAYOUT):
    path, layout_path = Path(path), Path(layout_path)
    profile, layout = json.loads(path.read_text()), json.loads(layout_path.read_text())
    digest = lambda item: hashlib.sha256(Path(item).read_bytes()).hexdigest()
    if profile['schema_version'] != 1 or profile['task_id'] != TASK or layout['task_id'] != TASK:
        raise ValueError('Unsupported source banana control profile')
    if profile['source_layout_sha256'] != digest(layout_path) or profile['source_kitchen_sha256'] != digest(ROOT/layout['kitchen_usd']):
        raise ValueError('Source layout/kitchen changed; do not silently reuse previous candidate')
    if set(profile['source_object_sha256']) != set(layout['objects']):
        raise ValueError('Source task object inventory changed')
    if any(profile['source_object_sha256'][name] != digest(ROOT/item['usd_path']) for name, item in layout['objects'].items()):
        raise ValueError('Source object asset changed')
    expected = dict(grasp_height_offset_m=.013, pregrasp_height_m=.125, lift_height_m=.15,
        transport_destination_raise_m=.20, release_gap_m=.003)
    if any(profile[key] != value for key, value in expected.items()) or profile['baseline_and_variants_share_profile'] is not True:
        raise ValueError('Unreviewed source grasp/motion profile')
    return profile


def grasp_target(actual_payload_pose, geometry, world, profile):
    if world.payload_name != 'banana' or geometry.provenance['source_sha256'] != profile['source_object_sha256']['banana']:
        raise ValueError('Runtime payload differs from the prepared original banana')
    target, info = top_down_grasp(actual_payload_pose, geometry, world)
    original_height = float(target[2, 3])
    target[2, 3] += profile['grasp_height_offset_m']
    info.update(original_open_envelope_tcp_z_m=original_height,
        additional_closed_gripper_clearance_height_m=profile['grasp_height_offset_m'],
        target_tcp_world_m=target[:3, 3].tolist(),
        physical_contact_verified=False,
        selection=profile['selection'], full_original_noncontact_checks_required=True)
    return target, info
