"""Source-bound local grasp targets for independent additional mustard runs."""
import json
from pathlib import Path
import numpy as np
from .mustard_family_preparation import ROOT,load_preparation,sha,TASKS
from .task_motion import pose_matrix


def read_controller_inputs(path):
    """JSON and hash checks only, safe before SimulationApp selects its USD ABI.

    This does not authorize a scene. load_controller must still re-evaluate all
    original relations, source geometry and profile gates after Kit starts.
    """
    controller=json.loads(Path(path).read_text())
    if controller['schema_version']!=1 or controller['task_id'] not in TASKS:
        raise ValueError('Unsupported additional mustard controller')
    for item in controller['inputs'].values():
        if sha(ROOT/item['path'])!=item['sha256']:raise ValueError('Changed controller evidence '+item['path'])
    prepared=json.loads((ROOT/controller['inputs']['preparation']['path']).read_text())
    if prepared['task_id']!=controller['task_id']:raise ValueError('Preparation/controller task mismatch')
    for item in prepared['evidence']['input_hashes'].values():
        if sha(ROOT/item['path'])!=item['sha256']:raise ValueError('Changed preparation input '+item['path'])
    return controller,prepared


def load_controller(path):
    controller,_=read_controller_inputs(path)
    prepared=load_preparation(ROOT/controller['inputs']['preparation']['path'])
    if prepared['task_id']!=controller['task_id']:raise ValueError('Preparation/controller task mismatch')
    if controller['source_layout_sha256']!=prepared['evidence']['input_hashes']['layout']['sha256']:
        raise ValueError('Controller source layout mismatch')
    if controller['source_kitchen_sha256']!=sha(ROOT/prepared['layout']['kitchen_usd']):raise ValueError('Source kitchen changed')
    for name,cfg in prepared['layout']['objects'].items():
        if controller['source_object_sha256'][name]!=sha(ROOT/cfg['usd_path']):raise ValueError('Source task object changed')
    expected=dict(pregrasp_height_m=.125,lift_height_m=.145,release_gap_m=.003,transport_guard_above_rim_m=.05)
    if controller['motion']!=expected or controller['clearances']!=prepared['profile']['clearances']:
        raise ValueError('Unreviewed motion or safety policy')
    local=np.asarray(controller['bottle_to_tcp_local'])
    if (local.shape!=(4,4) or not np.isfinite(local).all() or not np.allclose(local[3],[0,0,0,1])
            or not np.allclose(local[:3,:3].T@local[:3,:3],np.eye(3),atol=1e-10) or np.linalg.det(local[:3,:3])<0):
        raise ValueError('Invalid declared local grasp transform')
    return controller,prepared


def grasp_target(observed_bottle,profile,prepared):
    target=pose_matrix(observed_bottle)@np.asarray(profile['bottle_to_tcp_local'])
    geometry=prepared['profile']['source_bottle_geometry'];low,high=map(np.array,geometry['mesh_bounds_m'])
    low[2],high[2]=geometry['safe_contact_z_range_m']
    return target,dict(bottle_to_tcp_local=profile['bottle_to_tcp_local'],target_tcp_world_m=target[:3,3].tolist(),
        contact_region_local_m=[low.tolist(),high.tolist()],selection=profile['selection'],
        observed_pose_binding=True,physical_contact_verified=False)


def transport_target(state,attachment,payload_index,bowl_index,payload_geometry,bowl_geometry,guard):
    """Raise the carried source bottom above the observed bowl source rim."""
    if guard!=.05:raise ValueError('Keep the declared 50mm transport gap above source bowl rim')
    bottle=pose_matrix(state['task_object_states'][payload_index,:7]);bowl=pose_matrix(state['task_object_states'][bowl_index,:7])
    bottom=float((payload_geometry.vertices@bottle[:3,:3].T)[:,2].min())
    rim=float((bowl_geometry.vertices@bowl[:3,:3].T+bowl[:3,3])[:,2].max())
    desired=bottle.copy();desired[:2,3]=bowl[:2,3];desired[2,3]=max(bottle[2,3],rim+guard-bottom)
    return desired@np.linalg.inv(attachment),dict(source_bowl_rim_world_z_m=rim,
        carried_bottle_bottom_relative_root_z_m=bottom,source_vertical_gap_m=guard,
        desired_bottle_pose_world=desired.tolist(),measured_attachment_used=True)


def require_neutral(path,controller_path,task_id):
    result=json.loads((Path(path)/'result.json').read_text())
    if (result['task_id']!=task_id or result.get('controller_profile_sha256')!=sha(controller_path)
            or result.get('neutral_probe_passed') is not True or result.get('neutral_hold_seconds',0)<45.):
        raise ValueError('A matching actual family neutral probe of at least 45s must pass before a physical task trial')
    return dict(path=str(path),result_sha256=sha(Path(path)/'result.json'),duration_s=result['neutral_hold_seconds'],
                physical_duration_matching_pending=True)
