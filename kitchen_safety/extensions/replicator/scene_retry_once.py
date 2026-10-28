"""Validate the explicit one-change hazard-pose retry, retaining task geometry."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path


def validate_adjustment(root, recipe, previous_path=None, previous_sha=None):
    change = recipe.get('scene_adjustment')
    if not isinstance(change, dict) or change.get('adjustments') != 1:
        raise ValueError('Scene retry requires exactly one frozen adjustment')
    if recipe.get('layout_fixed') is not False or recipe.get('action') != 'plan':
        raise ValueError('Scene adjustment must explicitly declare one plan and changed layout')
    def read_bound(path_key, hash_key):
        path = (Path(root) / change[path_key]).resolve()
        path.relative_to(Path(root).resolve())
        if hashlib.sha256(path.read_bytes()).hexdigest() != change[hash_key]:
            raise ValueError('Scene adjustment input hash changed: '+path_key)
        return path, json.loads(path.read_text())
    old_path, old = read_bound('source_config_path', 'source_config_sha256')
    new_path, new = read_bound('config_path', 'config_sha256')
    if old_path == new_path or (previous_path is not None and old_path != (Path(root)/previous_path).resolve()):
        raise ValueError('Scene adjustment must retain the exact previous config separately')
    if previous_sha is not None and change['source_config_sha256'] != previous_sha:
        raise ValueError('Scene adjustment does not bind previous config')
    a, b = deepcopy(old), deepcopy(new)
    for config in (a, b):
        obstacle = config['obstacle']
        pos = obstacle.pop('position')
        yaw = float(obstacle.pop('yaw_deg', 0.))
        if (not isinstance(pos, (list, tuple)) or len(pos) != 3 or
                not all(isinstance(v, (int,float)) and math.isfinite(v) for v in pos) or not math.isfinite(yaw)):
            raise ValueError('Invalid hazard transform')
    if a != b:
        raise ValueError('Only added hazard position/yaw may change')
    if old['obstacle']['position'] == new['obstacle']['position'] and old['obstacle'].get('yaw_deg',0.) == new['obstacle'].get('yaw_deg',0.):
        raise ValueError('Scene adjustment is empty')
    if old['obstacle']['position'][2] != new['obstacle']['position'][2]:
        raise ValueError('Keep existing hazard support height')
    return change


def validate_round_authorization(root, manifest, source, original, failed, output):
    """A later explicit user request authorizes only this exact next round."""
    ref = manifest.get('scene_adjustment_authorization')
    if ref is None:
        return None
    path = (Path(root)/ref['path']).resolve()
    path.relative_to(Path(root).resolve())
    if hashlib.sha256(path.read_bytes()).hexdigest() != ref['sha256']:
        raise ValueError('Round authorization evidence changed')
    record = json.loads(path.read_text())
    if (record.get('source_run') != str(Path(source).resolve().relative_to(Path(root).resolve())) or
            record.get('output_run') != str(Path(output).resolve().relative_to(Path(root).resolve())) or
            record.get('source_progress_sha256') != hashlib.sha256((Path(source)/'progress.json').read_bytes()).hexdigest() or
            record.get('retry_round') != int(original.get('retry_round',0))+1 or
            record.get('source_retry_round') != original.get('retry_round') or
            set(record.get('scene_ids',[])) != set(failed) or
            record.get('max_scene_adjustments_per_scene_this_round') != 1 or
            record.get('max_planning_passes_per_scene_this_round') != 1 or
            not record.get('user_request') or original.get('stage') != 'complete'):
        raise ValueError('Authorization does not match this completed source and next round')
    return ref
