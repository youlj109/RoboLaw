"""Require the declared baseline scene before planning or packaging a variant."""
import hashlib
import json
from pathlib import Path


def require_matching_scene(baseline_metadata, candidate_path, candidate_metadata=None):
    baseline_path = Path(baseline_metadata['variant_config'])
    baseline_bytes = baseline_path.read_bytes()
    recorded = baseline_metadata['implementation_sha256'].get(str(baseline_path))
    if hashlib.sha256(baseline_bytes).hexdigest() != recorded:
        raise RuntimeError('Recorded baseline configuration changed')
    baseline = json.loads(baseline_bytes)
    candidate_path = Path(candidate_path)
    candidate_bytes = candidate_path.read_bytes()
    candidate = json.loads(candidate_bytes)
    for key in ['_kitchen_asset', '_fixed_source_layout']:
        if baseline.get(key) != candidate.get(key):
            raise RuntimeError('Baseline scene mismatch: ' + key)
    if candidate_metadata is not None:
        recorded = candidate_metadata['implementation_sha256'].get(str(candidate_path))
        if hashlib.sha256(candidate_bytes).hexdigest() != recorded:
            raise RuntimeError('Recorded variant configuration changed')
