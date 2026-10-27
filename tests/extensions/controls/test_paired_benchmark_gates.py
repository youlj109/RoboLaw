"""Paired coverage is explicit; an empty or incomplete run cannot pass."""
import hashlib
import json
import pytest
from scripts.extensions.controls import paired_planning as module
from scripts.extensions.controls.paired_planning import paired_gates


def row(seed, variant):
    return dict(seed=seed,variant=variant,success=True,
        true_shape_counterfactual=dict(violations=[dict(valid=False)],robust_violation=True,initial_and_goal_safe=True),
        true_shape_actual=dict(samples=3,violations=[]))


def test_empty_pair_cannot_pass():
    assert not any(paired_gates([], [42], ['base','human']).values())


def test_missing_seed_or_duplicate_pair_cannot_pass():
    rows=[row(42,'base'),row(42,'human')]
    assert not paired_gates(rows,[42,43],['base','human'])['complete_pairs']
    assert not paired_gates(rows+[rows[0]], [42], ['base','human'])['complete_pairs']


def test_base_only_is_not_obstacle_necessity():
    gates=paired_gates([row(42,'base')],[42],['base'])
    assert gates['complete_pairs']
    assert not gates['all_hazard_off_paths_require_adjustment']


def test_safe_hazard_off_or_unsafe_hazard_on_is_rejected():
    rows=[row(42,'base'),row(42,'human')]
    assert all(paired_gates(rows,[42],['base','human']).values())
    rows[1]['true_shape_counterfactual']['violations']=[]
    rows[1]['true_shape_actual']['violations']=[dict(valid=False)]
    gates=paired_gates(rows,[42],['base','human'])
    assert not gates['all_hazard_off_paths_require_adjustment']
    assert not gates['all_hazard_on_source_shapes_safe']


def test_paired_human_manifest_cannot_silently_change(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'ROOT',tmp_path)
    asset=tmp_path/'human';asset.mkdir()
    payload=asset/'person.usdc';payload.write_bytes(b'fixture')
    manifest=asset/'artifact_manifest.json'
    manifest.write_text(json.dumps(dict(generated_files={'human/person.usdc':hashlib.sha256(payload.read_bytes()).hexdigest()})))
    config=dict(variants=dict(human=dict(asset='human/collidable.usda')),
        human_pose_revision=dict(manifest='human/artifact_manifest.json',
            manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest()))
    assert module.verify_asset_manifests(config)['human']['passed']
    manifest.write_text(manifest.read_text()+'\n')
    with pytest.raises(ValueError,match='manifest hash differs'):
        module.verify_asset_manifests(config)
