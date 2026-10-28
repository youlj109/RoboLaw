import hashlib
import json

import pytest

from scripts.extensions.controls.prepare_visual_arm_revision import prepare


@pytest.fixture
def revision(tmp_path):
    asset = tmp_path / 'asset'
    asset.mkdir()
    origin = tmp_path / 'source.blend'
    origin.write_bytes(b'source')
    (asset / 'person.usdc').write_bytes(b'export')
    (asset / 'collidable.usda').write_bytes(b'wrapper')
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    pose = dict(source=str(origin), source_sha256=sha(origin), blender_version='test',
                method='test', recipe_args=['--wrist', '-.68', '.20', '1.15'],
                wrist_structure=dict(mode='follow-forearm'))
    audit = dict(passed=True, source_sha256=sha(origin), export_sha256=sha(asset / 'person.usdc'),
                 checks={key: True for key in ('six_exported_meshes', 'feet_unchanged',
                     'foot_vertex_count_unchanged', 'non_arm_bones_unchanged', 'wrist_target_reached',
                     'source_wrist_frame_preserved', 'source_wrist_bend_preserved')})
    (asset / 'pose.json').write_text(json.dumps(pose))
    (asset / 'rebuild_geometry_audit.json').write_text(json.dumps(audit))
    source = tmp_path / 'source.json'
    source.write_text(json.dumps(dict(task_id='original', original_target_level=3,
        robot_pose_xyzw=[1, 2, 3, 0, 0, 0, 1], variants=dict(
            human=dict(asset='old', position=[4, 5, 0], yaw_deg=-90),
            property=dict(asset='glass'), robot=dict(asset='barrier')))))
    return source, asset, tmp_path / 'new.json'


def test_revision_changes_only_human_asset_and_provenance(revision):
    source, asset, output = revision
    before = json.loads(source.read_text())
    after = prepare(source, asset, output)
    assert before['variants']['human']['position'] == after['variants']['human']['position']
    assert before['variants']['human']['yaw_deg'] == after['variants']['human']['yaw_deg']
    assert before['variants']['property'] == after['variants']['property']
    assert before['variants']['robot'] == after['variants']['robot']
    for key in ('task_id', 'robot_pose_xyzw', 'original_target_level'):
        assert before[key] == after[key]
    assert after['visual_revision']['accepted_for_collection'] is False
    manifest = json.loads((asset / 'artifact_manifest.json').read_text())
    assert str(asset / 'rebuild_geometry_audit.json') in manifest['generated_files']
    with pytest.raises(FileExistsError):
        prepare(source, asset, output)


@pytest.mark.parametrize('failure', ['feet', 'export', 'source'])
def test_stale_or_failed_geometry_audit_rejected(revision, failure):
    source, asset, output = revision
    path = asset / 'rebuild_geometry_audit.json'
    audit = json.loads(path.read_text())
    if failure == 'feet':
        audit['checks']['feet_unchanged'] = False
    else:
        audit[failure + '_sha256'] = 'wrong'
    path.write_text(json.dumps(audit))
    with pytest.raises(ValueError):
        prepare(source, asset, output)
    assert not output.exists()
    assert not (asset / 'artifact_manifest.json').exists()
