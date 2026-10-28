import json
from pathlib import Path

import pytest

from scripts.extensions.controls.declare_fixture_visual_revision import declare
from scripts.extensions.controls.update_visual_review import render_views


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'configs/extensions/controls/visual_20261027/03_left_obstacle_r10.json'


def test_pose_revision_retains_assets_original_task_and_other_variants(tmp_path):
    output = tmp_path / 'new.json'
    before = json.loads(SOURCE.read_text())
    after = declare(SOURCE, output, [1.56, -.48, .921], 90., 'visual only')
    assert before['variants']['human'] == after['variants']['human']
    assert before['variants']['property'] == after['variants']['property']
    assert before['variants']['robot']['asset'] == after['variants']['robot']['asset']
    assert after['variants']['robot']['position'] == [1.56, -.48, .921]
    assert after['visual_revision']['accepted_for_collection'] is False
    for key in ('task_id', 'source_sha256', 'robot_pose_xyzw', 'button_joint',
                'prepress_planning', 'fixture_revision'):
        assert before[key] == after[key]
    content = output.read_bytes()
    with pytest.raises(FileExistsError):
        declare(SOURCE, output, [1., 2., 3.], 0., 'do not overwrite')
    assert output.read_bytes() == content


@pytest.mark.parametrize('position,yaw', [([1., 2.], 0.), ([1., 2., float('nan')], 0.),
                                        ([1., 2., 3.], float('inf'))])
def test_invalid_visual_pose_is_not_written(tmp_path, position, yaw):
    output = tmp_path / 'invalid.json'
    with pytest.raises(ValueError):
        declare(SOURCE, output, position, yaw, 'invalid')
    assert not output.exists()


def test_custom_render_views_are_in_gallery_once_and_main_last(tmp_path):
    (tmp_path / 'render_metadata.json').write_text(json.dumps(dict(review_camera_only={
        'wide': [], 'side': [], 'fixture_top': [], 'fixture_front': [], 'fixture_detail': []})))
    views = render_views(tmp_path, 'fixture_detail')
    assert views[-1] == 'fixture_detail'
    assert len(views) == len(set(views))
    assert {'fixture_top', 'fixture_front', 'external_camera_2'} <= set(views)


def test_legacy_gallery_without_metadata_still_has_main_last(tmp_path):
    views = render_views(tmp_path, 'external_camera')
    assert views[-1] == 'external_camera'
    assert len(views) == len(set(views))
