import json
from pathlib import Path
import subprocess
import sys

import pytest

from kitchen_safety.extensions.controls.execution_contract import (
    controller_signature, frozen_task_differences)


ROOT = Path(__file__).resolve().parents[3]
CONTROLLER = ROOT / 'configs/extensions/controls/benchmark_round154/07_oven_temperature_human.json'
SCRIPT = ROOT / 'scripts/extensions/controls/bind_visual_controller.py'


def invoke(visual, output):
    return subprocess.run([sys.executable, str(SCRIPT), '--visual', str(visual),
        '--controller', str(CONTROLLER), '--output-dir', str(output), '--round', '195'],
        capture_output=True, text=True)


def visual_config(tmp_path):
    config = json.loads(CONTROLLER.read_text())
    config['variants']['human']['asset'] = 'assets/extensions/controls/new-human/collidable.usda'
    config['human_pose_revision'] = dict(manifest='new-manifest', manifest_sha256='hash')
    config['visual_revision'] = dict(scope='unaccepted test fixture')
    path = tmp_path / 'visual.json'
    path.write_text(json.dumps(config))
    return path, config


def test_rebinding_retains_controller_and_new_layout(tmp_path):
    path, visual = visual_config(tmp_path)
    output = tmp_path / 'candidate'
    result = invoke(path, output)
    assert result.returncode == 0, result.stderr
    candidate = json.loads((output / '07_oven_temperature_human.json').read_text())
    assert not frozen_task_differences(visual, candidate)
    assert controller_signature(candidate) == controller_signature(json.loads(CONTROLLER.read_text()))
    assert candidate['variants'] == visual['variants']
    assert candidate['human_pose_revision'] == visual['human_pose_revision']
    assert invoke(path, output).returncode != 0


@pytest.mark.parametrize('field,value', [('robot_pose_xyzw', [0, 0, 0, 0, 0, 0, 1]),
                                        ('original_target_level', 2), ('task_id', 'different')])
def test_rebinding_rejects_changed_original_task(tmp_path, field, value):
    path, visual = visual_config(tmp_path)
    visual[field] = value
    path.write_text(json.dumps(visual))
    output = tmp_path / 'candidate'
    assert invoke(path, output).returncode != 0
    assert not output.exists()
