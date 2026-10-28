import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from kitchen_safety.extensions.controls.execution_contract import (
    controller_signature, frozen_task_differences)


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'configs/extensions/controls/benchmark_round180/07_oven_temperature_human.json'
SCRIPT = ROOT / 'scripts/extensions/controls/declare_axial_candidate.py'


def declare(output, *extra):
    return subprocess.run([sys.executable, str(SCRIPT), '--source', str(SOURCE),
                           '--round', '190', '--angle-deg', '0', '--output', str(output),
                           *extra], capture_output=True, text=True)


def test_candidate_retains_frozen_scene_and_original_goal(tmp_path):
    output = tmp_path / 'candidate.json'
    assert declare(output, '--tilt-deg', '-45').returncode == 0
    before = json.loads(SOURCE.read_text())
    after = json.loads(output.read_text())
    assert not frozen_task_differences(before, after)
    assert before['variants'] == after['variants']
    assert before['grasp_tcp_world_m'] == after['grasp_tcp_world_m']
    assert before['friction_connection_world_m'] == after['friction_connection_world_m']
    assert controller_signature(before) != controller_signature(after)
    matrix = np.asarray(after['grasp_rotation_world'])
    assert np.allclose(matrix.T @ matrix, np.eye(3))
    assert np.linalg.det(matrix) == pytest.approx(1.)
    original = output.read_bytes()
    assert declare(output).returncode != 0
    assert output.read_bytes() == original


@pytest.mark.parametrize('extra', [('--tilt-deg', '90'), ('--tilt-deg', 'nan'),
                                   ('--angle-deg', '181'), ('--angle-deg', 'nan'),
                                   ('--tool-roll-deg', '181'), ('--tool-roll-deg', 'nan'),
                                   ('--phase-deg', '181'), ('--phase-deg', 'nan'),
                                   ('--contact-offset-m', '.01'),
                                   ('--contact-offset-m', '-.03')])
def test_invalid_candidate_is_not_written(tmp_path, extra):
    output = tmp_path / 'candidate.json'
    assert declare(output, *extra).returncode != 0
    assert not output.exists()


def test_contact_offset_is_explicit_and_bound_to_controller(tmp_path):
    output = tmp_path / 'candidate.json'
    assert declare(output, '--contact-offset-m', '-.005').returncode == 0
    before = json.loads(SOURCE.read_text())
    after = json.loads(output.read_text())
    assert not frozen_task_differences(before, after)
    assert after['benchmark_execution_round']['contact_start_offset_m'] == -.005
    assert after['friction_start_x_offset_m'] == -.005
    assert controller_signature(before) != controller_signature(after)
