import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT/'scripts/extensions/controls/collect_repaired_left.py'


def test_host_absolute_output_maps_to_container_relative_path(tmp_path):
    output = ROOT/'datasets'/('dry_run_' + tmp_path.name)
    result = subprocess.run([sys.executable, str(SCRIPT), '--output', str(output), '--dry-run'],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    commands = [json.loads(line.split(' ', 1)[1]) for line in result.stdout.splitlines()]
    assert len(commands) == 4
    for command in commands:
        assert not Path(command[command.index('--output')+1]).is_absolute()
    assert not output.exists()


def test_output_outside_mount_is_rejected(tmp_path):
    result = subprocess.run([sys.executable, str(SCRIPT), '--output', str(tmp_path), '--dry-run'],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert 'mounted at /workspace' in result.stderr
