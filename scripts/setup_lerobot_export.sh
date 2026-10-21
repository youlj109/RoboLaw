#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
export_env=${LEROBOT_EXPORT_ENV:-$project_dir/.venv-lerobot-export}
if [[ -n "${LEROBOT_EXPORT_PYTHON:-}" ]]; then
  export_python=$LEROBOT_EXPORT_PYTHON
elif [[ -x "$project_dir/.venv-arena/bin/python" ]]; then
  export_python=$project_dir/.venv-arena/bin/python
else
  export_python=python3
fi
if [[ ! -x "$export_env/bin/python" ]]; then
  "$export_python" -m venv "$export_env"
fi
export_env=$(realpath "$export_env")
wheel_dir=${LEROBOT_EXPORT_WHEELS:-$(dirname "$export_env")/lerobot-export-wheels}
env -u PYTHONPATH "$export_env/bin/python" -m pip download --no-deps --timeout 120 \
  --index-url https://download.pytorch.org/whl/cpu --dest "$wheel_dir" \
  'torch==2.7.1+cpu' 'torchvision==0.22.1+cpu'
env -u PYTHONPATH "$export_env/bin/python" -m pip install --no-deps --no-index \
  --find-links "$wheel_dir" 'torch==2.7.1+cpu' 'torchvision==0.22.1+cpu'
env -u PYTHONPATH "$export_env/bin/python" -m pip install --no-cache-dir --timeout 120 \
  --index-url "${LEROBOT_PYPI_INDEX:-https://pypi.org/simple}" -r requirements-lerobot-export.txt
env -u PYTHONPATH "$export_env/bin/python" -c 'from lerobot.datasets.lerobot_dataset import LeRobotDataset; import torch; print("LeRobot v3 export environment ready; torch", torch.__version__)'
