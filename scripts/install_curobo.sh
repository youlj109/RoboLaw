#!/usr/bin/env bash
# Install Arena's pinned cuRobo into the already-created .venv-arena container.
# Existing Torch, Isaac Sim, Isaac Lab, NumPy and Warp packages are preserved.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
arena_container=${ARENA_CONTAINER:-arena-kitchen-dev}
if ! docker exec -w /workspace "$arena_container" test -x /usr/local/cuda-12.8/bin/nvcc; then
  docker exec -u root -w /tmp "$arena_container" bash -eu -c '
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb
    dpkg -i /tmp/cuda-keyring.deb
    apt-get update -qq
    apt-get install -y --no-install-recommends cuda-nvcc-12-8 cuda-cudart-dev-12-8
  '
fi
docker exec -w /workspace \
  -e "UV_HTTP_TIMEOUT=${UV_HTTP_TIMEOUT:-300}" \
  -e "UV_HTTP_RETRIES=${UV_HTTP_RETRIES:-5}" \
  -e "UV_CONCURRENT_DOWNLOADS=${UV_CONCURRENT_DOWNLOADS:-4}" "$arena_container" uv pip install \
  --python /workspace/.venv-arena/bin/python --no-deps \
  -r /workspace/runtime/requirements-curobo.txt

# Use the same pinned source and DROID asset files for either download route.
python3 scripts/fetch_source_dependencies.py --components curobo
python3 scripts/restore_downloaded_assets.py --source "${ROBOLAW_DOWNLOAD_SOURCE:-official}"
bash scripts/build_curobo.sh
bash scripts/arena_python.sh scripts/smoke_curobo.py
