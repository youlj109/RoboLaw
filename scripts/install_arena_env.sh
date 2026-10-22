#!/usr/bin/env bash
# Install into the running project container; safe to repeat without changing
# the checked-in Arena lock or silently removing the separate cuRobo install.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
arena_container=${ARENA_CONTAINER:-arena-kitchen-dev}
docker exec -i -w /workspace \
  -e UV_PROJECT_ENVIRONMENT=/workspace/.venv-arena \
  -e UV_LINK_MODE=copy \
  -e "UV_HTTP_TIMEOUT=${UV_HTTP_TIMEOUT:-300}" \
  -e "UV_HTTP_RETRIES=${UV_HTTP_RETRIES:-5}" \
  -e "UV_CONCURRENT_DOWNLOADS=${UV_CONCURRENT_DOWNLOADS:-4}" "$arena_container" \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER bash -eu -c '
    uv python install 3.12.13
    # Keep the client dependency graph from uv.lock, but avoid recursively
    # cloning unrelated OpenPI simulator submodules for this small client.
    uv sync --project third_party/IsaacLab-Arena --python 3.12.13 --locked --extra dev --inexact --no-install-package openpi-client
    uv pip install --python /workspace/.venv-arena/bin/python --no-deps \
      "openpi-client @ https://github.com/Physical-Intelligence/openpi/archive/c23745b5ad24e98f66967ea795a07b2588ed6c79.tar.gz#subdirectory=packages/openpi-client"
    uv pip install --python /workspace/.venv-arena/bin/python --no-deps -r requirements-collection.txt
    uv pip install --python /workspace/.venv-arena/bin/python -r requirements-gdist.txt
    /workspace/.venv-arena/bin/python scripts/build_gdist_backend.py
    mkdir -p reports
  '
bash scripts/arena_python.sh scripts/restore_downloaded_assets.py
bash scripts/install_curobo.sh
bash scripts/arena_python.sh scripts/check_runtime.py
