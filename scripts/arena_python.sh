#!/usr/bin/env bash
set -euo pipefail
workspace_root="${ROBOLAW_WORKSPACE:-/workspace}"
docker exec -i -w "$workspace_root" -e PYTHONPATH="$workspace_root" \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 -e MKL_NUM_THREADS=1 "${ARENA_CONTAINER:-arena-kitchen-dev}" \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  /workspace/.venv-arena/bin/python "$@"
