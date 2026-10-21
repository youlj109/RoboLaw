#!/usr/bin/env bash
set -euo pipefail
docker exec -i -w /workspace -e PYTHONPATH=/workspace "${ARENA_CONTAINER:-arena-kitchen-dev}" \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  /workspace/.venv-arena/bin/python "$@"
