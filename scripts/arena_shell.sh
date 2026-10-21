#!/usr/bin/env bash
set -euo pipefail
docker exec -it -w /workspace -e PYTHONPATH=/workspace "${ARENA_CONTAINER:-arena-kitchen-dev}" \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  bash --rcfile /workspace/scripts/arena_shell.rc
