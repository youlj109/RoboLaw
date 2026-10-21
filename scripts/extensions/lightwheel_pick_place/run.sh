#!/usr/bin/env bash
# Run a single bounded GPU/Kit invocation with the shared workspace lock.
set -euo pipefail
script_name=${1:?Usage: run.sh SCRIPT.py [arguments...]}
shift
case "$script_name" in
  collect.py|inspect_runtime.py|audit_self_collision.py|check_lift_plan.py|probe_overview.py|probe_transfer_gpu.py|probe_pregrasp_gpu.py) ;;
  *) printf 'Unsupported lime GPU entry: %s\n' "$script_name" >&2; exit 2 ;;
esac
docker exec "${ARENA_CONTAINER:-arena-kitchen-dev}" mkdir -p /workspace/reports/skill_trials
exec docker exec -i -w /workspace -e PYTHONPATH=/workspace -e HEADLESS=1 -e PYTHONUNBUFFERED=1 \
  "${ARENA_CONTAINER:-arena-kitchen-dev}" flock /workspace/reports/skill_trials/gpu.lock \
  timeout "${LIME_EPISODE_TIMEOUT_S:-600}" env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  /workspace/.venv-arena/bin/python "scripts/extensions/lightwheel_pick_place/$script_name" "$@"
