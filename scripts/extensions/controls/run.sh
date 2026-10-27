#!/usr/bin/env bash
# Explicitly forward headless mode into the configured Arena container.
set -euo pipefail
script_name=${1:?Usage: run.sh SCRIPT.py [arguments...]}
shift
case "$script_name" in
  inspect_runtime.py|preview_button_layout.py|render_visual_scene.py|collect_trial.py|audit_self_collision.py|paired_planning.py|probe_passive_knobs.py|probe_faucet.py|probe_neutral.py) ;;
  *) printf 'Unsupported controls GPU script: %s\n' "$script_name" >&2; exit 2 ;;
esac
docker exec "${ARENA_CONTAINER:-arena-kitchen-dev}" mkdir -p /workspace/reports/skill_trials
docker exec -i -w /workspace -e PYTHONPATH=/workspace -e HEADLESS=1 -e PYTHONUNBUFFERED=1 \
  "${ARENA_CONTAINER:-arena-kitchen-dev}" \
  flock /workspace/reports/skill_trials/gpu.lock \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  /workspace/.venv-arena/bin/python "scripts/extensions/controls/$script_name" "$@"
