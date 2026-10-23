#!/usr/bin/env bash
# Host (no-Docker) bounded GPU/Kit entry for lime scripts.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
script_name=${1:?Usage: run_host.sh SCRIPT.py [arguments...]}
shift
case "$script_name" in
  collect.py|inspect_runtime.py|audit_self_collision.py|check_lift_plan.py|probe_overview.py|probe_transfer_gpu.py|probe_pregrasp_gpu.py) ;;
  *) printf 'Unsupported lime GPU entry: %s\n' "$script_name" >&2; exit 2 ;;
esac
PY=${PY:-/root/autodl-tmp/conda_envs/robolaw_data/bin/python}
mkdir -p "$ROOT/reports/skill_trials"
LOCK=${GPU_LOCK:-$ROOT/reports/skill_trials/gpu.lock}
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
export HEADLESS=${HEADLESS:-1}
export PYTHONUNBUFFERED=1
# --- Host Vulkan/render path (AutoDL): avoid black/gray RTX camera frames ---
# NVIDIA ICD must win over mesa; disable Optimus/present layers that confuse headless.
if [[ -f /etc/vulkan/icd.d/nvidia_icd.json ]]; then
  export VK_ICD_FILENAMES=${VK_ICD_FILENAMES:-/etc/vulkan/icd.d/nvidia_icd.json}
fi
export DISABLE_LAYER_NV_OPTIMUS_1=${DISABLE_LAYER_NV_OPTIMUS_1:-1}
export DISABLE_LAYER_NV_PRESENT_1=${DISABLE_LAYER_NV_PRESENT_1:-1}
export VK_LOADER_LAYERS_DISABLE=${VK_LOADER_LAYERS_DISABLE:-VK_LAYER_MESA_device_select}
# Ensure GLVND EGL dispatcher exists (ICD negotiate fails hard without libEGL.so.1).
if [[ ! -e /usr/lib/x86_64-linux-gnu/libEGL.so.1 ]]; then
  printf 'WARN: libEGL.so.1 missing; apt-get install -y libegl1\n' >&2
fi
# Container GPU minor may be nvidiaN (not nvidia0); modeset needed for Vulkan init.
if [[ ! -e /dev/nvidia0 ]]; then
  for n in /dev/nvidia[0-9]*; do
    [[ -e "$n" ]] || continue
    ln -sfn "$(basename "$n")" /dev/nvidia0 2>/dev/null || true
    break
  done
fi
if [[ ! -e /dev/nvidia-modeset ]]; then
  mknod -m 666 /dev/nvidia-modeset c 195 254 2>/dev/null || true
fi
# Avoid plugin-path poisoning for USD crate opens in child processes.
unset PXR_PLUGINPATH_NAME USD_PLUGIN_PATH || true
cd "$ROOT"
exec flock "$LOCK" timeout "${LIME_EPISODE_TIMEOUT_S:-1200}" \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER -u PXR_PLUGINPATH_NAME \
  "$PY" "scripts/extensions/lightwheel_pick_place/$script_name" "$@"
