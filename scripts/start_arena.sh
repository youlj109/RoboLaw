#!/usr/bin/env bash
# Build the system image if needed, then start the project's persistent runtime.
# The download must contain the Arena and IsaacLab source directories, not a
# copied virtual environment. Host Docker + NVIDIA Container Toolkit are needed.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
arena_container=${ARENA_CONTAINER:-arena-kitchen-dev}
arena_image=${ARENA_IMAGE:-arena-kitchen-runtime:2026-10-17}
if [[ ${1:-} == --accept-eula ]]; then
  export ACCEPT_EULA=Y
  shift
fi
if (($#)); then
  echo "Usage: bash scripts/start_arena.sh --accept-eula" >&2
  exit 2
fi
if [[ ${ACCEPT_EULA:-} != Y ]]; then
  echo "Isaac Sim requires acceptance of NVIDIA's license: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html" >&2
  echo "After reading it, run: bash scripts/start_arena.sh --accept-eula" >&2
  exit 2
fi
if docker container inspect "$arena_container" >/dev/null 2>&1; then
  mounted_project=$(docker inspect "$arena_container" --format '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}')
  if [[ "$mounted_project" != "$project_dir" ]]; then
    echo "Container $arena_container mounts another project: $mounted_project" >&2
    echo 'Choose another name with ARENA_CONTAINER=<name>.' >&2
    exit 1
  fi
  docker start "$arena_container" >/dev/null
else
  if ! docker image inspect "$arena_image" >/dev/null 2>&1; then
    # Context intentionally excludes data, assets, caches and credentials.
    docker build -t "$arena_image" -f "$project_dir/runtime/Dockerfile" "$project_dir/runtime/docker-context"
  fi
  mkdir -p "$project_dir/runtime/home"
  docker run -d --name "$arena_container" --gpus all --ipc=host \
    --user "$(id -u):$(id -g)" \
    --mount "type=bind,src=$project_dir,dst=/workspace" \
    --mount "type=bind,src=$project_dir/runtime/home,dst=/home/user" \
    -e HOME=/home/user -e ACCEPT_EULA=Y -e OMNI_KIT_ACCEPT_EULA=YES \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e UV_PROJECT_ENVIRONMENT=/workspace/.venv-arena \
    -e UV_LINK_MODE=copy \
    "$arena_image" >/dev/null
fi
echo "Runtime ready: $arena_container"
echo 'Next: bash scripts/install_arena_env.sh'
