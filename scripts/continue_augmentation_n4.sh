#!/usr/bin/env bash
# Continue the explicit N4 batch; old accepted data and consumed attempts are retained.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
export ARENA_CONTAINER=${ARENA_CONTAINER:-arena-kitchen-collect}
docker start "$ARENA_CONTAINER" >/dev/null
exec bash scripts/arena_python.sh scripts/collect_obstacle_library.py \
  --execution-backend vectorized --pipeline --num-envs 4 --validation-workers 4 \
  --randomization-config configs/augmentation_nonhuman10.json --seed 42 \
  --episodes-per-id 20 --max-attempts 100 --record-rgb --preview-per-id 3 \
  --device cpu --physics-threads 0 --torch-threads 1 \
  --camera-device cuda:0 --background-rgb --execution-checks deferred \
  --output datasets/augmentation/nonhuman10_n4_20261019/episodes --resume
