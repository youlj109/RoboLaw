#!/usr/bin/env bash
# Seven existing human variants; independent ID/attempt seeds and ordinary resume.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
export ARENA_CONTAINER=${ARENA_CONTAINER:-arena-kitchen-collect}
exec bash scripts/arena_python.sh scripts/collect_obstacle_library.py \
  --ids human_compact_sport_palm_down human_default_mpfb \
    human_medium_shirt_palm_down human_broad_workwear_palm_down \
    human_compact_sport_raised_wrist human_medium_shirt_raised_wrist \
    human_broad_workwear_raised_wrist \
  --randomization-config configs/augmentation_human7.json --seed 42 \
  --episodes-per-id 20 --max-attempts 100 --record-rgb --preview-per-id 3 \
  --placement-strategy lower-first --grasp-tilt-deg 35 --grasp-yaw-deg 0 \
  --grasp-offset-z-m 0 --grasp-offset-y-m -0.01 \
  --transport-bottle-height-m 1.11 --supported-release --transport-tilt-cap-deg 25 \
  --execution-backend vectorized --pipeline --num-envs 4 --validation-workers 4 \
  --device cpu --physics-threads 0 --torch-threads 1 \
  --camera-device cuda:0 --background-rgb --execution-checks deferred \
  --episode-length-s 180 \
  --output datasets/augmentation/human7_n4_180s_20261019/episodes "$@"
