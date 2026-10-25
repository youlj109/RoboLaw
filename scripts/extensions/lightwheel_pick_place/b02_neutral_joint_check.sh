#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
prefix=${1:-reports/skill_trials/lightwheel_pick_place/b02_neutral_sdk_20261025}
bash scripts/extensions/lightwheel_pick_place/run_host.sh neutral_hold.py \
  --task kitchen_bench_lightwheel_pick_lemon_shelf_on_plate_u_shaped_with_island_farmhouse2 \
  --device cpu --duration-seconds 6 --background-settling-seconds 2 --output "$prefix/probe_6s"
bash scripts/extensions/lightwheel_pick_place/run_host.sh neutral_hold.py \
  --task kitchen_bench_lightwheel_pick_lemon_shelf_on_plate_u_shaped_with_island_farmhouse2 \
  --device cpu --duration-seconds 69 \
  --reference-episode datasets/skill_trials/lightwheel_pick_place/human_lgd_b02_review_20261025T095249Z \
  --output "$prefix/hold_69s"
