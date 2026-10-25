#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
for variant in property human; do
  stamp=$(date -u +%Y%m%dT%H%M%SZ)
  out="datasets/skill_trials/lightwheel_pick_place/${variant}_lgd_b02_review_${stamp}"
  log="/root/autodl-tmp/lgd-b02-logs/${variant}_review_${stamp}.log"
  printf '%s variant=%s output=%s log=%s\n' "$(date -Is)" "$variant" "$out" "$log"
  EPISODE_TIMEOUT_S=2400 bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py \
    --task kitchen_bench_lightwheel_pick_lemon_shelf_on_plate_u_shaped_with_island_farmhouse2 \
    --variant "$variant" --variant-config configs/extensions/lightwheel_pick_place/variants_lemon_b02_review.json \
    --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_lemon.json \
    --seed 43 --placement_seed 42 --device cpu --grasp-tilt 0 --grasp-yaw 0 --grasp-height-offset .01 \
    --grasp-local-approach --shelf-lift-template reports/skill_trials/lightwheel_pick_place/shelf_extract_scan_20261024T053356Z/lift_template_v3.json \
    --place-lower-mode skip --place-lower-z .055 --overview-camera --output "$out" > "$log" 2>&1
  code=$?
  printf '%s variant=%s exit=%s output=%s\n' "$(date -Is)" "$variant" "$code" "$out"
done
