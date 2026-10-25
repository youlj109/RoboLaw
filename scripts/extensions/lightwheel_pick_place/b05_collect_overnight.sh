#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
variant=${1:?Choose base, human, property or robot}
case "$variant" in base|human|property|robot) ;; *) exit 2 ;; esac
extra=()
[[ "$variant" == human ]] && extra=(--overview-camera --overview-resolution 640 360)
stamp=$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p /root/autodl-tmp/lgd-b05-logs
out=datasets/skill_trials/lightwheel_pick_place/${variant}_lgd_b05_can_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S=2400 bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py --task kitchen_bench_lightwheel_place_pepsi_in_basket_u_shaped_with_island_farmhouse2 --variant "$variant" --variant-config configs/extensions/lightwheel_pick_place/variants_pepsi_overnight_layout.json --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_pepsi.json --seed 43 --placement_seed 42 --device cpu --defer-full-world-audit --defer-added-hazard-audit --transfer-route-search upper --grasp-body-point 0 0 .06075 --grasp-tilt 0 --grasp-yaw 0 --grasp-height-offset 0.024842308597613327 --pregrasp-planner motion_gen --place-lower-mode cartesian_line "${extra[@]}" --output "$out" > /root/autodl-tmp/lgd-b05-logs/${variant}_can_${stamp}.log 2>&1
code=$?
printf 'exit=%s output=%s\n' "$code" "$out"
exit "$code"
