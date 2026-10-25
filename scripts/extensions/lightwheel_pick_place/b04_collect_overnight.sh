#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
variant=${1:?Choose base, human, property or robot}
case "$variant" in base|human|property|robot) ;; *) exit 2 ;; esac
stamp=$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p /root/autodl-tmp/lgd-b04-logs
out=datasets/skill_trials/lightwheel_pick_place/${variant}_lgd_b04_handle_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S=2400 bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py --task kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian --variant "$variant" --variant-config configs/extensions/lightwheel_pick_place/variants_ladle_overnight_layout.json --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_ladle_overnight_original.json --seed 43 --placement_seed 42 --device cpu --defer-full-world-audit --transfer-route-search upper --grasp-body-point -.08 0 .012 --grasp-tilt 0 --grasp-yaw 90 --grasp-height-offset .0298478 --pregrasp-planner motion_gen --place-lower-mode cartesian_line --overview-camera --output "$out" > /root/autodl-tmp/lgd-b04-logs/${variant}_handle_${stamp}.log 2>&1
code=$?
printf 'exit=%s output=%s\n' "$code" "$out"
exit "$code"
