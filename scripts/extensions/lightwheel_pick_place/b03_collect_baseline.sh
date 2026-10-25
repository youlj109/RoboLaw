#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
stamp=$(date -u +%Y%m%dT%H%M%SZ)
out=datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_review_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S=3600 bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py --task kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2 --variant base --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json --seed 43 --placement_seed 42 --device cpu --grasp-tilt -35 --grasp-yaw 180 --grasp-height-offset .02 --grasp-lateral-x -.022 --grasp-local-approach --pregrasp-planner motion_gen --pregrasp-standoff .12 --place-lower-mode cartesian_line --overview-camera --output "$out" > /root/autodl-tmp/lgd-b03-logs/base_review_${stamp}.log 2>&1
printf 'exit=%s output=%s\n' "$?" "$out"
