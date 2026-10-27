#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
variant=${1:?robot or property}
mode=${2:?development or replay}
case "$variant" in robot) checkpoint=/root/autodl-tmp/B03-review/checkpoints/robot_first17/checkpoint.json ;; property) checkpoint=/root/autodl-tmp/B03-review/checkpoints/property_first16/checkpoint.json ;; *) exit 2 ;; esac
case "$mode" in development) flag=--development-checkpoint ;; replay) flag=--replay-prefix-checkpoint ;; *) exit 2 ;; esac
stamp=$(date -u +%Y%m%dT%H%M%SZ)
out=datasets/skill_trials/lightwheel_pick_place/${variant}_lgd_b03_checkpoint_${mode}_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S=1500 bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py "$flag" "$checkpoint" --gentle-supported-placement --single-candidate-round --align-second-grasp --round10-second-placement --placement-goals /root/autodl-tmp/B03-review/planning_round_18.json --placement-reference datasets/skill_trials/lightwheel_pick_place/human_lgd_b03_fixedscene_20261026T092630Z/result.json --task kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2 --pick-order banana_1 banana_2 --variant "$variant" --variant-config configs/extensions/lightwheel_pick_place/variants_bananas_transportreview_20261026.json --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json --initial-state-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas_settled_review.json --seed 43 --placement_seed 42 --device cpu --defer-full-world-audit --defer-added-hazard-audit --grasp-search b03-topdown --grasp-tilt 0 --grasp-yaw 180 --grasp-height-offset .02 --grasp-lateral-x 0 --grasp-local-approach --pregrasp-planner motion_gen --pregrasp-standoff .12 --place-lower-mode transfer --release-ramp-steps 30 --output "$out" > /root/autodl-tmp/lgd-b03-logs/${variant}_checkpoint_${mode}_${stamp}.log 2>&1
code=$?
printf 'exit=%s output=%s\n' "$code" "$out"
exit "$code"
