#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
variant=${1:?Choose base, human, property or robot}
case "$variant" in base|human|property|robot) ;; *) exit 2 ;; esac
extra=()
[[ "$variant" == human ]] && extra=(--overview-camera)
stamp=$(date -u +%Y%m%dT%H%M%SZ)
out=datasets/skill_trials/lightwheel_pick_place/${variant}_lgd_b03_round20_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S="${B03_EPISODE_TIMEOUT_S:-3600}" bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py --release-retract-z .06 --placement-feedback-lower --single-candidate-round --align-second-grasp --round10-second-placement --placement-goals /root/autodl-tmp/B03-review/planning_round_18.json --free-release --placement-reference datasets/skill_trials/lightwheel_pick_place/human_lgd_b03_fixedscene_20261026T092630Z/result.json --reference-clearance-z .005 --task kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2 --pick-order banana_1 banana_2 --variant "$variant" --variant-config "${B03_VARIANT_CONFIG:-configs/extensions/lightwheel_pick_place/variants_bananas_review_layout_v2.json}" --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json --initial-state-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas_settled_review.json --seed 43 --placement_seed 42 --device cpu --defer-full-world-audit --defer-added-hazard-audit --grasp-search b03-topdown --grasp-tilt 0 --grasp-yaw 180 --grasp-height-offset .02 --grasp-lateral-x 0.0 --grasp-local-approach --pregrasp-planner motion_gen --pregrasp-standoff .12 --place-lower-mode transfer --dual-first-place-offset-x -.015 --dual-second-place-offset-x .10 --release-ramp-steps 12 "${extra[@]}" --output "$out" > /root/autodl-tmp/lgd-b03-logs/${variant}_fixedscene_${stamp}.log 2>&1
printf 'exit=%s output=%s\n' "$?" "$out"
