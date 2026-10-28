#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
profile=${1:?Choose robot or property profile}
case "$profile" in robot|property) ;; *) exit 2 ;; esac
variant=base
case "$variant" in base|human|property|robot) ;; *) exit 2 ;; esac
extra=()
[[ "$variant" == human ]] && extra=(--overview-camera)
extra+=(--capture-latency-guard)
stamp=$(date -u +%Y%m%dT%H%M%SZ)
out=datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_matched_${profile}_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S="${B03_EPISODE_TIMEOUT_S:-5400}" bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py --b03-stage-profile "/root/autodl-tmp/B03-review/standard_matched_${profile}_stage_profile.json" --single-candidate-round --placement-joint-goals /root/autodl-tmp/B03-review/planning_round_17.json --task kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2 --pick-order banana_1 banana_2 --variant "$variant" --variant-config "configs/extensions/lightwheel_pick_place/variants_bananas_transportreview_20261026.json" --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json --initial-state-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas_settled_review.json --seed 43 --placement_seed 42 --device cpu --defer-full-world-audit --defer-added-hazard-audit --grasp-search b03-topdown --grasp-tilt 0 --grasp-yaw 180 --grasp-height-offset .02 --grasp-lateral-x 0.0 --grasp-local-approach --pregrasp-planner motion_gen --pregrasp-standoff .12 --place-lower-mode transfer --dual-first-place-offset-x -.03 --dual-second-place-offset-x .075 --release-ramp-steps 12 "${extra[@]}" --output "$out" > /root/autodl-tmp/lgd-b03-logs/matched_${profile}_${stamp}.log 2>&1
code=$?
printf 'exit=%s output=%s\n' "$code" "$out"
exit "$code"
