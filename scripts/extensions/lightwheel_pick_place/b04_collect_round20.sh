#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
variant=${1:?Choose base, human, property or robot}
case "$variant" in base|human|property|robot) ;; *) exit 2 ;; esac
extra=()
[[ "$variant" == human ]] && extra=(--overview-camera --overview-resolution 640 360)
stamp=$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p /root/autodl-tmp/lgd-b04-logs
out=datasets/skill_trials/lightwheel_pick_place/${variant}_lgd_b04_round20_${stamp}
printf 'output=%s\n' "$out"
EPISODE_TIMEOUT_S=2400 bash scripts/extensions/lightwheel_pick_place/run_host.sh collect.py --keep-transport-wrist --loaded-transfer-time-scale 3 --certify-execution-margin --supported-landing-transfer --phase-joint-seeds /root/autodl-tmp/B04-review/pregrasp_only_joint_seed.json --gentle-supported-placement --post-lift-rise .06 --placement-reference datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_handle_20261025T194846Z/result.json --placement-goals "/root/autodl-tmp/B04-review/round15_goals_${variant}.json" --single-candidate-round --task kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian --variant "$variant" --variant-config configs/extensions/lightwheel_pick_place/variants_ladle_robot_round17.json --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_ladle_overnight_original.json --seed 43 --placement_seed 42 --device cpu --defer-full-world-audit --defer-added-hazard-audit --transfer-route-search upper --grasp-body-point .03904817 0 .012 --grasp-tilt 0 --grasp-yaw 90 --grasp-height-offset 0.02617519776119511 --pregrasp-planner auto --place-lower-mode skip --release-ramp-steps 30 "${extra[@]}" --output "$out" > /root/autodl-tmp/lgd-b04-logs/${variant}_handle_${stamp}.log 2>&1
code=$?
printf 'exit=%s output=%s\n' "$code" "$out"
exit "$code"
