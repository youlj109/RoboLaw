#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
for variant in human property robot; do
 case "$variant" in
 human) config=variants_ladle_overnight_layout.json ;;
 property) config=variants_ladle_property_round14.json ;;
 robot) config=variants_ladle_robot_round25.json ;;
 esac
 output=/root/autodl-tmp/B04-review/paired_current_${variant}_20261027
 [[ -e "$output/paired_planning.json" ]] && continue
 EPISODE_TIMEOUT_S=1800 bash scripts/extensions/lightwheel_pick_place/run_host.sh paired_planning.py --baseline datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_handle_20261025T194846Z --variant-config "configs/extensions/lightwheel_pick_place/$config" --recorded-plan-index 3 --seeds 42 43 44 --segments transfer --variants base "$variant" --output "$output"
done
