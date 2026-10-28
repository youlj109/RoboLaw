#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
base=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b04_fresh_matched_base_runner.log | head -1)
for variant in human property robot; do
 case "$variant" in human) config=variants_ladle_overnight_layout.json ;; property) config=variants_ladle_property_round14.json ;; robot) config=variants_ladle_robot_round25.json ;; esac
 out=/root/autodl-tmp/B04-review/paired_fresh_common_${variant}_102807_20261028
 [[ ! -e "$out/paired_planning.json" ]] || continue
 EPISODE_TIMEOUT_S=1800 bash scripts/extensions/lightwheel_pick_place/run_host.sh paired_planning.py --baseline "$base" --variant-config "configs/extensions/lightwheel_pick_place/$config" --recorded-plan-index 3 --seeds 42 43 44 --segments transfer --variants base "$variant" --output "$out"
done
