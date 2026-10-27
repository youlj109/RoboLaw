#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
for leg in 3 9; do
 output=/root/autodl-tmp/B03-review/paired_transportreview_080313_leg${leg}_20261027
 [[ -e "$output/paired_planning.json" ]] && continue
 EPISODE_TIMEOUT_S=1800 bash scripts/extensions/lightwheel_pick_place/run_host.sh paired_planning.py --baseline datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_fixedscene_20261026T080313Z --variant-config configs/extensions/lightwheel_pick_place/variants_bananas_transportreview_20261026.json --recorded-plan-index "$leg" --seeds 42 43 44 --segments transfer --variants base human property robot --output "$output"
done
