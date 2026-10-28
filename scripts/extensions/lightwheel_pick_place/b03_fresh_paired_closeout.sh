#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
out=/root/autodl-tmp/B03-review/paired_fresh_property_controller_leg3_20261028
[[ ! -e "$out/paired_planning.json" ]] || exit 0
EPISODE_TIMEOUT_S=1800 bash scripts/extensions/lightwheel_pick_place/run_host.sh paired_planning.py --baseline datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_matched_property_20261028T063733Z --variant-config configs/extensions/lightwheel_pick_place/variants_bananas_transportreview_20261026.json --recorded-plan-index 3 --seeds 42 43 44 --segments transfer --variants base property robot --output "$out"
