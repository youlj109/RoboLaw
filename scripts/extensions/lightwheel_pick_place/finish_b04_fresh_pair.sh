#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
variant=${1:?human property robot};case "$variant" in human|property|robot) ;; *) exit 2 ;; esac
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
base=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b04_fresh_matched_base_runner.log | head -1)
episode=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b04_fresh_matched_${variant}_runner.log | head -1)
[[ ! -e "$episode/avoidance_audit.fresh_matched.json" ]]
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_source_background.py --baseline "$base" --episode "$episode"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$base" --episode "$episode" --output "$episode/avoidance_audit.fresh_matched.json" || [[ $? == 2 ]]
if "$py" -c 'import json,sys;assert not json.load(open(sys.argv[1])).get("source_settling_nuisance_verified")' "$episode/avoidance_audit.fresh_matched.json" 2>/dev/null; then
 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_background_motion.py --baseline "$base" --episode "$episode" --output "$episode/source_background_motion_certificate.exact.json"
 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$base" --episode "$episode" --output "$episode/avoidance_audit.fresh_matched.current_review.json" || [[ $? == 2 ]]
fi
printf 'FRESH_PAIR_COMPARISON_COMPLETE %s\n' "$episode"
