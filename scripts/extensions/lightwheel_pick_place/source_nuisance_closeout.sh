#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
root=datasets/skill_trials/lightwheel_pick_place
for item in B03:property_lgd_b03_checkpoint_replay_20261027T064923Z B03:robot_lgd_b03_checkpoint_replay_20261027T033440Z B04:human_lgd_b04_round09_20261027T081735Z B04:property_lgd_b04_round16_20261027T095059Z B04:robot_lgd_b04_round36_20261027T152820Z; do
 task=${item%%:*};episode=$root/${item#*:}
 if [[ "$task" == B03 ]]; then baseline=$root/base_lgd_b03_fixedscene_20261026T080313Z; else baseline=$root/base_lgd_b04_handle_20261025T194846Z; fi
 printf 'EPISODE %s\n' "$episode"
 if [[ ! -e "$episode/source_background_audit.json" ]]; then
  timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_source_background.py --baseline "$baseline" --episode "$episode" > "$episode/source_background_closeout.log" 2>&1
 fi
 if [[ ! -e "$episode/avoidance_audit.json" ]]; then
  timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$baseline" --episode "$episode" --output "$episode/avoidance_audit.json" > "$episode/avoidance_closeout.log" 2>&1
 fi
 if "$py" -c 'import json,sys;from pathlib import Path;p=Path(sys.argv[1]);a=json.load(open(p/"source_background_audit.json"));b=json.load(open(p/"avoidance_audit.json"));assert a["passed"] and not b.get("source_settling_nuisance_verified")' "$episode" 2>/dev/null; then
  if [[ ! -e "$episode/source_background_motion_certificate.exact.json" ]]; then
   timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_background_motion.py --baseline "$baseline" --episode "$episode" --output "$episode/source_background_motion_certificate.exact.json" > "$episode/source_background_motion_closeout.log" 2>&1
  fi
 fi
 if [[ ! -e "$episode/avoidance_audit.current_review.json" ]]; then
  timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$baseline" --episode "$episode" --output "$episode/avoidance_audit.current_review.json" > "$episode/avoidance_current_review.log" 2>&1
 fi
 printf 'FINISHED %s\n' "$episode"
done
