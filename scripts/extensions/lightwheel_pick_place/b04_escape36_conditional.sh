#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
for ((i=0;i<60;i++)); do
 kill -0 155047 2>/dev/null || break
 sleep 5
done
/root/autodl-tmp/conda_envs/robolaw_data/bin/python -c 'import json;r=json.load(open("/root/autodl-tmp/B04-review/measured_release_plan36.json"));assert r["all_passed"],r["attempts"][-1:]' || exit 2
bash scripts/extensions/lightwheel_pick_place/b04_collect_round36.sh robot original > /root/autodl-tmp/lgd-day-logs/b04_robot_round36_original_runner.log 2>&1
[[ $? == 0 ]] || exit 2
bash scripts/extensions/lightwheel_pick_place/b04_release36_audits.sh > /root/autodl-tmp/lgd-day-logs/b04_robot_release36_audits.log 2>&1
