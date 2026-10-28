#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
log=/root/autodl-tmp/lgd-day-logs/b03_standard_matched_robot_runner.log
for ((i=0;i<600;i++)); do
 grep -q '^exit=' "$log" && break
 sleep 10
done
audit=/root/autodl-tmp/lgd-day-logs/b03_standard_matched_robot_audits.log
for ((i=0;i<600;i++)); do
 grep -q "^AUDIT_EXIT=" "$audit" && break
 sleep 10
done
grep -q "^AUDIT_EXIT=0$" "$audit" || exit 2
baseline=$(sed -n 's/^output=//p' "$log" | head -1)
/root/autodl-tmp/conda_envs/robolaw_data/bin/python -c 'import json,sys;r=json.load(open(sys.argv[1]+"/result.json"));assert r.get("success"),"Do not repeat a failed common stage on another profile: "+str(r.get("error"))' "$baseline"
bash scripts/extensions/lightwheel_pick_place/b03_collect_standard_matched_baseline.sh property > /root/autodl-tmp/lgd-day-logs/b03_standard_matched_property_runner.log 2>&1
bash scripts/extensions/lightwheel_pick_place/finish_b03_standard_baseline.sh property > /root/autodl-tmp/lgd-day-logs/b03_standard_matched_property_audits.log 2>&1
