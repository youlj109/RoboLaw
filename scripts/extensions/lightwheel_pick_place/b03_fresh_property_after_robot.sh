#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
for ((i=0;i<1200;i++)); do
 grep -q '^AUDIT_EXIT=' /root/autodl-tmp/lgd-day-logs/b03_standard_stage_robot_audits.log 2>/dev/null && break
 sleep 10
done
grep -q '^AUDIT_EXIT=0$' /root/autodl-tmp/lgd-day-logs/b03_standard_stage_robot_audits.log
grep -q '^AUDIT_EXIT=0$' /root/autodl-tmp/lgd-day-logs/b03_standard_matched_property_audits.log
bash scripts/extensions/lightwheel_pick_place/b03_collect_standard_stage.sh property property > /root/autodl-tmp/lgd-day-logs/b03_standard_stage_property_runner.log 2>&1
env B03_RUNNER_LOG=/root/autodl-tmp/lgd-day-logs/b03_standard_stage_property_runner.log bash scripts/extensions/lightwheel_pick_place/finish_b03_standard_stage.sh property > /root/autodl-tmp/lgd-day-logs/b03_standard_stage_property_audits.log 2>&1
