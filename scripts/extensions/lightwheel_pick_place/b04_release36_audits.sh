#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
log=/root/autodl-tmp/lgd-day-logs/b04_robot_round36_original_runner.log
for ((i=0;i<120;i++)); do
  grep -q '^exit=' "$log" && break
  sleep 10
done
episode=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b04_robot_round36_original_runner.log | head -1)
"$py" -c 'import json,sys;r=json.load(open(sys.argv[1]+"/result.json"));assert r.get("success") and not r.get("development_only"),r.get("error")' "$episode" || exit 2
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_deferred_world.py "$episode" --workers 3
bash scripts/extensions/lightwheel_pick_place/run_host.sh audit_self_collision.py "$episode"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/validate.py "$episode"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_fixed_base.py "$episode"
