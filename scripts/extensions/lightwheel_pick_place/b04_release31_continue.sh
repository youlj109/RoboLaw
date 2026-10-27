#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
log=/root/autodl-tmp/lgd-day-logs/b04_robot_round31_development_runner.log
for ((i=0;i<120;i++)); do
  grep -q '^exit=' "$log" && break
  sleep 10
done
episode=$(sed -n 's/^output=//p' "$log" | head -1)
"$py" - "$episode" <<'PY'
import json,sys
from pathlib import Path
r=json.loads((Path(sys.argv[1])/'result.json').read_text())
assert r.get('development_only') and r.get('development_release_native_destination_passed') and not r.get('error'),r.get('error','native release did not pass')
print('Development suffix passed original destination predicate; original-reset replay begins')
PY
[[ $? == 0 ]] || exit 2
bash scripts/extensions/lightwheel_pick_place/b04_collect_round31.sh robot original > /root/autodl-tmp/lgd-day-logs/b04_robot_round31_original_runner.log 2>&1
[[ $? == 0 ]] || exit 2
episode=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b04_robot_round31_original_runner.log | head -1)
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_deferred_world.py "$episode" --workers 3
bash scripts/extensions/lightwheel_pick_place/run_host.sh audit_self_collision.py "$episode"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/validate.py "$episode"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_fixed_base.py "$episode"
