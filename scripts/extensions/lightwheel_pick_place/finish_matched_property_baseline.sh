#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
log=/root/autodl-tmp/lgd-day-logs/b04_matched_property_baseline_runner.log
for ((i=0;i<270;i++)); do
 grep -q '^exit=' "$log" && break
 sleep 10
done
baseline=$(sed -n 's/^output=//p' "$log" | head -1)
"$py" -c 'import json,sys;r=json.load(open(sys.argv[1]+"/result.json"));assert r.get("success") and not r.get("development_only"),r.get("error")' "$baseline" || exit 2
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_deferred_world.py "$baseline" --workers 3
bash scripts/extensions/lightwheel_pick_place/run_host.sh audit_self_collision.py "$baseline"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/validate.py "$baseline"
"$py" - "$baseline" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1])
for name in ('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json'):
 assert json.loads((p/name).read_text())['passed'],name
# Separate audit view preserves all original raw files and prior false reports.
source=Path('datasets/skill_trials/lightwheel_pick_place/property_lgd_b04_round16_20261027T095059Z').resolve()
view=Path('/root/autodl-tmp/B04-review/matched_property_standard_view_20261028');view.mkdir(exist_ok=False)
for f in source.iterdir():
 if f.is_file() and not f.name.startswith(('avoidance_audit','source_background')):(view/f.name).symlink_to(f)
(view/'AUDIT_VIEW_PROVENANCE.json').write_text(json.dumps({'source_episode':str(source),'baseline_episode':str(p.resolve()),'scope':'Read-only links to actual native episode, new comparison outputs only; no result/trajectory/scene rewrite, no simulation or replay'},indent=2))
PY
[[ $? == 0 ]] || exit 2
view=/root/autodl-tmp/B04-review/matched_property_standard_view_20261028
timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_source_background.py --baseline "$baseline" --episode "$view"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$baseline" --episode "$view" --output "$view/avoidance_audit.json"
if "$py" -c 'import json,sys;r=json.load(open(sys.argv[1]));assert not r.get("source_settling_nuisance_verified")' "$view/avoidance_audit.json" 2>/dev/null; then
 timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_background_motion.py --baseline "$baseline" --episode "$view" --output "$view/source_background_motion_certificate.exact.json"
 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$baseline" --episode "$view" --output "$view/avoidance_audit.current_review.json"
fi
