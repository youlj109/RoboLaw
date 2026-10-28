#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
profile=${1:?robot or property};case "$profile" in robot|property) ;; *) exit 2 ;; esac
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
log=/root/autodl-tmp/lgd-day-logs/b03_standard_matched_${profile}_runner.log
audit=/root/autodl-tmp/lgd-day-logs/b03_standard_matched_${profile}_audits.log
for ((i=0;i<1200;i++)); do
 grep -q '^AUDIT_EXIT=' "$audit" 2>/dev/null && break
 sleep 10
done
grep -q '^AUDIT_EXIT=0$' "$audit"
baseline=$(sed -n 's/^output=//p' "$log" | head -1)
recipe=/root/autodl-tmp/B03-review/matched_${profile}_stage_recipe_review_20261028.json
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/review_b03_matched_stage_recipe.py --baseline "$baseline" --variant "$profile" --output "$recipe"
"$py" -c 'import json,sys;r=json.load(open(sys.argv[1]));assert r["parameter_recipe_match"] and r["baseline_native_success"]' "$recipe"
view=/root/autodl-tmp/B03-review/matched_${profile}_comparison_20261028
"$py" - "$baseline" "$profile" "$view" <<'PY'
import json,sys,pathlib
base,variant,view=sys.argv[1:];root=pathlib.Path('datasets/skill_trials/lightwheel_pick_place');source=root/({'robot':'robot_lgd_b03_checkpoint_replay_20261027T033440Z','property':'property_lgd_b03_checkpoint_replay_20261027T064923Z'}[variant]);view=pathlib.Path(view);view.mkdir(exist_ok=False)
for f in source.resolve().iterdir():
 if f.is_file() and not f.name.startswith(('avoidance_audit','source_background')):(view/f.name).symlink_to(f)
(view/'AUDIT_VIEW_PROVENANCE.json').write_text(json.dumps({'source_episode':str(source.resolve()),'baseline':str(pathlib.Path(base).resolve()),'scope':'Original raw files linked read-only; new comparisons only; no result metadata rewrite or acceptance override'},indent=2))
PY
timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_source_background.py --baseline "$baseline" --episode "$view"
set +e
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$baseline" --episode "$view" --output "$view/avoidance_audit.json"
code=$?
set -e
(( code==0 || code==2 ))
if "$py" -c 'import json,sys;r=json.load(open(sys.argv[1]));assert not r.get("source_settling_nuisance_verified")' "$view/avoidance_audit.json" 2>/dev/null; then
 timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_background_motion.py --baseline "$baseline" --episode "$view" --output "$view/source_background_motion_certificate.exact.json"
 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$baseline" --episode "$view" --output "$view/avoidance_audit.current_review.json" || [[ $? == 2 ]]
fi
printf 'COMPARISON_COMPLETE: original audit booleans preserved; stage recipe proof separate\n'
