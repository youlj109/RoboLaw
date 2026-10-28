#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
profile=${1:?robot or property};case "$profile" in robot|property) ;; *) exit 2 ;; esac
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
audit=/root/autodl-tmp/lgd-day-logs/b03_standard_stage_${profile}_audits.log
for ((i=0;i<1200;i++)); do
 grep -q '^AUDIT_EXIT=' "$audit" 2>/dev/null && break
 sleep 10
done
grep -q '^AUDIT_EXIT=0$' "$audit"
control_profile=${B03_CONTROL_PROFILE:-$profile}
base=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b03_standard_matched_${control_profile}_runner.log | head -1)
episode=$(sed -n 's/^output=//p' /root/autodl-tmp/lgd-day-logs/b03_standard_stage_${profile}_runner.log | head -1)
"$py" - "$base" "$episode" <<'PROFILE'
import hashlib,json,sys,pathlib
b,e=map(pathlib.Path,sys.argv[1:]);br=json.loads((b/'result.json').read_text());er=json.loads((e/'result.json').read_text());bp=br.get('explicit_b03_stage_profile',{});ep=er.get('explicit_b03_stage_profile',{})
assert bp.get('sha256') and bp['sha256']==ep.get('sha256') and bp['profile']==ep.get('profile'),'Explicit per-pick control profile mismatch'
(e/'stage_profile_pair_gate.json').write_text(json.dumps({'passed':True,'baseline_episode':str(b),'baseline_result_sha256':hashlib.sha256((b/'result.json').read_bytes()).hexdigest(),'variant_result_sha256':hashlib.sha256((e/'result.json').read_bytes()).hexdigest(),'identical_explicit_profile_sha256':bp['sha256'],'scope':'Additional strict gate: all per-pick controls/goals, not only global first-stage metadata, must match. Original avoidance and native safety gates still mandatory.'},indent=2))
PROFILE
[[ ! -e "$episode/avoidance_audit.fresh_matched.json" ]]
timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_source_background.py --baseline "$base" --episode "$episode"
set +e
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$base" --episode "$episode" --output "$episode/avoidance_audit.fresh_matched.json"
code=$?
set -e
(( code==0 || code==2 ))
if "$py" -c 'import json,sys;assert not json.load(open(sys.argv[1])).get("source_settling_nuisance_verified")' "$episode/avoidance_audit.fresh_matched.json" 2>/dev/null; then
 timeout --kill-after=15 1800 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_background_motion.py --baseline "$base" --episode "$episode" --output "$episode/source_background_motion_certificate.exact.json"
 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_avoidance.py --baseline "$base" --episode "$episode" --output "$episode/avoidance_audit.fresh_matched.current_review.json" || [[ $? == 2 ]]
fi
printf 'FRESH_PAIR_COMPARISON_COMPLETE %s\n' "$episode"
