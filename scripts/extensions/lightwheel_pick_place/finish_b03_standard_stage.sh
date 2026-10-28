#!/usr/bin/env bash
set -euo pipefail
trap 'printf "AUDIT_EXIT=%s\n" "$?"' EXIT
cd /root/autodl-tmp/RoboLaw-lgd
profile=${1:?robot or property}
case "$profile" in robot|property) ;; *) exit 2 ;; esac
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
log=${B03_RUNNER_LOG:-/root/autodl-tmp/lgd-day-logs/b03_standard_matched_${profile}_runner.log}
for ((i=0;i<600;i++)); do
 grep -q '^exit=' "$log" && break
 sleep 10
done
baseline=$(sed -n 's/^output=//p' "$log" | head -1)
"$py" -c 'import json,sys;r=json.load(open(sys.argv[1]+"/result.json"));assert r.get("success") and not r.get("development_only"),r.get("error")' "$baseline"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_deferred_world.py "$baseline" --workers 3
bash scripts/extensions/lightwheel_pick_place/run_host.sh audit_self_collision.py "$baseline"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/validate.py "$baseline"
if "$py" -c 'import json,sys;assert json.load(open(sys.argv[1]+"/result.json"))["variant"]!="base"' "$baseline" 2>/dev/null; then
 env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_fixed_base.py "$baseline"
fi
"$py" - "$baseline" <<'PY'
import json,sys,pathlib
p=pathlib.Path(sys.argv[1]);g={}
for n in ('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json'):
 g[n]=json.loads((p/n).read_text()).get('passed')
print(json.dumps({'baseline':str(p),'core_gates':g,'all_core_passed':all(v is True for v in g.values()),'strict_comparison':'Requires separate per-stage provenance review; not automatically accepted'}),flush=True)
PY
