#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
exec 8>/root/autodl-tmp/B04-review/property_render_recovery.lock
flock -n 8 || exit 0
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
logs=/root/autodl-tmp/lgd-day-logs
"$py" - <<'CHECK'
import json
p='datasets/skill_trials/lightwheel_pick_place/property_lgd_b04_fresh_matched_20261028T110654Z/result.json'
r=json.load(open(p));assert not r['success'] and 'Capture latency guard' in r.get('error','')
CHECK
for ((i=0;i<720;i++)); do
 grep -q '^FRESH_CHAIN_FINISHED' "$logs/b04_fresh_completion_chain.log" 2>/dev/null && break
 sleep 10
done
grep -q '^FRESH_CHAIN_FINISHED' "$logs/b04_fresh_completion_chain.log"
out=/root/autodl-tmp/B04-review/property_render_recovery_$(date -u +%Y%m%dT%H%M%SZ)
EPISODE_TIMEOUT_S=300 bash scripts/extensions/lightwheel_pick_place/run_host.sh profile_b03_saved_render.py --task kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian --variant property --variant-config configs/extensions/lightwheel_pick_place/variants_ladle_property_round14.json --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_ladle_overnight_original.json --display-layout datasets/skill_trials/lightwheel_pick_place/property_lgd_b04_fresh_matched_20261028T110654Z/result.json --device cpu --output "$out" > "$logs/b04_property_render_health.log" 2>&1
"$py" - "$out" <<'HEALTH'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]);j=json.load(open(p/'render_profile.json'));rows=j['measurements'];assert len(rows)==9 and all(r['physics_steps']==0 and r['ensure_render_s']<2 for r in rows),rows
(p/'recovery_gate.json').write_text(json.dumps({'passed':True,'scope':'Fresh isolated Kit static render health only, zero physics; does not prove permanent fix or native success. Original camera/video configuration retained in subsequent collection. One infrastructure-recovery capture follows; existing latency guard still protects all frames.'},indent=2))
HEALTH
mv "$logs/b04_fresh_matched_property_runner.log" "$logs/b04_fresh_matched_property_runner.110654.capture_guard.log"
mv "$logs/b04_fresh_matched_property_audits.log" "$logs/b04_fresh_matched_property_audits.110654.capture_guard.log"
bash scripts/extensions/lightwheel_pick_place/b04_collect_fresh_matched.sh property > "$logs/b04_fresh_matched_property_runner.log" 2>&1
bash scripts/extensions/lightwheel_pick_place/finish_b04_fresh_core.sh property > "$logs/b04_fresh_matched_property_audits.log" 2>&1
bash scripts/extensions/lightwheel_pick_place/finish_b04_fresh_pair.sh property > "$logs/b04_fresh_matched_property_comparison.log" 2>&1
printf 'PROPERTY_RENDER_RECOVERY_FINISHED: inspect actual gates\n'
