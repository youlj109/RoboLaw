#!/usr/bin/env bash
set -uo pipefail
cd /root/autodl-tmp/RoboLaw-lgd
exec 9>/root/autodl-tmp/B04-review/fresh_completion_chain.lock
flock -n 9 || exit 0
logroot=/root/autodl-tmp/lgd-day-logs
for ((i=0;i<720;i++)); do
 grep -q '^AUDIT_EXIT=' "$logroot/b04_fresh_matched_base_audits.log" 2>/dev/null && break
 sleep 10
done
grep -q '^AUDIT_EXIT=0$' "$logroot/b04_fresh_matched_base_audits.log" || { echo 'BASE_NOT_READY: preserve failed attempt, evidence-based diagnosis required'; exit 1; }
for variant in human property robot; do
 runner="$logroot/b04_fresh_matched_${variant}_runner.log"
 if [[ -e "$runner" ]]; then echo "Existing $variant runner: do not duplicate"; continue; fi
 bash scripts/extensions/lightwheel_pick_place/b04_collect_fresh_matched.sh "$variant" > "$runner" 2>&1
 B04_RUNNER_LOG="$runner" bash scripts/extensions/lightwheel_pick_place/finish_b04_fresh_core.sh "$variant" > "$logroot/b04_fresh_matched_${variant}_audits.log" 2>&1
 if grep -q '^AUDIT_EXIT=0$' "$logroot/b04_fresh_matched_${variant}_audits.log"; then
  nohup bash scripts/extensions/lightwheel_pick_place/finish_b04_fresh_pair.sh "$variant" > "$logroot/b04_fresh_matched_${variant}_comparison.log" 2>&1 < /dev/null &
 else echo "$variant failed original native/core; retained, no identical retry"; fi
done
wait
printf 'FRESH_CHAIN_FINISHED: inspect actual gate outcomes, not automatic acceptance\n'
