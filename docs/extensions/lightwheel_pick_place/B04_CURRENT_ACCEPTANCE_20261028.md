# B04 current acceptance — 2026-10-28

Final selected task-scope acceptance: passed=true, remaining=[]. Authoritative evidence: /root/autodl-tmp/B04-review/current_selected_acceptance_20261028.json and acceptance_manifest.current_selected_20261028.json.

Matched baseline: base_lgd_b04_fresh_matched_20261028T102807Z. Human104647, property114946 and robot111828 have original native success, core safety checks and strict matched avoidance acceptance. All use original-reset fresh planning and the same recorded controller. No intermediate-state restoration or cross-variant prefix replay.

Current baseline-start paired seeds42/43/44 all hazard-on valid and zero violations. Hazard-off violations: human53/53/53, property36/36/36, robot9/9/9. This proves scoped loaded leg avoidance, not full-task success rates. Neutral90seconds covers longest selected episode71.2seconds. Original three-view videos fully decoded; six chronological synchronized keyframes per episode visually reviewed; path diagrams exported.

Disclosures: unrelated global fixture-joint checks remain explicitly excluded by user scope. Human additional30mm planning reserve is false; original20mm execution certification passes. Property/robot final observation records success=true and time_out=true; the unchanged termination guard bounds success by the first timeout, but exact first-success timestamps were not separately recorded. Do not claim final time_out=false. Historical failed/false reports remain preserved alongside the audited tool corrections.

Final source/artifact verification is tracked separately in B-group-review/final_handoff_gate_20261028.json. No GitHub push has been performed.
