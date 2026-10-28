# B-group acceptance requirement provenance

Authority: the human supplied the original mentor prompt again on 2026-10-28. It explicitly invokes student_skills/robolaw-lightwheel-pick-place/SKILL.md and its current requirements.

| Requirement | Authoritative project source |
| --- | --- |
| Personal branch lgd, preserve changes, no main push | Mentor prompt; docs/student_assignments/README.md |
| Five tasks; human/property/robot one real success each; no 200-run collection | Mentor prompt; SKILL.md |
| Successful no-added-hazard matching baseline; different controls need their own baseline | README unified delivery; references/avoidance-trial.md Control comparison |
| Actual old trajectory violates original margin under added hazard; actual replanned trajectory safe | SKILL.md; references/avoidance-trial.md Layout and evidence |
| Same source/reset/control; actual initial pose checks; independent world/self/source/background/fixed checks | references/avoidance-trial.md Layout and evidence |
| Task-duration neutral observation and paired planning seeds42/43/44 | references/review-gates.md |
| Original RGB/action/state data, three videos, human overview, decoding and visual review | README unified delivery; references/avoidance-trial.md |
| Preserve failed/historical reports; old accepted does not certify current gates | Mentor prompt; references/review-gates.md |

Audit scripts are implementations of these requirements, not independent authority. A script defect must be corrected with reproducible evidence and negative checks, preserving original reports; it does not justify weakening physical margins or native predicates. Exact whole-file hashes are provenance evidence, not a substitute for checking actual controller equivalence. Four task types remain base plus three variants; additional baseline captures are comparisons, not a fifth type.

Correction: prior statements that matching baselines might be merely an agent-added requirement were incorrect after reading the supplied mentor prompt and the linked skill references. Conversely, not every implementation-specific metadata whitelist is itself a mentor rule.

Current verified B03 robot: robot_lgd_b03_standard_stage_property_20261028T074115Z, baseline base_lgd_b03_matched_property_20261028T063733Z; avoidance_audit.fresh_matched.policy_review.json accepted=true, same_control_parameters=true, avoidance_effective=true. Historical false reports remain. Property082633 native and core safety passed; final pair audit pending when this note was written. No claim that B04 strict comparison or whole-group delivery is already complete.
