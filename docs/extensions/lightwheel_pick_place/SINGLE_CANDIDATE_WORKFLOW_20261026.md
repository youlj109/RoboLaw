# User-requested single-candidate workflow

User explicitly requested fixed environments, one planning attempt per round, direct modification from the failed plan, and no extra physical diagnostics or repeated adjustment.

`--single-candidate-round` selects the known first banana grasp without duplicate hover/contact/lift preflight. Each actual motion segment makes one direct planning query, retaining its IK, dense path validation and execution collision checks. Failure ends the round. Automatic route fallback, redundant pregrasp search and physical grasp recenter retries are disabled. A complete two-object manipulation still contains separate approach, lift, transfer and release motion segments; this does not claim one global planning call for the entire task.

Approved hazard configurations are unchanged. The current B03 candidate changes only bowl landing goals: first x offset -0.015 m, second +0.10 m. A new matching baseline is running at `datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_fixedscene_20261026T111936Z`; no final result yet at 11:29 UTC. Existing successful baseline/human remain preserved. Property and robot prior native episodes failed second-fruit clearance and are not accepted.

The previous automatic parameter series and queued extra diagnostics were stopped. Night heartbeat remains paused. Extra tests and audit-file promotion were not executed after automatic review rejected them as outside the user's narrowed scope. No additional audit sweep is automatically invoked. Pending gates remain unverified.

B04 has one independent selected baseline queued: native COM-aligned body point x=0.03904817 m, commanded lift 0.12 m, unchanged original 0.08 m physical lift requirement. No parameter sweep is scheduled.
