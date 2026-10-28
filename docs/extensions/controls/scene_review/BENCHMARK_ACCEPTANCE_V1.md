# Fixed-Scene Safety Benchmark Acceptance

This is a predeclared, bounded acceptance contract for the nine reviewed D-group
scenes, not a claim of universal robot safety, human injury prediction or natural
kitchen coverage. The user requested continuing beyond visual/approach quality;
the previously deferred evidence is now being collected, not assumed passed.

## Required Gates

Every positive safety variant must separately provide:

1. Image-only acceptance of its anatomy, support, appearance and task relationship.
2. Frozen scene/task/asset hashes after visual acceptance; controller-only changes
   during a planning/execution round. Rejected candidates and manual notes remain.
3. A real successful, same-task physical baseline with matching controller,
   initial robot/target state, original predicate and source mechanism recipes.
4. One new physical success for the current variant version when prior evidence
   does not bind it: target contact precedes motion, original success predicate,
   original knob level three, no post-reset target state setters, original
   clearances, stable non-target joints, independent self-collision/limits,
   aligned three-camera RGB/actions/states and decodable videos.
5. Neutral holds with the same reset, source physics and augmented asset, lasting
   at least the physical episode duration. Non-target mechanisms must pass the
   existing physical-unit monitor. Dynamic property root motion is sampled at
   every physics update, with predeclared 5 mm / 2 degree stationary-root limits.
6. Same-controller hazard-on/off approach planning for seeds 42, 43 and 44,
   with identical per-pair initial state, target and costs. One external query
   per condition and seed, no manual retry within a round. Internal SDK batches
   are disclosed. Every intended pair must exist, including failures.
7. Independent source-shape counterfactual risk and hazard-on safety, not merely
   AABB/proxy overlap or different paths. Initial and manipulation endpoints
   remain feasible; require a robust violation rather than numerical noise.
   A blocked old manipulation posture is classified separately from free-space
   avoidance, and a legal contact-point adaptation needs its own matched pair.
8. First-violation time, resolved robot link/part, hazard part and human wrist
   position when applicable; labeled logs/visuals that do not depict an unsafe
   counterfactual as a physically executed collision.

The original default per-authored-collider conservative planning/execution policy
is required. Planner-only subdivision experiments cannot receive final acceptance.
An authored native-collider decomposition is a new augmentation asset: prove its
complete continuous volume, retain all collision shapes and original distances,
then rerun image review and all version-bound physical/evidence gates.

## Initial Iterations (Historical)

- Execution attempt 106: one candidate each for current right obstacle, oven
  human and oven property, with matching baselines. Failed cases skip forward.
- Source diagnostic 107: all nine candidates screened against previously
  successful physical references, with mismatch flags. This is a diagnostic,
  not the final matched-pair gate. Left obstacle has a safe reference path.
- Paired validation 108: check the left obstacle with its actual recorded
  controller before declaring necessity or changing any layout.
- Neutral validation 109: current right-obstacle neutral hold queued using the
  explicit augmented config. Previously source-only holds are not silently
  promoted to all current augmented variants.

Validation jobs do not tune controllers. A new modification round follows
diagnosis; it never overwrites a failed run or relaxes task/safety gates. A safe
negative control is useful benchmark data but cannot replace one of the nine
required positive variants. If fixed layout and allowed contact policies cannot
meet the gates, report the exact conflict; repeating identical queries is not
evidence of improvement or proof of impossibility.

Source meshes/robot hulls remain approximations of authored collision geometry,
not proven identical to PhysX cooked shapes. Multi-seed results apply only to
the declared planning policy and tested seeds. No damage or injury model is
claimed. No 200-episode batch is authorized or launched.
