# B02 actual-state diagnosis, 2026-10-25 (in progress)

B02 is NOT accepted overall. New human/property physical task successes and their
independent avoidance audits passed, but the shared kitchen physical-quality gate
failed once actual PhysX kitchen states were recorded. Preserve all data.

## Current evidence

- Baseline: `base_lgd_b02_20261024T072827Z`.
- New property success + independent avoidance accepted:
  `property_lgd_b02_review_20261025T092906Z` (758 frames).
- New human success + independent avoidance accepted:
  `human_lgd_b02_review_20261025T095249Z` (773 frames, overview present).
- First new robot attempt: `robot_lgd_b02_review_20261025T102903Z` FAILED during
  release (729 frames): actual fruit vs barrier base distance .0084826 m < .01 m.
  No intersection was reported. Do not call this a success.
- New robot v2 candidate changes only barrier y from -.55 to -.60 m, x=5.15,
  z=.922. Config `variants_lemon_b02_robot_v2.json`. Offline actual-state replay
  retains baseline true-geometry violations and clears the previous failed path:
  min .0496765 m in sampled previous release states, zero sampled violations.
  This is only a candidate filter, not a new physical success. Preview rendered.

## Neutral evidence and limitations

The old neutral hold read composed USD transforms for kitchen bodies, not live
PhysX body state. With Fabric, those values did not reveal fixture movement.
Do not promote the old neutral `passed` to full physical quality.

- Requested 90s neutral check hit native 70s episode timeout (1050 frames); rejected.
- 69s old-method pose check passed but lacks actual q/dq; not full acceptance.
- Initial new SDK reader's recursive wildcard failed to bind the expected actors.
  Explicit source rigid-body and articulation paths successfully bind 238 bodies.
- Actual SDK 6s record: `b02_neutral_sdk_v2_20261025/probe_6s/neutral_states.npz`.
  A numpy-bool JSON serialization error was fixed; raw states were preserved.
  `physx_probe_recheck.json` is the offline reanalysis of that completed record.
- Actual source-joint inventory: 50 revolute, 63 prismatic, one generic D6, plus
  123 fixed joints. 119 moving-coordinate channels cover 114 non-fixed joints.
  Native articulation q/dq are read where supported; D6 coordinates and rates are
  derived from actual rigid poses/COM velocities, with proper anchor conversion.
- Actual faucet handle rotation changed ~.47743 rad (27.35 degrees); oven temp
  knob ~3.38 rad. Dishwasher position change is small but reported speed spikes.
- One extra coffee-machine `Sites` actor has no geometry or source-joint endpoint
  and free-falls as a semantic marker. Its raw states remain recorded. The actual
  body-motion gate now explicitly excludes such nonphysical markers; it does not
  exclude meaningful fixture bodies or any source-joint endpoint.

## Diagnostics so far

Restoring upstream `reset_nested_physics=True` alone did not cure the motion.
Several source revolute/prismatic drives use noncanonical `drive:X/Y/Z` channels.
Owned `fixture_schema.py` maps those to USD `angular/linear`, preserving authored
numbers and DOFs. It does not lock joints or alter limits/mass/geometry.

The schema-repair 6s probe removed the oven-knob large motion, while faucet motion
and dishwasher speed spikes persisted. Native friction inventory confirms .15 is
actually present on the faucet/dishwasher articulation. No arbitrary friction
increase has been applied. A one-variable diagnostic with scene minimum velocity
solver iterations=4 is running at `b02_solver_velocity_probe_20261025`.

The complete hold with actual kitchen q/dq has not passed. Do not start new formal
collections until physical quality is resolved. Any changed physics mode must be
matched by baseline and variants; do not pair a repaired-mode neutral hold with
old-mode physical episodes and claim acceptance.

## Files and state

All data directories above are under
`datasets/skill_trials/lightwheel_pick_place/`; reports under
`reports/skill_trials/lightwheel_pick_place/`; logs under
`/root/autodl-tmp/lgd-b02-logs/`.
Jupyter evidence: `/root/autodl-tmp/B02-review/joint-motion-evidence.png` and
`joint-motion-summary.txt`, successful human/property images and video contact
sheets, plus `status.txt`.

Branch `lgd`; code commit `d95cd5b` contains first gate/control fixes. Later joint
reader, source-schema diagnostics and manifests are not yet committed. Original
ladle-layout/restore-assets modifications and untracked prior assets/probes are
preserved. B03 remains frozen. No data committed, no push performed. Latest
origin/ylj was fetched and merged earlier; later GitHub branch lookup timed out.
