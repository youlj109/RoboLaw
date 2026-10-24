# C07 bounded release-profile diagnosis and trial

The C07 hold58 matched baseline passed its five gates and actual image review.
The corresponding human60 and robot59 episodes opened the microwave but were
rejected during release. They remain failures. This checkpoint records those
failures and a fresh, matched baseline trial; it declares no new task complete.

## Evidence that changed the next action

Robot59 failed at action 978 during release: conservative finger-to-door-panel
clearance 0.001880549789 m versus the original 0.002 m requirement. The new CPU
reconstruction reproduces that measured failed transition to 5.64e-16 m. Human60
likewise failed during release, and its recorded transition is reproduced to
8.09e-16 m. These numbers concern the conservative original finger envelope;
they do not establish physical finger penetration.

During the extra fixed-target hold, measured door position became nearly
stationary, but the instantaneous joint velocity at the end of a physics step
remained nonzero. Increasing hold duration alone is therefore not assumed to
solve release. All existing failed data and the successful hold58 baseline remain.

The bounded CPU diagnostic tests hypothetical release targets over the actual
recorded door and finger states. Both full pose following and position following
produced IK failures, so neither was implemented in the collector or physically
executed. Under the explicitly ideal arm-tracking assumption, fixed-orientation
release targets pass the original world check. This differs from the measured
failed transition and motivates testing retreat timing; it does not establish a
unique failure cause or predict physical success.

## Optional ease-out retreat

The new `--release-retreat-profile ease_out` changes scalar progress from `f` to
`1-(1-f)^2`, with 20 release actions as before. More displacement is requested in
the earlier part, and the increments diminish near the finish. The total 0.03 m
retreat, -55 degree tilt, measured start pose and fixed tool orientation remain.
The original 0.025 rad per-command joint increment cap, cuRobo checks, every
physical-substep audit, contact rules and safety thresholds remain active.

The default is `linear`: its target arithmetic and metadata remain compatible
with existing policies. Baseline and variant drivers forward the optional profile;
the profile is recorded in controller parameters for exact matching. A changed
profile requires a new matched baseline and neutral, never transplanting success
or paired reports from hold58.

In the bounded hypothetical diagnostic, ease-out has no IK failures or world
violations in the observed release samples. The minimum original execution-world
slack changes from 0.005963 to 0.008618 m for robot59, and from 0.005627 to
0.007759 m for human60. These calculations assume perfect arm tracking and the
recorded door/finger evolution; they do not include GPU self-collision or physics.
They justify a checked experiment, not acceptance.

Five geometry/profile tests and the related 42-test CPU regression passed. The
tests cover unchanged default targets, monotone bounded progress with a slower
finish, rigid hinge transforms and original parameter bounds. Syntax checks and
host supervisor CLI checks passed. Actual safety remains subject to full runtime
checks and independent audits.

## Current experiment and other unfinished work

New root: `reports/doors/lwx_C07_release_ease_retry69_20261024`, with datasets in
the corresponding `datasets/doors` root. Its actual fresh baseline is running.
The original contact screen is copied byte-for-byte, with explicit provenance
limiting its reuse to unchanged grasp/approach/pull. It does not validate the new
release timing. Human61 and robot60 source searches and single-sample supervisors
wait for all five new baseline gates; they retain the full approach guard.

C02 human08 and robot26 accepted evidence is retained. Property24 again exceeded
0.15 m added TCP length. Property25 now searches around measured source hulls,
allowing redundant elbow adaptation at the same Cartesian target while screening
the fixed conservative distal envelopes. It must still pass source intersection
in all off seeds, small-detour limits, full approach and actual independent gates.

C01 original opening success and approved steel remain untouched; effective
human/property obstruction remains pending. Wider source hull discovery under
both existing roll39 and center33 baselines produced no accepted placements.
Those bounded negative results do not prove infeasibility. The broad human62
source search is still running. Rejected new experiments do not reclassify the
original opening as failed.

`manifest.json` hashes the frozen evidence copies. Earlier diagnostic source
versions are retained to match their implementation hashes. Large raw data stays
remote. C03/C04/C05/C06/C08 remain the five previously delivered tasks; C01/C02/C07
are not complete. Current processes and future results must be checked live.
