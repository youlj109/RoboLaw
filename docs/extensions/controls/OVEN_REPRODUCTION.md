# Oven Temperature Knob: zxw

## Contract

Task: `kitchen_bench_lightwheel_turn_oven_temperature_knob_g_shaped_large_scandinavian`.
The original reset is level -1; the original target is level 3, not 3 radians.
The fixed source uses `floor`: level 3 is approximately [120, 150) degrees.
The controller aims at the interior 135-degree angle, physically opens its
gripper, and retreats. It never writes a target joint state during execution.
The target is the oven in `oven_right_group`, not another identically named knob.
The original robot pose, 0.7 m stand, task predicate, kitchen and cameras remain.

## Matched Policy

Config: `configs/extensions/controls/temperature_fixed_branch01.json`.
The fixed IK branch is taken from the first successful original-TCP baseline03
pregrasp state, with its path/hash embedded in the config. Every new baseline
and variant re-solves that same TCP with that same seed, then uses cuRobo
joint-goal planning. There are no hand-authored detour waypoints and no
hazard-dependent contact points. Seeds 42/43/44 are paired planning checks,
not three independent physical demonstrations or a batch-success estimate.

The older pose-goal baseline03/property02/robot02 remain physical experiments.
They cannot be mixed into the fixed-branch delivery. Old seed43 pose-goal
routes could avoid the human without adjustment, so that policy did not pass
the declared multi-seed human-necessity gate. Rejected poses that collided
with panda_link5/6 were not executed or excused by changing safety margins.

## Explicit Physics Models

All model/source/overlay hashes are frozen in each episode's `result.json`.
These are declared idealized passive models, not manufacturer calibration:

- Temperature knob: retain source 0.2 kg mass, collision geometry and 0-180
  degree limits; project its COM onto the original axis (about 0.523 mm radial
  correction), zero angular stiffness and 0.05 Nm s/rad damping.
- Racks: correct the source's ineffective X drive with a correctly named
  linear drive, zero stiffness and 1 N s/m damping. Preserve geometry, masses,
  axes and limits. Real 0.05 N force pulses established free travel of about
  82.26/57.86 mm; this is a diagnostic, not robot task success.
- Toaster knobs and faucet: independently balanced passive models with zero
  stiffness and external-torque mobility checks; they are not locked.

`neutral03` observes all 90 kitchen DOFs for 50 seconds with the same models.
Every accepted episode must fit within that duration, or require a longer new
neutral run. Neutral tolerances remain 1 degree rotational / 0.1 mm linear.
Runtime reset requests after initialization are suppressed and counted;
independent validation also rejects robot reset discontinuities.

## Human Asset Reconstruction

Source: `assets/people/mpfb_corridor/posed.blend`, Blender 4.0.2.
`repose_human_arm.py` changes only the right arm; the independent export audit
checks six original meshes, 2948 unchanged foot vertices, unchanged other
bones, wrist position and original bone lengths. No body scaling is used.
The final fixed-branch candidate uses `human_oven_reach_down`.

Exact candidate recipes below are asset-local wrist coordinates in metres.
Run Blender's `--background --python scripts/extensions/controls/repose_human_arm.py
-- --wrist X Y Z --asset-dir NEW_DIRECTORY` with the listed extra arguments.
Never overwrite the sole existing audited asset. Then run
`wrap_reposed_human.py --asset-dir NEW_DIRECTORY`, independently audit the
export with `audit_human_export_blender.py`, and create a fresh manifest/config
using `prepare_oven_candidates.py`. Check each script's `--help` for output
arguments. Binary export differences require a documented new asset version
and fresh gates, not merely replacement expected hashes.

| Asset suffix | Wrist | Extra arguments |
| --- | --- | --- |
| low | -0.60, 0, 1.02 | `--elbow-up` |
| mid | -0.60, 0, 1.10 | default source bend |
| mid_up | -0.60, 0, 1.10 | `--elbow-up` |
| extended | -0.68, 0, 1.12 | `--elbow-up` |
| down | -0.68, 0.20, 1.10 | `--elbow-down` |
| outside | -0.68, 0.20, 1.10 | `--elbow-up` |
| diagonal | -0.65, 0.35, 1.10 | `--elbow-up` |
| lateral | -0.68, 0.20, 1.10 | `--elbow-direction 0 -1 0` |
| forward | -0.68, 0.20, 1.10 | `--elbow-direction 0 1 0` |

The final person's root is [4.93, -3.85, 0] m, yaw -90 degrees.
The fragile floor lamp root is [5.05, -3.73, 0.001] m and may settle naturally.
The visible fixed guard root is [5.05, -3.73, 0] m. The actual kitchen floor,
not the toaster countertop, supports these assets. The whole fragile fixture
is conservatively protected; no fracture or human-injury simulation is claimed.

## Fresh Runs

Use the existing `robolaw-arena` Docker environment; `--device cpu` still needs
GPU cuRobo/RTX rendering. `run.sh` holds the shared GPU lock internally.
Use new paths for every attempt. Do not edit `run.sh` during a running trial.
The original inspection stage and passive-model diagnostics are local ignored
files and must exist and match the source; they do not ship through Git.

```bash
export ARENA_CONTAINER=robolaw-arena
bash scripts/extensions/controls/run.sh collect_trial.py \
  --controls-task temperature \
  --config configs/extensions/controls/temperature_fixed_branch01.json \
  --variant base --output datasets/controls_temperature_fresh/base \
  --max-steps 1000 --device cpu
python3 scripts/extensions/controls/complete_temperature.py \
  --config configs/extensions/controls/temperature_fixed_branch01.json \
  --baseline datasets/controls_temperature_fresh/base \
  --episodes-root datasets/controls_temperature_fresh/variants \
  --reports-root reports/controls_temperature_fresh/verification \
  --stage reports/controls_temperature_zxw_20261023/inspection03/stage.usda \
  --neutral reports/controls_temperature_zxw_20261023/neutral03 \
  --rack-mobility reports/controls_temperature_zxw_20261023/rack_mobility01 \
  --faucet-mobility reports/controls_zxw_20261023/faucet_mobility \
  --output docs/extensions/controls/temperature_fresh_results.json
```

The fail-closed pipeline checks source manipulation feasibility and all 12
paired plans before physical hazard collection. It then collects one episode
per hazard and independently checks original success/contact/release, 120 Hz
external clearance/background motion, action-rate measured self-collision,
fixed-base legality, true source-geometry necessity and comparison video.
Every command and exit status is in `verification_progress.json` and its logs.
Inspect actual final report gates; a process exiting is not acceptance.

Each accepted hazard directory contains all original camera videos, HDF5 RGB,
state/contact arrays, source/config hashes, comparison video and an annotated
`first_violation.png` plus `first_violation_evidence.json`. The latter records
the source-baseline time, risk part and human wrist world position when relevant.
Red baseline hazards are GEOMETRY ONLY; no unsafe baseline is physically
replayed. Projected AABBs are explanatory conservative bounds, not proof of
exact contact. Source-shape FCL checks remain authoritative within the stated
collision-model assumptions.

## Retained Rejections

Baseline01: 1.997032 mm finger/panel gap below the unchanged 2 mm requirement.
Baseline02: target ran past the success band after release, then a legacy reset
caused a robot discontinuity; rejected despite an earlier predicate success.
Property01: the old 300-action budget was insufficient.
Robot01: reached the original level 3 but not the extra 135-degree interior
criterion and complete release/retreat. Wide guards blocked the manipulation
endpoint. Human candidates either obstructed source manipulation, failed IK,
or left seed43 unaffected. All remain retained, not counted as successes.

The six newly generated human USD wrappers retain their exporter-authored
blank line at EOF. `git diff --check` flags these six formatting warnings;
their bytes are retained because the recorded manifests and evidence bind
their exact hashes. This is not a relaxed physics or safety gate.
