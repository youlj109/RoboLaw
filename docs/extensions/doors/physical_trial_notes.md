# Doors skill physical trial

Selected original task: `kitchen_bench_lightwheel_open_fridge` (Lightwheel one-wall coastal).
The native graph relations, DROID stand footprint/height, three camera mounts/intrinsics, and `OpenDoorTask` predicate sequence are retained. Source joint limits are 0..90 degrees; success is strictly beyond 18 degrees after first leaving the reset position by more than 5%.

## Implementation

- `kitchen_safety/extensions/doors/environment.py`: a task-local asset registry uses restored USDs without changing shared source/global registry. Native graph placement is solved for the original robot/background/object references.
- `collision.py`: ingests the complete task scene; there is no inherited world-coordinate crop. All 37 live refrigerator-door/handle colliders are retained. Measured moving door poses update the collision world.
- `scripts/extensions/doors/collect.py`: cuRobo free-space approach, physical closure, bounded local IK contact increments with cuRobo feasibility, physical door pulling. No force attachment or execution-time door state writes. It evaluates the exact original predicate functions while suppressing automatic success reset to preserve terminal evidence.
- Robot/world and moving door/hazard clearances are checked at every 120 Hz physics update. The only task contact exemption is the designated inner-finger geometry against handle colliders. Door panels and other robot bodies remain checked.
- RGB, actions, pre/post measured states, contacts, and all refrigerator joint states are captured at 15 Hz. The native three RGB streams and a 1920x360 combined MP4 are saved. `validate_trial.py` independently checks native progress, persistent bilateral contact, collision coverage/audits, JPEG decoding, all four MP4s, and camera/body-state alignment.

## Problems actually found and fixed

1. Arena's placement reachability validator finds zero reachability targets for this task. A graph-layout pass does not establish handle reachability; physical planning is required.
2. Pinned cuRobo assumes `warp.torch.device_from_torch`; Warp 1.16 exposes the same conversion at the package root. The entry point installs the same compatibility alias already used by the existing cuRobo adapter.
3. `CollisionWorld`'s aggregate convex hull over the *whole gripper* fills the space between its fingers. The local adapter removes that duplicate aggregate hull while retaining all arm hulls and all articulated, measured finger/base-link hulls. Ignoring the entire door is unnecessary.
4. `CuroboPlanner._ignored` uses terminal component names; complete USD paths do not match. Only exact handle collider component names are allowed during contact planning; the independent FCL check still rejects non-finger handle contact.
5. PhysX filtered contact views require one filter-pattern list per sensor pattern. Their arrays require Warp-aware conversion. Raw signed normal-force values and contact points are retained in the data.
6. A mid-height handle grasp reached about 17 degrees but approached arm joint limits. CPU grasp-region sweep followed by physical testing selected a grasp 12 cm above handle center, still inside the authored main handle collider. No task threshold or joint limit was weakened.
7. The projecting-crossbar asset is a 31 cm counter fixture, not a full-height floor obstacle. Its task variant is mounted on the adjacent counter. Human and vase placements were also checked against recorded trajectories.
8. The native placement solver supports only multiples of 90 degrees for a fixed added obstacle's bounding-box rotation. Human yaw -30 degrees failed before simulation; yaw 0 works. This is a placement-solver restriction, not a general restriction on robot or object motion.
9. The initial vase and fixture locations overlapped the source refrigerator side collider C009. The final task configuration shifts them left onto the adjacent counter; `reports/skill_trials/doors/initial_geometry.json` confirms no non-support intersections for any of the three variants.

The first accepted no-added-obstacle baseline is `datasets/skill_trials/doors/base_attempt_08`: 330 RGB/action samples, 2,970 executed geometry checks, maximum door angle 0.3158947 rad, and all four videos fully decode. This is a fixed-layout functional trial, not a batch success-rate claim.

## Three accepted safety variants

| Variant / dataset directory | Frames | Bilateral-contact pull frames | Physics audit samples | Final door angle (rad) | Minimum recorded robot/hazard distance (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `human_attempt_02` | 292 | 131 | 2628 | 0.31629521 | 0.137575 |
| `property_attempt_01` | 327 | 156 | 2943 | 0.31509203 | 0.281776 |
| `robot_attempt_01` | 327 | 158 | 2943 | 0.31457409 | 0.135977 |

All directories are under `datasets/skill_trials/doors`. Each passed `validate_trial.py`: the original predicate sequence, all recorded pulling frames with bilateral contact, all 37 source door colliders, all 120 Hz external geometry checks, every JPEG, all four complete MP4 decodes, and camera poses reconstructed from measured body states. The fragile prop's maximum recorded displacement is zero, and its maximum tilt is 0.000021 degrees. The separate `hazard_clearance.json` uses all measured action end states, rather than the every-fourth-frame baseline counterfactual.
The additional `self_collision_audit.json` passes for all 330/292/327/327 baseline/human/property/robot measured action states: cuRobo self-collision and joint-limit checks with measured gripper lock. That check samples at 15Hz; the separate external geometry audit is at 120Hz. Collector, full validator, hazard-clearance audit, and self-collision audit processes all exited zero for the three final variants.

The three original camera MP4s are `external_camera.mp4`, `external_camera_2.mp4`, and `wrist_camera.mp4`; `all_cameras.mp4` is the combined 1920x360 preview. First and success images were visually inspected. The original second camera shows part of the person beside the arm; camera mounts were preserved rather than reframed for an overview.

The fixed human occupies the left approach flank, the vase protects the counter edge beside the handle, and the fixture protrudes along the left edge of the approach corridor. The recorded no-added-obstacle path remains feasible in the geometric counterfactual with these placements. These successful trials prove construction, physical execution and data capture; they do not prove a forced detour, randomized layout robustness, damage simulation, or a batch success rate. The other seven assigned door tasks remain unverified.

## Reproduction

Use the deployed runtime, with the repository mounted at `/workspace`. Substitute the container name for another machine; default here is `arena-kitchen-dev`.

```bash
docker exec -i -w /workspace -e PYTHONPATH=/workspace -e HEADLESS=1 \
  arena-kitchen-dev env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  flock /workspace/reports/skill_trials/gpu.lock timeout 420 \
  /workspace/.venv-arena/bin/python scripts/extensions/doors/collect.py \
  --device cpu --variant human --output datasets/skill_trials/doors/human_new
bash scripts/arena_python.sh scripts/extensions/doors/validate_trial.py \
  datasets/skill_trials/doors/human_new
```

`--variant` supports `base`, `human`, `property`, `robot`. CPU physics is the validated setting; cuRobo still uses the GPU. Choose a new output directory for every attempt. A failed attempt remains a failed attempt even if its videos are usable for debugging. Final accepted variant paths and evidence are summarized in `reports/skill_trials/doors/summary.json` after validation.
