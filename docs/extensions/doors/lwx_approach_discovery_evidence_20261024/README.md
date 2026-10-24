# Contact approach discovery and retained failures

This is an intermediate checkpoint. C01, C02 and C07 are not complete deliveries.

C02 steel24 passed all three paired pregrasp comparisons (maximum added TCP 0.0466916003 m), then the physical collector rejected approach action 115 at 0.0099844341 m versus the unchanged 0.01 m requirement. The failure and original videos are retained. The CPU diagnostic reproduces that measured rejection within 0.000000067 m; source robot hull clearance remains positive while the original conservative arm capsule is below its threshold. Thus this is an execution-envelope rejection, not a demonstrated source-mesh collision.

The same diagnostic continues the existing 49-step contact approach from each paired on endpoint under perfect command tracking. All three predicted approaches intersect the conservative arm envelope. Earlier discovery constrained only measured partial approach/reference states, not their entire still-unexecuted continuation. These offline continuations are not physical rollouts and cannot certify dynamics, GPU self collision, or full task success.

`search_task_hazards_v27_lwx.py` can use the recorded diagnostic command states as additional conservative discovery constraints. It preserves original source-intersection evidence and margins. Reference robot configurations may constrain a different explicitly disclosed hazard asset; no hazard geometry is transferred as evidence. Additional support-rejection counters explain discovery failures. A six-candidate actual-geometry comparison gave identical v26/v27 candidates and all original statistics when the new option is absent; v27 also reports a new diagnostic counter.

The optional `--screen-contact-approach` campaign/watcher flag evaluates each otherwise eligible new paired candidate before a physical attempt. It only rejects candidates; it does not change the controller, grasp target, safety thresholds, or final acceptance. Predicted clearance must include 3 mm extra hazard slack plus unchanged full-world checks; a passing prediction still requires real collection and all independent audits. New C02 crossbar26 and property23 campaigns enable this screen.

Fifty existing CPU tests passed; scripts compile. Failure reconstruction and the v26/v27 comparison use real source geometry. No unsafe counterfactual was physically executed. Source discovery was incrementally developed; historical source reports do not contain immutable runtime copies and are not retrospectively rehashed as execution snapshots.

C07 baseline52 has all five independent gates and previously viewed three-camera keyframes. Human55 physically opened the door but release was rejected at 0.00194272094 m against 0.002 m; retain it as rejected. Its release began near -0.04675 rad/s, compared with baseline -0.01915 rad/s. A new matched baseline58 tests the existing 15-action fixed closed-gripper hold, followed by the same 0.03 m/-55 degree retreat. This is a hypothesis until physical execution and independent audits pass. Root52 steel52 remains separate; the new root waits for its terminal result to avoid duplicate accepted collection.

C01 original successful episodes and the user-approved steel presentation remain untouched. New human/property experiments repair obstruction and add current evidence; their rejected planning attempts are not original C01 opening failures.

Raw data remain on the remote workstation. The manifest identifies exact evidence paths and copied hashes. See the task status and live report files for subsequent progress.

C02 crossbar26 candidate15 passed the unchanged three-seed small-detour criteria (maximum added TCP 0.1411324135 m, joint ratio 1.0451300106) and all predicted contact approaches with at least 3 mm extra hazard slack. Its diagnostic is copied here. This validates the new guard on a positive real-geometry case; it is still not physical acceptance.

Later root52 steel52 also opened physically but was rejected during release at 1.89389954 mm against 2 mm. New baseline58 and both matched watchers are already queued; previous raw episodes remain unchanged.

C02 property23 uses a declared three-candidate derived selection preferring fewer measured baseline intersection actions. Its CPU discovery allows one intersecting baseline sample to find boundary placements; this does not change any safety distance or final requirement for actual source intersection in every predeclared paired seed, small rerouting, and complete physical/audit evidence. The original 20-candidate source report and exact selection indices/hashes remain remote.
