# Automatic Scene Finalization

Use the existing local review service at `http://127.0.0.1:8765`. Manual notes
remain in its SQLite database; the finalizer only adds evidence references.
The original reviewed scene and physical collection must already be started.
This command does not generate another trajectory, collect another episode,
change layouts, or retry a failed physical scene.

Current left-obstacle finalization example, using a new output directory:

```bash
env ARENA_CONTAINER=robolaw-arena python3 scripts/extensions/controls/finalize_reviewed_scene.py \
  --scene 03_left_cancel_obstacle \
  --progress reports/controls_completion_20261027/benchmark_round167/progress.json \
  --summary-progress reports/controls_completion_20261027/benchmark_round106/progress.json \
    reports/controls_completion_20261027/benchmark_round119/progress.json \
    reports/controls_completion_20261027/benchmark_round141/progress.json \
    reports/controls_completion_20261027/benchmark_round154/progress.json \
  --manifest configs/extensions/controls/benchmark_acceptance_manifest.json \
  --output reports/controls_completion_20261027/benchmark_finalize167_new \
  --wait-seconds 3600
```

After physical validation passes, it verifies the current frozen config, runs
the independent avoidance audit, resolves the first source-shape violation,
generates a bound real-image witness, and publishes `report.json` to all nine
web entries. The witness overlay is explicitly not a visual-layout review or an
unsafe physical playback. Existing audit/image outputs are verified, not replaced.

`progress.json` records waiting/completion/failure. Failed physical validation
stops finalization, retains the reason, and produces a partial strict report
where possible. Missing, stale or failed evidence cannot become a passed gate.
The global report can remain below9/9 when this individual scene passes.

Large videos/records live in local ignored datasets/reports directories; Git
contains the tools, configurations, authored asset sources and tests, not those
large recordings. These fixed-layout results do not establish randomized or
200-episode reliability, injury prediction or PhysX/source-shape identity.
