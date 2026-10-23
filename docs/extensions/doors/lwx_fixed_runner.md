# Bounded fixed-layout runner (lwx)

`run_fixed_sample_lwx.py` collects at most one sample passing the automated gates for a selected task/variant. It does not mark a complete C task or waive actual visual review. Listing and dry-run have been exercised on the current repository; the wrapper has not yet completed an independent end-to-end physical reproduction. Existing live campaigns use their recorded supervisors.

Run from the repository root on the Docker host. Explicitly choose the actual container; this example uses the current lab environment.

```bash
python3 scripts/extensions/doors/run_fixed_sample_lwx.py --list-tasks
python3 scripts/extensions/doors/run_fixed_sample_lwx.py --label C02 --variant robot --baseline datasets/doors/lwx_C02_C03_20261023/C02_base_04 --variant-config configs/extensions/doors/c02_variants_lwx_03.json --container robolaw-wenxuanli --output-root datasets/doors/lwx_reproduction_example --dry-run
```

For an explicitly authorized reproduction, remove `--dry-run`. Default `--max-attempts 1`; the hard upper bound is 10 and `--success-quota` accepts only 1. All GPU work uses the same shared lock. The current run intentionally does not invoke that extra reproduction because accepted configurations should not be sampled again.

Use `--resume` with identical paths and policy/config hashes. An accepted existing sample is rechecked and returned without recollection. An interrupted partial attempt is retained and requires inspection; there is no silent reset or overwrite. Three equal failure categories stop the run for diagnosis even if numerical collision distances/action indices differ.

The variant must use a matching baseline accepted by all five independent gates. Policy, fixed source layout, robot base, review-config hash, seed and source task are bound to recorded metadata. One current-policy paired 42/43/44 check must establish actual source intersections for every off-path and a safe small reroute before physical collection. Safety-margin-only violations are rejected.

The operation and an equal-or-longer neutral are collected, followed by independent scene/full/contact/self audits, original camera keyframes, source avoidance audit, original-speed comparison video and explicitly labelled offline source-risk illustration. Manual frame and projection-alignment review remain required before `finalize_task_delivery_lwx.py` can package a complete base/human/property/robot selection.

Checks: `reports/doors/lwx_C04_C08_20261024/fixed_runner_list01.log`, `fixed_runner_dryrun01.log`, and `unit_tests_all_05.log` (34 tests). No physics was started by the CLI verification.
