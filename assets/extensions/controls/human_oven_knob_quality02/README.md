# Knob-Height Human Candidate

Source: the repository's restored `assets/people/mpfb_corridor/posed.blend`,
bound by `artifact_manifest.json`. Generated with Blender 4.0.2 from the original
rig, retaining body root/feet and the source forearm-relative wrist orientation.
This is a visual candidate, not clinical anatomy or physical safety certification.

The large `person.usdc` and texture payloads are deliberately ignored by Git.
This workstation already has them. To rebuild missing payloads elsewhere, use a
fresh repository-relative output directory visible in the Arena mount; do not
overwrite existing pose/manifests. With the source assets restored:

```bash
/path/to/blender-4.0.2 --background assets/people/mpfb_corridor/posed.blend \
  --python scripts/extensions/controls/repose_human_arm.py -- \
  --wrist -.68 .20 1.27 --hand-orientation follow-forearm \
  --arm-frame source-bend-plane \
  --asset-dir reports/controls_visual_20261027/human_quality02_rebuild_NEW
env ARENA_CONTAINER=YOUR_CONTAINER bash scripts/arena_python.sh \
  scripts/extensions/controls/wrap_reposed_human.py \
  --asset-dir reports/controls_visual_20261027/human_quality02_rebuild_NEW
```

Verify source, USD and texture hashes against the recorded manifest before
restoring missing payloads. If a different exporter produces different hashes,
retain it as a new asset revision and repeat visual/provenance checks; do not
simply replace expected hashes or inherit acceptance. Generator bone checks are
edit-integrity evidence, not an independent scene collision audit.
