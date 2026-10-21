#!/usr/bin/env bash
# Run from repository root. Blender executable is supplied by the caller.
set -euo pipefail
blender_bin=${1:?Usage: build_reposed_human.sh /path/to/blender}
"$blender_bin" --background assets/people/mpfb_corridor/posed.blend --python scripts/extensions/doors/repose_human_arm.py -- --wrist -.6 0 1.2 --elbow-up --asset-dir assets/extensions/doors/human_reach_elbow_up
bash scripts/arena_python.sh scripts/extensions/doors/wrap_reposed_human.py --asset-dir assets/extensions/doors/human_reach_elbow_up
