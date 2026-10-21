#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
revision=7050ecbc9b3fc689df51ea7cc7f9a64b18294bf6
if [[ ! -d third_party/IsaacLab-Arena/.git ]]; then
  mkdir -p third_party
  git clone https://github.com/isaac-sim/IsaacLab-Arena.git third_party/IsaacLab-Arena
fi
git -C third_party/IsaacLab-Arena fetch origin "$revision"
git -C third_party/IsaacLab-Arena checkout --detach "$revision"
git -C third_party/IsaacLab-Arena submodule update --init submodules/IsaacLab
printf 'Arena source ready at %s\n' "$revision"
