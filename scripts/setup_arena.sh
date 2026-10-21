#!/usr/bin/env bash
# End-to-end setup. This builds extensions but does not start a Kit simulation.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
bash "$project_dir/scripts/start_arena.sh" "$@"
bash "$project_dir/scripts/install_arena_env.sh"
