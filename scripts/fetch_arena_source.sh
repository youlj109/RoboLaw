#!/usr/bin/env bash
# Restore the pinned Arena and Isaac Lab source; default source is official.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
python3 "$project_dir/scripts/fetch_source_dependencies.py" --components arena isaaclab "$@"
