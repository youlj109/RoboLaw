#!/usr/bin/env bash
# Install Arena's pinned cuRobo into the already-created .venv-arena container.
# Existing Torch, Isaac Sim, Isaac Lab, NumPy and Warp packages are preserved.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
arena_container=${ARENA_CONTAINER:-arena-kitchen-dev}
if ! docker exec -w /workspace "$arena_container" test -x /usr/local/cuda-12.8/bin/nvcc; then
  docker exec -u root -w /tmp "$arena_container" bash -eu -c '
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb
    dpkg -i /tmp/cuda-keyring.deb
    apt-get update -qq
    apt-get install -y --no-install-recommends cuda-nvcc-12-8 cuda-cudart-dev-12-8
  '
fi
docker exec -w /workspace \
  -e "UV_HTTP_TIMEOUT=${UV_HTTP_TIMEOUT:-300}" \
  -e "UV_HTTP_RETRIES=${UV_HTTP_RETRIES:-5}" \
  -e "UV_CONCURRENT_DOWNLOADS=${UV_CONCURRENT_DOWNLOADS:-4}" "$arena_container" uv pip install \
  --python /workspace/.venv-arena/bin/python --no-deps \
  -r /workspace/runtime/requirements-curobo.txt

curobo_revision=ebb71702f3f70e767f40fd8e050674af0288abe8
if [[ ! -f third_party/curobo/setup.py ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/NVlabs/curobo.git third_party/curobo
  git -C third_party/curobo config core.sparseCheckout true
  mkdir -p third_party/curobo/.git/info
  cat > third_party/curobo/.git/info/sparse-checkout <<'EOF'
/*
!/images/
!/src/curobo/content/assets/
EOF
fi
# A downloaded project includes this exact source tree without its .git data.
# Fetch the pinned revision only for a newly created checkout.
if [[ -d third_party/curobo/.git ]] && [[ ! -f third_party/curobo/setup.py ]]; then
  git -C third_party/curobo checkout "$curobo_revision"
fi

asset_url=https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/robot_library/droid/droid_fixed_mimic_joint
mkdir -p assets/curobo/droid/urdf
for asset_file in franka_robotiq_2f_85_zero_curobo.yml urdf/franka_robotiq_2f_85_zero.urdf; do
  if [[ ! -f "assets/curobo/droid/${asset_file}" ]]; then
    curl -fL "${asset_url}/${asset_file}" -o "assets/curobo/droid/${asset_file}"
  fi
done
bash scripts/build_curobo.sh
bash scripts/arena_python.sh scripts/smoke_curobo.py
