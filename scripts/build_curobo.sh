#!/usr/bin/env bash
# Rebuild cuRobo against the existing Arena Torch ABI; execute on the host.
set -euo pipefail
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
arena_container=${ARENA_CONTAINER:-arena-kitchen-dev}
docker exec -w /workspace/third_party/curobo \
  -e CUDA_HOME=/usr/local/cuda-12.8 \
  -e "TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_LIST:-}" -e "MAX_JOBS=${MAX_JOBS:-4}" \
  -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 \
  "$arena_container" bash -eu -c '
    export PATH=/workspace/.venv-arena/bin:/usr/local/cuda-12.8/bin:$PATH
    if [[ -z "$TORCH_CUDA_ARCH_LIST" ]]; then
      TORCH_CUDA_ARCH_LIST=$(python -c '\''import torch; print(";".join(sorted({"%d.%d" % torch.cuda.get_device_capability(i) for i in range(torch.cuda.device_count())})))'\'')
      export TORCH_CUDA_ARCH_LIST
    fi
    if [[ -z "$TORCH_CUDA_ARCH_LIST" ]]; then
      echo "No GPU visible. Check NVIDIA Container Toolkit, or set TORCH_CUDA_ARCH_LIST explicitly." >&2
      exit 1
    fi
    echo "Building cuRobo for CUDA architectures: $TORCH_CUDA_ARCH_LIST"
    # Use a fresh build directory: a previous machine may have another GPU ABI.
    build_dir=$(mktemp -d /tmp/arena-curobo-build.XXXXXX)
    trap '\''rm -rf "$build_dir"'\'' EXIT
    python setup.py build --build-base "$build_dir/build" bdist_wheel --skip-build --dist-dir "$build_dir/wheels"
    uv pip install --python /workspace/.venv-arena/bin/python --no-deps --reinstall "$build_dir"/wheels/nvidia_curobo-*.whl
    mkdir -p /workspace/runtime/curobo_wheels
    cp "$build_dir"/wheels/nvidia_curobo-*.whl /workspace/runtime/curobo_wheels/
  '
