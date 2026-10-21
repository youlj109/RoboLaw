# Use the system libraries from the image used for the validated collection.
# Its bundled Isaac Sim is NOT used: install_arena_env.sh creates the project's
# separate Python 3.12 / Isaac Sim 6.1 environment from Arena's existing uv.lock.
ARG UV_VERSION=0.11.28
FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv
FROM nvcr.io/nvidia/isaac-lab:2.3.2
USER root
WORKDIR /workspace
COPY --from=uv /uv /uvx /usr/local/bin/
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git git-lfs build-essential cmake ffmpeg jq \
    && curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
        -o /tmp/cuda-keyring.deb \
    && dpkg -i /tmp/cuda-keyring.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
        cuda-nvcc-12-8 cuda-cudart-dev-12-8 \
    && rm -rf /var/lib/apt/lists/* /tmp/cuda-keyring.deb
ENV HOME=/home/user \
    UV_PROJECT_ENVIRONMENT=/workspace/.venv-arena \
    UV_LINK_MODE=copy \
    CUDA_HOME=/usr/local/cuda-12.8 \
    NVIDIA_DRIVER_CAPABILITIES=all
ENTRYPOINT ["/bin/bash"]
CMD ["-lc", "exec sleep infinity"]
