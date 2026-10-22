# 本机 Docker 运行记录

本机项目：`/home/wp/Projects/RoboLaw`。团队代码、风险规则和依赖版本以 `huirongzeng` 仓库为准；本次对齐提交为 `c69c4f459850aa3d8ad2c0785d0f134a5ea43b15`。

本文件仅说明本机连接方式。共同使用的说明是 `README.md`、`docs/risk_perception.md` 和 `docs/warp_mesh_30hz.md`，共同运行入口为仓库原有 `scripts/arena_python.sh`。没有修改已跟踪的源码、规则或安装脚本。

## 每个新终端

```bash
cd /home/wp/Projects/RoboLaw
export DOCKER_HOST=unix:///run/robolaw-docker/docker.sock
export ARENA_CONTAINER=robolaw-arena
bash scripts/start_arena.sh --accept-eula
bash scripts/arena_python.sh scripts/check_runtime.py
bash scripts/arena_python.sh scripts/check_risk_perception.py
```

两个环境变量只作用于当前终端，用于选择本项目的 Docker 服务和容器。未设置时，Docker 仍使用原来的服务。不要在宿主机激活 `.venv-arena`：该环境的解释器和源码路径属于容器。

## 按 README 采集

```bash
bash scripts/arena_python.sh scripts/collect_safety_benchmark.py \
  --ids human_default_mpfb --num-envs 1 --episodes-per-id 1 --max-attempts 1 \
  --output datasets/risk/docker_human_run01 --run-id docker_human_run01
```

每次使用新的输出目录和 run-id；加 `--dry-run` 可只检查命令。输出文件保存在当前项目的 `datasets/` 中。上述完整采集命令本次仅验证了 dry-run。

## 本机隔离方式

- 专用系统服务：`robolaw-docker.service`，已启用开机启动。
- 服务定义：`/etc/systemd/system/robolaw-docker.service`。
- 专用配置、镜像和容器数据：`.runtime/docker/`。
- 专用网桥：`robolaw0`，网段 `172.29.233.0/24`。
- 容器：`robolaw-arena`；镜像由原仓库 `runtime/Dockerfile` 构建。
- 容器内项目位置：`/workspace`；容器 HOME 绑定本机 `runtime/home/`。
- 原有 rootless Docker 服务未修改或重启；NVIDIA 运行时未启用 `no-cgroups`。
- 如专用服务未运行：`sudo systemctl start robolaw-docker`，然后执行上面的新终端命令。

## 安装与验证记录

已运行原仓库 `scripts/setup_arena.sh --accept-eula`，使用同一 Arena 锁文件和依赖清单，并在容器中重新编译 cuRobo。现有下载缓存仅用于减少下载量。容器直连 GitHub 曾超时，安装临时采用 uv 离线缓存模式；该设置已移除。

已完成运行时版本检查、cuRobo FK/IK 检查、Warp CUDA 风险预检，以及 5 项 Warp 几何、6 项 IR 开关和 18 项参考评价器测试。记录在 `reports/docker_alignment/`。真实人体场景完成 3 个物理环境步，三路 1280×720 RGB 均通过检查，进程退出码 0。最终记录为该目录 `completed.json`，画面在 `scene-smoke/`。

当前风险后端是 Warp CUDA，动作/风险/RGB 来源状态为 30 Hz。旧提交 `acd515c` 的 779 帧采集保留在 `datasets/risk/local_human_run01/`，不能当作新版 Docker 完整采集结果。

## 原生环境备份

原生环境及之前新增的本机适配文件已移到 `.runtime/native-backup/20261022/`，原生历史说明在其 `local-adaptations/LOCAL_RUN.md`。这些备份不作为当前启动入口；原生虚拟环境移动后需要恢复原路径或重新绑定才能使用。

之前补齐的三张 NVIDIA 官方材质贴图仍保留为本机资产，校验清单和恢复脚本随原生适配文件归档。原仓库资产清单未改动。
