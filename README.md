# RoboLaw：厨房任务安全增广

本仓库发布 Isaac Sim / IsaacLab-Arena 厨房任务的场景与采集代码。原始 Lightwheel 范例为 DROID 机械臂抓起芥末瓶并放入碗中，扩展了 **6 个财产障碍、4 个机器人周边障碍、1 个默认人体场景**；后续 Replicator 扩展覆盖 14 个原始任务、154 个场景。

本仓库**不发布风险评估系统**：没有 HS/PT/RS/IR 分级器、指标规则或风险评级结果。任务执行所需的碰撞规划、实际轨迹间隙检查和采集验证仍保留；这些检查不等于风险等级评价。

## Replicator 154 景：逐景成功与失败

截至 2026-10-28，**154 景全部完成规划；首轮真实物理执行为 70 成功、84 任务失败、0 执行错误**，每景均有实际视频。U 型香蕉 11 景另作相机补录，结果为 7 成功、4 失败，单独记录，不改首轮成绩。

**[查看全部 154 景成功／失败表](docs/extensions/replicator/PHYSICAL_EXECUTION_154_20261028.md)** · [机器可读执行结果 JSON](configs/extensions/replicator/physical_execution_154_20261028.json) · [发布说明与输入恢复](docs/extensions/replicator/RELEASE_154_PLANNED_SCENES.md)

Git 保存逐景状态与证据 SHA；规划输入和视频另行打包，原始物理状态与接触记录保留本机，不随代码提交。文档中的本机视频路径不是 GitHub 在线视频链接。实际任务成功不等于安全避障验收。

## 原始 Lightwheel 11 个场景

| 类别 | 场景 ID | 障碍物 |
| --- | --- | --- |
| 财产障碍 | `property_tall_glass` | 高水杯 |
| 财产障碍 | `property_wine_glass` | 高脚酒杯 |
| 财产障碍 | `property_glass_bottle` | 带盖玻璃瓶 |
| 财产障碍 | `property_ceramic_mug` | 陶瓷杯 |
| 财产障碍 | `property_vase` | 陶瓷花瓶 |
| 财产障碍 | `property_tablet_stand` | 带底座的平板支架 |
| 机器人周边障碍 | `robot_steel_clamp` | 固定钢夹具 |
| 机器人周边障碍 | `robot_projecting_crossbar` | 突出横杆 |
| 机器人周边障碍 | `robot_narrow_opening` | 刚性窄口 |
| 机器人周边障碍 | `robot_sharp_equipment_housing` | 带突出斜面的设备外壳 |
| 默认人体 | `human_default_mpfb` | MPFB 静态成人，伸手站在工作区旁 |

10 个非人体场景的独立 USD 入口在 [`scenes/obstacle_library`](scenes/obstacle_library)，逐场景参数在 [`configs/obstacle_library`](configs/obstacle_library)，障碍物 USD 与分类清单在 [`assets/obstacle_library`](assets/obstacle_library)。默认人体入口是 [`scenes/human_mpfb.usda`](scenes/human_mpfb.usda)，摆放和资产路径由 [`configs/layout.json`](configs/layout.json) 指定。`human_default_mpfb` 是此发布包的场景 ID，不属于 10 项障碍库的 catalog。

## 文件放在哪里

| 位置 | 内容 | 获取方式 |
| --- | --- | --- |
| [GitHub：`youlj109/RoboLaw`](https://github.com/youlj109/RoboLaw) | 11 个场景入口、10 个小型障碍 USD、配置、采集与恢复脚本、许可说明 | `git clone` |
| [魔搭数据集：`YouLinjing/Isaac-Arena-Kitchen-Safety`](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety) 的 `robolaw/` 目录 | Lightwheel 厨房与默认 MPFB 人体的大文件资产包 | `python3 scripts/restore_modelscope_assets.py` |
| NVIDIA 官方资产服务器 / 魔搭 `robolaw/` 镜像 | DROID 机器人、支座、芥末瓶、碗、材质及 cuRobo YAML/URDF，共 13 个文件 | `python3 scripts/restore_downloaded_assets.py --source official` 或 `--source modelscope` |
| 上游官方 GitHub / 魔搭 `robolaw/` 镜像 | IsaacLab-Arena、Isaac Lab 和 cuRobo 固定版本源码 | `python3 scripts/fetch_source_dependencies.py --source official` 或 `--source modelscope` |

### 魔搭资产包的准确位置

- [资产包 `robolaw-assets-v1.tar.gz`](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/robolaw-assets-v1.tar.gz)：**79,022,015 字节**，SHA-256：`6d16d4320f8b75d4382edf51fa047e5808907b6352e93a4dc27924e43489530a`。
- [逐文件清单 `manifest.json`](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/manifest.json)：列出归档内每个文件的相对路径、字节数和 SHA-256。
- [魔搭侧许可说明](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/THIRD_PARTY_NOTICES.txt)。

归档共 **56 个文件**：`assets/kitchen/` 下 48 个文件（原始约 72.1 MB），默认人体的 `person.usdc`、贴图和 Blender 源文件共 8 个（原始约 25.6 MB）。解压后的路径与 USD 引用保持一致。GitHub 当前提交不包含这些二进制资产；它们由恢复脚本下载、核对归档哈希、检查归档路径，再逐文件校验并写入对应目录。恢复脚本可以重复运行，已校验的文件会跳过。

```bash
python3 scripts/restore_modelscope_assets.py
python3 scripts/restore_modelscope_assets.py --verify-only
```

也可手工下载上面的资产包和清单；建议仍用恢复脚本安装，避免解压到错误层级。恢复脚本使用 Python 标准库，不要求安装魔搭 SDK 或提供 Token。已经在空目录中从魔搭公开地址重新下载、解压并通过 56 个文件的校验。

## 官方与魔搭两种获取方式

**下面两条路径二选一**。二者使用相同的 13 个 NVIDIA 资产文件和相同的三套源码修订，恢复位置相同。资产逐文件校验 SHA-256；魔搭源码包同时校验归档和逐文件清单。`--source` 可单独选择来源；环境变量 `ROBOLAW_DOWNLOAD_SOURCE` 会将选择传给后续安装步骤，默认值为 `official`。

### 路径 A：从 NVIDIA 官方和官方 GitHub 下载

```bash
git clone https://github.com/youlj109/RoboLaw.git
cd RoboLaw
export ROBOLAW_DOWNLOAD_SOURCE=official

# 厨房及默认人体仍使用项目的魔搭资产包
python3 scripts/restore_modelscope_assets.py
# NVIDIA 官方服务器：机器人、支座、瓶子、碗、材质、YAML/URDF
python3 scripts/restore_downloaded_assets.py --source official
# 官方 GitHub：Arena、Isaac Lab、cuRobo 的固定版本
python3 scripts/fetch_source_dependencies.py --source official
```

官方资产的逐文件 URL、大小和 SHA-256 见 [`configs/external_assets.json`](configs/external_assets.json)。源码分别来自 [IsaacLab-Arena](https://github.com/isaac-sim/IsaacLab-Arena)、[Isaac Lab](https://github.com/isaac-sim/IsaacLab) 和 [cuRobo](https://github.com/NVlabs/curobo)。源码方式使用 HTTPS Git 拉取指定提交，无需 GitHub SSH 密钥。

### 路径 B：从魔搭下载资产与源码

```bash
git clone https://github.com/youlj109/RoboLaw.git
cd RoboLaw
export ROBOLAW_DOWNLOAD_SOURCE=modelscope

python3 scripts/restore_modelscope_assets.py
python3 scripts/restore_downloaded_assets.py --source modelscope
python3 scripts/fetch_source_dependencies.py --source modelscope
```

镜像都位于 [魔搭 `YouLinjing/Isaac-Arena-Kitchen-Safety`](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety) 的 **`robolaw/`** 目录，公开下载无需 Token。源码包不含 `.git` 历史，可直接用于安装。保留上游 LICENSE/NOTICE；本工程运行时的本地修改不混入上游源码快照。cuRobo 与原安装脚本一样，不包含 `images/` 和本任务不用的 `src/curobo/content/assets/` 示例资源；本任务的 DROID 模型和 YAML/URDF 由 13 文件资产包提供。Arena 不包含文档动画/图片目录 `docs/images/` 和测试数据目录 `isaaclab_arena/tests/test_data/`、`isaaclab_arena_gr00t/tests/test_data/`，官方方式也使用相同的精简范围；Isaac Lab 不包含部分文档预览媒体、测试基准图像/场景和测试资产数据，逐项排除路径记录在 `configs/dependency_mirrors.json` 的 `excluded_paths`，其余源码及配置保留。Arena 的 Isaac Lab 子模块单独提供；本任务未使用的 Isaac-GR00T 子模块不在包内。

| 魔搭补充包 | 固定版本 | 压缩字节数 | 条目数 |
| --- | --- | ---: | ---: |
| [机器人、瓶子、碗等 13 个 NVIDIA 文件](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/robolaw-nvidia-assets-v1.tar.gz) | `Isaac 6.1 asset paths` | 29,020,619 | 13 |
| [IsaacLab-Arena 源码](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/robolaw-arena-7050ecbc9b3f.tar.gz) | `7050ecbc9b3fc689df51ea7cc7f9a64b18294bf6` | 1,535,586 | 1,126 |
| [Isaac Lab 源码](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/robolaw-isaaclab-91a0906f9380.tar.gz) | `91a0906f93802eb859192dcb0bca879f891e92d6` | 34,989,667 | 3,952 |
| [cuRobo 源码](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/robolaw-curobo-ebb71702f3f7.tar.gz) | `ebb71702f3f70e767f40fd8e050674af0288abe8` | 457,479 | 299 |

每个包均有相邻的 `.manifest.json` 逐文件清单。完整提交号、URL、包大小、包与清单 SHA-256 固定在 [`configs/dependency_mirrors.json`](configs/dependency_mirrors.json)。手工下载时应按清单将内容解压到项目根目录；建议使用上述脚本，以自动完成路径及内容校验。

恢复后的目录：

```text
assets/nvidia/                               # DROID、支座及其引用资源
assets/robolab/                              # 芥末瓶、碗及贴图
assets/curobo/droid/                         # DROID 运动学与碰撞配置
third_party/IsaacLab-Arena/                   # Arena
third_party/IsaacLab-Arena/submodules/IsaacLab/ # Isaac Lab
third_party/curobo/                          # cuRobo
```

仅检查已恢复文件，不重新下载：

```bash
python3 scripts/restore_downloaded_assets.py --source "$ROBOLAW_DOWNLOAD_SOURCE" --verify-only
python3 scripts/fetch_source_dependencies.py --source "$ROBOLAW_DOWNLOAD_SOURCE" --verify-only
```

重复运行会校验并复用已有文件。发现已存在的资产或源码与固定版本不一致时，脚本报错并保留原文件，请使用新目录或先备份修改文件。切换下载来源不会自动替换版本。

## 环境和安装

建议使用 Linux x86-64、支持 RTX 渲染的 NVIDIA GPU、Docker Engine 与 NVIDIA Container Toolkit。当前源工程运行于 RTX 5090。**魔搭路径覆盖上面列出的场景资产和三套源码，不是完整离线安装包**：Docker 基础镜像、Isaac Sim/Kit、CUDA、Python/PyPI 依赖以及 OpenPI 客户端仍需访问各自的软件源。

完成上面任意一条获取路径后，在同一个 shell 中执行：

```bash
export ARENA_CONTAINER=robolaw-arena
# 阅读 NVIDIA Isaac Sim 许可后执行；此参数表示接受该许可
bash scripts/setup_arena.sh --accept-eula

bash scripts/arena_python.sh scripts/check_runtime.py
python3 scripts/check_scene_assets.py --require-restored
```

`setup_arena.sh` 创建项目容器与 Python 3.12 环境，安装采集依赖并编译 cuRobo；资产和 cuRobo 源码步骤会使用 `ROBOLAW_DOWNLOAD_SOURCE` 指定的来源。若单独开了新终端，请重新设置该变量。新机器需要为镜像、依赖和输出预留充足空间。若已有同名容器挂载其他工程，请换一个 `ARENA_CONTAINER` 名称。

已有 `fetch_arena_source.sh` 入口继续可用：`bash scripts/fetch_arena_source.sh --source modelscope` 或 `--source official`，只获取 Arena 与 Isaac Lab；cuRobo 可通过 `python3 scripts/fetch_source_dependencies.py --components curobo --source modelscope` 单独恢复。

## 查看命令与重新采集

先查看即将执行的命令；此步骤不会启动 Isaac Sim，也不会新建采集目录：

```bash
python3 scripts/run_robolaw_scenes.py --dry-run
python3 scripts/run_robolaw_scenes.py --scene property_vase --dry-run
```

环境安装完成后，运行全部 11 个场景，或只运行指定场景：

```bash
bash scripts/arena_python.sh scripts/run_robolaw_scenes.py
bash scripts/arena_python.sh scripts/run_robolaw_scenes.py --scene property_vase
bash scripts/arena_python.sh scripts/run_robolaw_scenes.py --scene human_default_mpfb
```

运行器按顺序启动 Kit 进程，每次默认创建新的 `datasets/robolaw/<UTC时间戳>/`，也可用 `--output` 指定新的输出目录。非人体批次写入 `nonhuman/manifest.json` 和逐次的 `result.json`、`verification.json`、`audit.json`；人体运行写入 `human_default_mpfb/result.json`、`validation.json`、`audit.json`。使用 `--video` 录制原三路相机。请查看本次输出文件，而不要将旧轨迹当作新运行结果。同一 GPU/容器上一次只运行一项 Kit 采集。

## 已有证据与限制

源工程在 **2026-10-17** 对 10 个非人体固定布局按 seed 42、零扰动完成过实体抓取与放置，并通过数据和间隙检查。这仅说明这些**固定配置**当时可行，不是随机化成功率。默认人体也曾在源工程中采集；本 GitHub 新检出的独立运行命令尚未完成一次完整仿真复现，因此不能宣称这 11 条在任意新机器上都能逐帧成功。GPU、驱动、Isaac Sim 与资产版本变化可能影响结果。

本次发布已检查：GitHub 的 11 个场景入口与本地小文件清单、厨房和人体的 56 文件资产包恢复、采集命令的 dry-run。2026-10-21 已从魔搭公开 URL 在空目录恢复新增的 4 个依赖包，并通过归档和逐文件哈希校验；另从官方拉取三套固定源码及下载 13 个资产文件，与魔搭恢复结果逐项比较，5,377 个源码文件/符号链接和 13 个资产均一致。已检查重复恢复、环境变量选择来源、旧脚本入口、路径越界拒绝和修改文件保护。此次未重新安装完整仿真环境或运行完整采集。没有随仓库提供现成视频或安全风险等级。

## 许可与来源

厨房来自 Lightwheel AI 的 RoboCasa one-wall coastal 布局，按 **CC BY-NC 4.0** 用于非商业研究复现，并可能受单项资产条款约束。默认人体使用 MPFB / MakeHuman Community 的生成与系统资产；MPFB 代码和生成资产适用不同条款，详见 [`assets/people/SOURCES.txt`](assets/people/SOURCES.txt)。NVIDIA 资产可从官方原站或魔搭镜像获取，其原始许可保持有效；Isaac Sim/Kit 运行时仍从官方获取。请在再发布或商用前阅读 [`THIRD_PARTY_NOTICES.txt`](THIRD_PARTY_NOTICES.txt)；仓库根目录的 MIT `LICENSE` 不会将第三方资产改授 MIT 许可。
