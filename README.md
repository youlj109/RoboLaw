# RoboLaw：DROID 芥末瓶抓放任务的 11 个场景

本仓库发布同一个 Isaac Sim / IsaacLab-Arena 厨房任务的场景与采集代码：DROID 机械臂抓起芥末瓶，将其放入碗中。共 **6 个财产障碍、4 个机器人周边障碍、1 个默认人体场景**。任务起终点、机器人和原有三路相机在这些场景中保持一致；改变的是场景里的障碍物。

本仓库**不发布风险评估系统**：没有 HS/PT/RS/IR 分级器、指标规则、风险结果或历史轨迹和视频。任务执行所需的碰撞规划、实际轨迹间隙检查和采集验证仍保留；这些检查不等于风险等级评价。

## 11 个场景

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
| NVIDIA 官方资产服务器 | DROID 机器人、支座、芥末瓶、碗、材质及 cuRobo YAML/URDF | 安装过程中由 `scripts/restore_downloaded_assets.py` 获取 |
| 上游 GitHub 仓库 | IsaacLab-Arena、Isaac Lab 和 cuRobo 源码 | 安装脚本按固定修订获取 |

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

## 环境和安装

建议使用 Linux x86-64、支持 RTX 渲染的 NVIDIA GPU、Docker Engine 与 NVIDIA Container Toolkit；需要能访问 GitHub、魔搭、NVIDIA NGC/资产服务器及 Python 包源。当前源工程运行于 RTX 5090。Isaac Sim、容器镜像、Python 环境和 CUDA 扩展体积较大；本仓库不提交这些运行时。

从新目录开始：

```bash
git clone https://github.com/youlj109/RoboLaw.git
cd RoboLaw

# 先恢复魔搭上的厨房与人体资产
python3 scripts/restore_modelscope_assets.py

# 获取固定修订的 Arena / Isaac Lab 源码
bash scripts/fetch_arena_source.sh

# 与源工程使用不同的容器名；在同一个 shell 中保留此变量
export ARENA_CONTAINER=robolaw-arena

# 阅读 NVIDIA Isaac Sim 许可后执行；此参数表示接受该许可
bash scripts/setup_arena.sh --accept-eula

# 不启动 Kit 的运行环境检查、全部资产入口检查
bash scripts/arena_python.sh scripts/check_runtime.py
python3 scripts/check_scene_assets.py --require-restored
```

`setup_arena.sh` 创建项目容器与 Python 3.12 环境，安装采集依赖和 cuRobo，并从 NVIDIA 官方地址恢复另 13 个必需文件。新机器需要为镜像、依赖和输出预留充足空间。若已有同名容器挂载其他工程，请换一个 `ARENA_CONTAINER` 名称。官方资产路径与来源见 [`configs/external_assets.json`](configs/external_assets.json)。

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

本次发布已检查：GitHub 的 11 个场景入口与本地小文件清单、魔搭归档公开下载及 SHA-256、从空目录恢复全部 56 个大文件、采集命令的 dry-run。没有随仓库提供现成视频或安全风险等级。

## 许可与来源

厨房来自 Lightwheel AI 的 RoboCasa one-wall coastal 布局，按 **CC BY-NC 4.0** 用于非商业研究复现，并可能受单项资产条款约束。默认人体使用 MPFB / MakeHuman Community 的生成与系统资产；MPFB 代码和生成资产适用不同条款，详见 [`assets/people/SOURCES.txt`](assets/people/SOURCES.txt)。NVIDIA 二进制资产不在本仓库或魔搭 RoboLaw 包内，而从 NVIDIA 原站下载。请在再发布或商用前阅读 [`THIRD_PARTY_NOTICES.txt`](THIRD_PARTY_NOTICES.txt)；仓库根目录的 MIT `LICENSE` 不会将第三方资产改授 MIT 许可。
