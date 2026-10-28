# RoboLaw / huirongzeng：厨房任务与风险感知

本分支包含原有芥末瓶入碗任务的 **6 个财产障碍、4 个机器人周边障碍、1 个默认人体场景**，以及扩展到 **9 种厨房、31 个任务**的原生 HS/PT/RS 适配代码。当前交付目标为安全评估体系正确性：原始证据、资产关系、离线特征与 HS/PT/RS 判级；无需每组采集 200 条。已完成逐任务短程链路和代表性运动验证，全部风险变体布局仍未逐项验收。

当前分支 **`huirongzeng`** 包含 HS、PT v4、RS、IR 的风险特征采集、评价规则、SafeBench 参考评价器、gDist CUDA 完整三角网格测距、任务豁免、逐步证据记录、验证报告、LeRobot 风险标签和三视角风险视频导出。等级仅按已知有效分项聚合，覆盖完整性和缺测独立保留；当前暂不启用 IR：仅 HS/PT/RS 参与评价、汇总和视频展示，IR 保留为可恢复模块。

默认采用 **必要证据在线采集、风险与媒体离线处理**，适用于全部已配置场景。每批传入自己的场景规格和采集参数，当前人体场景只作为验证案例。策略保存于 [`configs/risk_capture_workflow.json`](configs/risk_capture_workflow.json)。

```bash
# 场景文件由本次场景生成器提供；不固定人体场景 ID
bash scripts/arena_python.sh scripts/risk_capture_workflow.py capture \
  --episode-specs datasets/my_scene_specs.json \
  --collector-overrides datasets/my_collector_overrides.json \
  --output datasets/risk/my_run --run-id my_run
# 采集后，单独校验并提取特征、评价 HS/PT/RS
bash scripts/arena_python.sh scripts/risk_capture_workflow.py evaluate \
  --output datasets/risk/my_run
```

风险审计、三路 RGB 渲染和视频导出作为单独离线阶段运行，见 **[通用在线/离线工作流](docs/offline_risk_workflow.md)**。原始采集完成保持 pending_offline，不能当作风险标注或生产准入已经完成。历史独立 clearance `audit` 不默认启动，它可能重新查询已明确移除的距离。

完整的规则范围、当前豁免、采集输出、视频导出及已知限制见 **[风险感知安装与复现说明](docs/risk_perception.md)**。实验阈值与部分缺测仍然存在，不应把已知 L0 解释为全部证据完整。

## 原生任务扩展（当前 31 项；2026-10-24 历史探针为 32 项）

任务清单为 21 个 pick-and-place 和 11 个 articulation；半岛式香蕉入碗按用户确认加入。每个任务声明 12 种场景配置，共 384 个配置组合。**每组 200 条已取消作为交付要求，不开展正式批量采集**；只执行必要的正确性验证。

- **历史证据（2026-10-24）**：32/32 任务完成短程原始采集、审计和 gDist 离线评估接口检查；379 个官方依赖文件逐项校验通过。
- **新增能力**：独立多物体/关节状态、任务角色与操作面绑定、任务相对障碍放置、离线外观随机化、可配置视角、单 episode 终止证据与视频溯源导出。
- **正确性验证**：82 项规则边界/缺测检查、24 项故障时序检查；门、按钮、旋钮实际运动及人体关系诊断；10 种障碍真实模型的原始证据、网格距离与规则路由检查。
- **明确的缺口**：操纵资产的停止模型尚未标定，部分权威故障/损伤输入不可用；对应分项保持未知或原定审计状态。10 种障碍均已有通过的代表性布局，横杆与尖角外壳使用单独验证的 G 型模板；384 个组合不等于全部完成运行验收。完整任务控制器成功率、每组 200 条及全量视频不属于本次要求。

先用 [统一任务/变体准备入口](docs/native_task_variants.md) 选择任务与风险资产，并提供本任务放置配置；当前启用 31 项任务；历史 32×12 输入准备记录保留，不自动采集或声称布局通过。

入口见 [原生任务在线/离线工作流](docs/native_task_workflow.md)，逐项证据见 [扩展进度](docs/task_suite_expansion.md) 与 [验收状态 JSON](docs/task_suite_validation_status.json)。[障碍放置](docs/task_variant_placement.md) 和 [离线外观/相机](docs/native_offline_appearance.md) 均绑定本条任务和真实来源，不能套用旧场景世界坐标或豁免表。

直接离线评价已加入 [强制原始证据预检](docs/native_evaluation_preflight.md)，即使未单独运行 audit，也会在 GPU 测距前核对时钟、接触和命令证据。

最新模型核验结果见 [风险资产接口检查](docs/native_risk_asset_probes.md)。

本轮 384 项资产关系组合检查全部通过，未新增仿真采集。验证范围和缺测边界见 [安全评估体系验证说明](docs/safety_system_verification.md)；合成拓扑检查不代表 384 个布局均已实跑。

## 可移植 Skill 与数据规范

三份完整 Skill 目录位于 [`skills/`](skills/使用说明.md)，可复制到其他 agent。需要保留各目录的 `references`、`assets`、`scripts` 和元数据；不只复制 `SKILL.md`。

| 内容 | 入口 |
| --- | --- |
| HS/PT 资产范围与正常任务关系 | [robot-risk-asset-scope](skills/robot-risk-asset-scope/SKILL.md) |
| 必要原始证据在线采集与 gDist 离线流水线 | [robot-risk-offline-pipeline](skills/robot-risk-offline-pipeline/SKILL.md) |
| HS 动态规则、PT v4、RS 与已知结果聚合 | [robot-risk-hs-pt-rs-rules](skills/robot-risk-hs-pt-rs-rules/SKILL.md) |
| 数据输出标准格式 | [在线阅读](docs/standards/机器人风险数据输出标准格式.md) · [Word](docs/standards/机器人风险数据输出标准格式.docx) · [PDF](docs/standards/机器人风险数据输出标准格式.pdf) |
| 原始数据采集与离线安全评估规范 | [在线阅读](docs/standards/机器人原始数据采集与离线安全评估规范.md) · [Word](docs/standards/机器人原始数据采集与离线安全评估规范.docx) · [PDF](docs/standards/机器人原始数据采集与离线安全评估规范.pdf) |

通用规范覆盖 pick-and-place、开门、抽屉等 articulation 任务的适配要求。每个任务建立自己的角色、碰撞候选、正常操作关系、动态连杆和规则绑定，不能照搬当前厨房的资产排除。数据输出文档定义的是通用交换契约，现有工程 schema 的对应关系需通过适配器实现；这不表示所有新任务和字段已经实测支持。

Skill 包含冻结规则源、58 个既有边界案例和 gDist 参考构建材料。上游 gDist 从固定提交获取，新硬件重新构建；不包含仿真器、GPU 二进制、密码、视频或原始轨迹。

## 当前人体案例的范围与已有验证

2026年10月23日用户确认的新范围：

- **HS**：移除指定休闲服和固定底座/支座的 HS 计算与保存；17 个运动部件及芥末瓶对剩余 5 个人体网格，共 90 个动态关系。见 [HS 数据排除](docs/hs_data_omissions.md)。
- **PT**：停止 81 项距离关系的查询和保存，保留 6 项几何关系；其中机器人到芥末瓶仅供抓持证据，5 项可参与 PT 距离判级。接触、力、冲量、掉落及共享重建证据按自身范围保留。见 [PT 距离排除](docs/pt_distance_omissions.md)。
- **RS**：独立完整机器人几何不随 HS 裁剪。IR 关闭，缺测仍未知。

最新已有案例模拟 76.4 秒、9,168 个物理步、2,292 帧，任务成功；在线风险几何和等级调用均为 0。HS/PT/RS 几何完整，但类别证据完整性仍有缺口。采集墙钟 321.85 秒，离线处理 131.98 秒，其中 geometry 38.66 秒。完整性、等级分布、缺测和分阶段计时见 [轻量验证记录](validation/hs_pt_latest_review_20261023/README.md)。上述历史案例未在本轮重新采集；本轮新增原生任务探针见前节。视频和原始数据仅保存在远端与本地。

## 默认 gDist 后端

默认 HS/PT/RS 距离后端已更新为 **gDist CUDA**。新数据冻结真实后端、参数与构建哈希；历史数据保留原后端，显式重算写入独立目录。安装与迁移命令见 **[gDist 后端说明](docs/gdist_backend.md)**。当前批量接口提供完整网格距离和最近碰撞体路径，最近点坐标明确标为不可用；当前风险等级和视频距离依据不依赖这些坐标。

## 当前 HS 距离规则

默认已启用 `arena-hs-dynamic-ssm-1`：按每个运动部件／搬运物与人体网格的完整表面距离、真实 COM 线速度和角速度，离线计算动态保护距离 S 与安全余量 D−S，再汇总已知关系的最高等级。主导关系取最小余量；当前人体案例已进一步停止固定支座和指定休闲服的 HS 查询、接触点提取与逐帧审计，RS 仍保留独立完整机器人几何，TTC 只作诊断。详见 **[动态 HS 公式、参数和缺测策略](docs/hs_dynamic_distance.md)**。

停止减速度等目前是**未标定实验参数**；本系统用于离线标注，不是认证的在线 SSM 安全控制。较早的人体短段验证含 64 个动作／256 个物理步，主导关系保护距离随速度从 2.50 cm 变至 3.165 cm，在线几何与风险判级次数均为 0；102/108 个关系有已知结果，未受控瓶子的 6 个人体关系缺少停止模型，仍保留未知。该短段不是完整任务成功验证；其 108 对范围已经由下述新范围取代。最新完整案例为 2,292 帧，见 [已有验证记录](validation/hs_pt_latest_review_20261023/README.md)。旧归档继续用各自冻结的历史规则。

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

### Replicator 154 景原始轨迹输入包

部署 `ylj` 分支并重放 Replicator 原始轨迹时，还需下载独立的[规划输入包 `replicator_154_planning_inputs_20261028.zip`](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/replicator_154_planning_inputs_20261028.zip)。`exports/` 被 Git 忽略，拉取代码不会自动获取此包。

包大小为 **9,740,639 字节（9.74 MB）**，SHA256 为 `32563dc7d829d1131c1541858b11c255318c618fc308ae0b6f21bcca43d631cd`。2026-10-28 已从魔搭公开地址无凭据下载，并验证 SHA256 与 ZIP CRC。下载、解压及逐文件校验命令见 [Replicator 发布说明](docs/extensions/replicator/RELEASE_154_PLANNED_SCENES.md#快速核对与查看)。包内包含轨迹和规划输入，不包含大型源资产、完整运行时或物理视频；源资产仍按安装说明恢复。

## 官方与魔搭两种获取方式

**下面两条路径二选一**。二者使用相同的 13 个 NVIDIA 资产文件和相同的三套源码修订，恢复位置相同。资产逐文件校验 SHA-256；魔搭源码包同时校验归档和逐文件清单。`--source` 可单独选择来源；环境变量 `ROBOLAW_DOWNLOAD_SOURCE` 会将选择传给后续安装步骤，默认值为 `official`。

### 路径 A：从 NVIDIA 官方和官方 GitHub 下载

```bash
git clone --branch huirongzeng https://github.com/youlj109/RoboLaw.git
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
git clone --branch huirongzeng https://github.com/youlj109/RoboLaw.git
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

## 场景采集入口

风险采集使用 [`scripts/risk_capture_workflow.py`](scripts/risk_capture_workflow.py)。输入各场景自己的 episode specs，可选传入 collector overrides；同一批次为同资产拓扑，不同场景分别使用运行目录。物理与不可重建证据在线保存，后续 evaluate / audit / render / video 全部离线执行。完整步骤和各场景验证边界见 [通用工作流说明](docs/offline_risk_workflow.md)。

## 已有证据与限制

源工程在 **2026-10-17** 对 10 个非人体固定布局按 seed 42、零扰动完成过实体抓取与放置，并通过数据和间隙检查。这仅说明这些**固定配置**当时可行，不是随机化成功率。默认人体也曾在源工程中采集；本 GitHub 新检出的独立运行命令尚未完成一次完整仿真复现，因此不能宣称这 11 条在任意新机器上都能逐帧成功。GPU、驱动、Isaac Sim 与资产版本变化可能影响结果。

本次发布已检查：GitHub 的 11 个场景入口与本地小文件清单、厨房和人体的 56 文件资产包恢复、采集命令的 dry-run。2026-10-21 已从魔搭公开 URL 在空目录恢复新增的 4 个依赖包，并通过归档和逐文件哈希校验；另从官方拉取三套固定源码及下载 13 个资产文件，与魔搭恢复结果逐项比较，5,377 个源码文件/符号链接和 13 个资产均一致。已检查重复恢复、环境变量选择来源、旧脚本入口、路径越界拒绝和修改文件保护。此次未重新安装完整仿真环境或运行完整采集。风险感知分支的预检、18 项参考评价器测试和历史适配器测试夹具问题见 [发布验证记录](docs/risk_perception.md#发布验证与已知限制2026-10-21)。

## 风险模块与参考实现

| 目录/文件 | 内容 |
| --- | --- |
| `kitchen_safety/risk_*.py` | 配置绑定、时序观察器、报告、标签导出 |
| `kitchen_safety/fcl_distance.py`、`hs_geometry.py`、`pt_geometry.py` | Warp 完整网格距离与几何覆盖（保留历史适配文件名） |
| `kitchen_safety/pt_assets.py`、`grasp_evidence.py`、`rs_evidence.py` | 资产关系、抓持、自碰撞及故障证据 |
| `configs/safety_observer_arena.json`、`configs/safety/` | 当前 Arena 规则绑定和 profile |
| `third_party/safebench_reference/` | 随分支提供的参考评价器、测试及采样说明 |
| `scripts/export_risk_review_video.py` | 简化三视角、等级、指标和依据视频 |

GitHub 保存源码与小配置。场景大资产及上游依赖镜像继续放在前述魔搭目录；历史原始采集库、备份、完整 reports 和运行环境不在 Git 分支中。

## 许可与来源

厨房来自 Lightwheel AI 的 RoboCasa one-wall coastal 布局，按 **CC BY-NC 4.0** 用于非商业研究复现，并可能受单项资产条款约束。默认人体使用 MPFB / MakeHuman Community 的生成与系统资产；MPFB 代码和生成资产适用不同条款，详见 [`assets/people/SOURCES.txt`](assets/people/SOURCES.txt)。NVIDIA 资产可从官方原站或魔搭镜像获取，其原始许可保持有效；Isaac Sim/Kit 运行时仍从官方获取。请在再发布或商用前阅读 [`THIRD_PARTY_NOTICES.txt`](THIRD_PARTY_NOTICES.txt)；仓库根目录的 MIT `LICENSE` 不会将第三方资产改授 MIT 许可。


## ylj 场景/规划与当前风险流水线融合（2026-10-28）

场景和规划以 `ylj` 的 Replicator 实现为准，风险特征、HS/PT/RS 和资产排除以本分支为准。不得由场景全资产清单或规划器碰撞屏蔽自动扩充风险范围。原始状态/接触/命令在线保存；gDist 完整网格测距、判级与视频离线。IR 保持关闭。

- [融合入口、范围锁与验证](docs/ylj_risk_integration.md)
- [ylj 的 154 景规划与执行说明](docs/extensions/replicator/PHYSICAL_EXECUTION_154_20261028.md)

上游 154 景任务执行成绩与本分支风险评估结果分别记录，不互相替代。
