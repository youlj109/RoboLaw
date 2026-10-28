# Replicator 154 景规划结果发布说明

截至 2026-10-28，14 个 Replicator 原始任务 × 11 种安全增广变体，共 **154 景已生成完整 cuRobo 轨迹，0 景规划失败**。每条完整轨迹包含预抓取、接近、抬升、转运、下放和撤离六阶段。

首轮真实 PhysX 执行已全部完成：**154 景实际执行、70 成功、84 任务失败、0 执行错误，保存 154 个真实视频**。首轮每景一次，失败也保留；U 型香蕉 11 景因首轮相机遮挡，另各真实补录一次，补录也已完成（7 成功、4 失败）。最终演示包已生成，包含 154 景主视频和 11 个原遮挡视频，全部 165 个视频通过完整解码与来源校验。首轮与补录成绩分开记录，不用规划轨迹、静态截图或轨迹动画计作物理任务成功。逐景结果见本机 [物理执行面板](http://127.0.0.1:8765/#physicalWorkspace)，安全避障和长期稳定性未单独验收。

## 首轮物理执行统计

统计来自 `exports/replicator_154_physical_demos_20261028/manifest.json`，每个原始任务均有 11 个变体、11 次实际执行和 11 个视频。任务标识省略共同前缀 `kitchen_bench_replicator_`；成功依据原始任务判据，补录不改动下表。

| 原始任务 | 成功 | 任务失败 | 执行错误 | 真实视频 |
| --- | ---: | ---: | ---: | ---: |
| `g_shape_banana_bowl` | 3 | 8 | 0 | 11 |
| `g_shape_bowl_sink` | 4 | 7 | 0 | 11 |
| `g_shape_mustard_bowl` | 1 | 10 | 0 | 11 |
| `l_island_banana_bowl` | 6 | 5 | 0 | 11 |
| `l_island_bowl_sink` | 4 | 7 | 0 | 11 |
| `l_island_mustard_bowl` | 8 | 3 | 0 | 11 |
| `l_shape_banana_bowl` | 6 | 5 | 0 | 11 |
| `l_shape_bowl_sink` | 2 | 9 | 0 | 11 |
| `l_shape_mustard_bowl` | 2 | 9 | 0 | 11 |
| `peninsula_bowl_sink` | 7 | 4 | 0 | 11 |
| `peninsula_mustard_bowl` | 5 | 6 | 0 | 11 |
| `u_shape_banana_bowl` | 7 | 4 | 0 | 11 |
| `u_shape_bowl_sink` | 7 | 4 | 0 | 11 |
| `u_shape_mustard_bowl` | 8 | 3 | 0 | 11 |
| **合计** | **70** | **84** | **0** | **154** |

首轮独立原生执行审计已通过：154 次实际 episode 与视频来源绑定、原规划指令、连续 120 Hz 物理状态和原始任务判据均已核对，70 项成功都有实际判据支持。报告保存在 `reports/replicator_physical_execution_20261028/completed_execution_audit.json`，按仓库规则不随 Git 提交。11 次补录也通过独立审计，首轮清单保持原字节。最终打包器另行完成全部 165 视频解码与 ZIP 成员 SHA 校验；这些检查不等于安全避障认证。

## 完整索引

使用 [final_154_scenes_20261028.json](../../../configs/extensions/replicator/final_154_scenes_20261028.json) 获取全部 154 景。不要将 `trajectory_retry_v10.json` 当作完整清单：它只包含第十轮最后两景的重试方案。

索引逐景保存：

- `scene_id`、原始 `task_id`、变体及类别。
- `config_path/config_sha256`：该景实际使用的最终配置。
- `source_result_path/source_result_sha256`：R10 目录中的最新结果快照；沿用的成功结果保留原文件字节。
- `original_generation_result`：实际生成该轨迹的原轮次结果位置及 SHA。
- `artifacts.trajectory` 和 `artifacts.trajectory_json`：实际 NPZ、轨迹说明文件的位置、SHA 和大小。
- `source_scene`、来源资产列表、原始机器人关节值、样本数及分阶段样本数。
- 场景质量问题及明确的物理执行未验证标记。

最终配置并非来自单一目录：`visual_scene_variants_final03` 有 85 景，`visual_only_review_v1` 有 51 景，`scene_retry_once_20261028` 有 13 景，`scene_retry_round9_20261028` 有 5 景。应读取逐景 `config_path`，不要把某一版目录批量套用到所有场景。

NPZ 包含实际规划关节序列 `q`、名义 TCP `tcp_world`、阶段 `phase`、夹爪目标 `gripper` 和时间步长 `dt`。`nominal_attachment` 是规划携物假设，不能作为仿真固定物体或证明抓取成功的依据。

## 两项已知场景质量问题

| 场景 | 已知问题 | 当前结论 |
| --- | --- | --- |
| `g_shape_banana_bowl__human_default_mpfb` | 一次 CPU 检查仍发现人体衣服与厨房橱柜源网格相交 | 虽已生成轨迹，场景质量不通过 |
| `u_shape_bowl_sink__robot_narrow_opening` | 障碍底座 AABB 超出台面 AABB 边界 | 支撑质量待复核；AABB 提示不等于已经证实实际悬空 |

这两项不会因轨迹完整或视频录制完成而自动消失。其余 152 景也不能仅凭没有这两项提示就判为安全或避障合格；实际任务结果以上面的首轮统计为准。

## 从 Git 获取什么，从证据包恢复什么

Git 保存代码、技能包、配置、索引和说明。`reports/`、大部分运行资产、轨迹及视频按仓库规则不随代码提交。

单独输入包为 `exports/replicator_154_planning_inputs_20261028.zip`，实际大小 **9,740,639 字节（9.74 MB）**，SHA256 为 `32563dc7d829d1131c1541858b11c255318c618fc308ae0b6f21bcca43d631cd`。包内共 **798 个文件**：原 788 个规划输入加 6 个相机证据文件，共 794 个原路径文件、13,894,697 字节；另有 README、输入 manifest、外部资产 manifest 和 SHA256SUMS 四个元数据文件。

原 788 个输入包括每景的 **配置、R10 结果 JSON、R10 `source_scene.usda`、实际轨迹 NPZ 和轨迹 JSON**（154 × 5 = 770 个），以及 R10 progress、完整最终索引、14 个原任务 YAML、任务 assignment 和一个 1,242 字节的生成站台 USD。**`source_scene.usda` 是执行器 `load_inputs` 的强制输入，不能省略。** 另附新相机强制校验的 3 份报告与 3 张 PNG；相机 override 配置从最新 Git 获取。包内不包含大型源资产、完整运行时、全部网页截图或物理视频；无需恢复各轮全部失败报告。

本包已通过 ZIP CRC、每个成员的 SHA 和原路径字节检查。解压后的 154 景均通过真实执行器 `source_entries/load_inputs` 的 CPU 校验，六份相机证据也由 `load_camera_overrides` 校验；相机配置取当前 Git，21 个未打包直接源资产映射到本机已安装资产，原始 SHA 校验保留。这不表示单独解压本包即可运行，源资产及其网格、纹理依赖仍需恢复。另已执行带新相机参数的真实容器 CLI `--dry-run`，154 景通过，未启动物理或规划。

保留原相对路径及文件字节，按索引 SHA 校验。沿用轨迹的 `artifacts.*.path` 可能指向 R1–R9 的原轮次，因此只复制 R10 的目录不能保证证据自包含。不得为了通过校验改写原始结果或忽略缺失文件。最终视频包独立分发，已附逐景视频与真实执行结果的清单和 SHA，详见下方。

厨房、机器人、任务物体和人体源资产按 [仓库安装与资产恢复说明](../../../README.md) 恢复；Replicator 的额外厨房、香蕉资产入口分别为 `scripts/extensions/replicator/restore_layout_assets.py` 和 `scripts/extensions/replicator/restore_banana.py`。索引顶层 `source_assets` 与 `robot_models` 保存本次实际来源文件 SHA，恢复后以这些记录核对。

两个衍生资产必须保留或准确恢复：

- `assets/extensions/replicator/human_visual_low_reach01/`：3 景使用。保留 `collidable.usda`、`person.usdc`、纹理及姿态/生成来源记录；原默认人体不能直接替代它。
- `assets/extensions/replicator/steel_clamp_reach650/`：12 景使用。保留 USD 和 `derivation.json`，构建器为 `scripts/extensions/replicator/build_extended_steel_clamp.py`。

不同 Blender 或 USD 导出版本重建的文件字节可能不同。若重建资产与已冻结 SHA 不同，应记录为新的数据版本，不能把它冒充本索引绑定的原资产。

## 快速核对与查看

1. 获取 `ylj` 中本次代码、配置与完整索引，按原安装流程准备运行环境和源资产。
2. 将单独提供的输入包放到 `exports/`，在仓库根目录解压并核对 SHA：

   ```bash
   unzip -t exports/replicator_154_planning_inputs_20261028.zip
   unzip -n exports/replicator_154_planning_inputs_20261028.zip
   sha256sum -c PLANNING_INPUTS_SHA256SUMS
   ```

   `-n` 保留已有文件；如 SHA 不符，先核对本地改动和包内版本，不要跳过验证。外部资产清单见 `PLANNING_INPUTS_EXTERNAL_ASSETS.json`，它不枚举 USD 的全部递归依赖。

3. 运行执行入口的 CPU 输入检查。以下命令对应本机已有的 `arena-kitchen-dev` 容器及 `/workspace` 挂载；其他安装可在激活的 Arena 环境中直接运行同一 Python 入口：

   ```bash
   docker exec -w /workspace -e PYTHONPATH=/workspace arena-kitchen-dev \
     /workspace/.venv-arena/bin/python scripts/extensions/replicator/execute_planned_trajectories.py \
     --device cpu --physics-threads 8 \
     --camera-overrides configs/extensions/replicator/physical_demo_camera_overrides_20261028.json --dry-run
   ```

   当前 Arena CLI 默认 headless=true，不支持 `--headless` 参数。此命令已在真实容器 CLI 上验证通过。预期 `cpu_preflight_passed: true`、`scenes: 154`、`physics_launched: false`。此命令在创建 SimulationApp 之前退出，不启动物理、渲染或规划，且严格检查源资产 SHA。

4. 首次启动物理执行（只用于没有执行 manifest 的输出目录）：

   ```bash
   docker exec -w /workspace -e PYTHONPATH=/workspace arena-kitchen-dev \
     /workspace/.venv-arena/bin/python scripts/extensions/replicator/execute_planned_trajectories.py \
     --device cpu --physics-threads 8 \
     --camera-overrides configs/extensions/replicator/physical_demo_camera_overrides_20261028.json \
     --source-run reports/replicator_trajectory_retry10_20261028 \
     --output exports/replicator_154_physical_demos_20261028
   ```

   本机首轮已完成，无需重跑；任何批次有运行项时都不要并行重复启动。`--device cpu` 选择 CPU PhysX，实际相机渲染仍需 GPU/RTX。恢复未完成的批次前，确认上一进程已经退出且 manifest 没有 running 项。推荐使用主机协调器，每个原始任务启动一个全新 Kit 进程，默认最多 11 景，显式使用 8 个 PhysX 线程：

   ```bash
   python3 scripts/extensions/replicator/run_physical_demo_batches.py \
     --camera-overrides configs/extensions/replicator/physical_demo_camera_overrides_20261028.json --dry-run
   python3 scripts/extensions/replicator/run_physical_demo_batches.py \
     --camera-overrides configs/extensions/replicator/physical_demo_camera_overrides_20261028.json
   ```

   协调器的 `--dry-run` **只检查调度**，不等于第 3 步执行器的完整输入检查。协调器要求已有物理 manifest，只消费 pending 景，遇到 running、子进程错误或输入变化就停止，不自动重试或停止其他进程。单次手工恢复仍可在第 4 步执行器命令末尾加 `--resume --limit 11`，但手工限额不会自动保证不跨任务。success/failed/error 不重跑，中断的 running 项需要先核查，不能直接交给协调器。代码或参数变化会拒绝普通恢复，核实后才可在执行器上显式加 `--allow-runtime-revision` 记录新版本；协调器不自动提供该授权。包内 README 是输入包发布时的直接执行器示例，当前分任务恢复优先使用这里的命令。

5. 启动网页：`python3 scripts/extensions/replicator/progress_dashboard.py --port 8765`，打开 [物理执行面板](http://127.0.0.1:8765/#physicalWorkspace)。实际输出在 `exports/replicator_154_physical_demos_20261028/manifest.json`、`executions/` 和 `videos/`。初始化错误 `executed=false` 不算抓放失败或成功；只有真实任务判据通过才记执行成功。最小输入包不包含全部截图、网页历史和标注，新克隆时不保证显示本机历史的全部图片与人工记录。

首轮 154 份真实视频已齐全，可用 `scripts/extensions/replicator/package_trajectory_demos.py` 校验首轮集合。最终供查看的演示已用下面的独立入口生成；失败执行的视频也保留，缺视频时拒绝完整打包，不用规划动画补齐。

### 相机补录与最终演示包

U 型香蕉首轮 11 景的相机高度在天花板范围内，视频遮挡，因此这 11 景在独立目录 `exports/replicator_u_banana_camera_retakes_20261028/` 各再真实执行一次。补录仅改相机与曝光，锁定原布局、控制参数和轨迹；原首轮 manifest、结果与视频不改。**无论补录成功还是失败，11 景全部固定选新录，不按成绩择优。** 网页上方仍统计首轮，补录另列已保存数量，视频明确标为第 2 次。

首轮 154 景和补录 11 景均已完成，独立发布入口如下。当前包已生成，脚本拒绝覆盖已有包；仅重新校验可使用 `--validate-only`。

```bash
python3 scripts/extensions/replicator/package_reviewable_physical_demos.py --validate-only
python3 scripts/extensions/replicator/package_reviewable_physical_demos.py
```

已生成 `exports/replicator_154_physical_video_bundle_20261028/` 及同名 ZIP：主 `videos/` 154 个（143 首轮 + 11 新录），`first_pass_videos/` 保留 11 个原遮挡视频，共 165 个 MP4；154 份原始结果与 154 份所选结果分目录保存。`demo_selection.json` 逐景给出首轮和所选视频/结果的 SHA、实际状态与尝试编号；README 和网页并列首轮成绩与演示集成绩。原 manifest 作为原字节溯源保存，不将其旧视频路径改指新内容。

视频 ZIP 大小 **168,040,081 字节（168.04 MB）**，SHA256 为 `c0745ac0c48c6d0c1832766798f3760ddb70c6230cf5ef85e500a0b8156db51c`。解压后打开 `index.html` 即可浏览。首轮成绩与所选 154 个演示的成绩本次均为 **70 成功、84 失败**；11 个补录为 **7 成功、4 失败**，逐景结果也与原首轮一致，选择规则仍为固定使用全部补录。所有 165 个视频完整解码通过，ZIP CRC 与全部成员 SHA 验证通过。视频包保存在本机，不随 Git 代码上传。

`physical_camera_retake_plan_20261028.json` 绑定本机已有首轮物理结果与视频 SHA，**新克隆不能凭规划输入包直接使用该补录计划**。新首次执行只使用上方 `--camera-overrides`，避免旧默认相机再次遮挡，不传 `--camera-retake-plan`。

### 初始化停滞与唯一物理 episode

本批次的 `l_island_banana_bowl__human_default_mpfb` 在首次 `env.step` 前初始化停滞，旧结果仍为 `status=initializing`、`executed=false`；对应进程已停止。它没有开始物理 episode，不计作抓放失败。恢复只针对这次尚未开始的 episode，在新 Kit 进程中显式使用 8 个 PhysX 线程；已进入执行的场景不适用此例外。

原 `executions/<scene_id>/result.json` 保持原字节，后续结果写入 `executions/<scene_id>/initialization_retry_01/result.json`，首轮正式视频仍为 `videos/<scene_id>.mp4`。清单和实际结果的 `initialization_history` 绑定旧初始化记录及 SHA，视频包额外收录这些历史文件。**首轮 154 个场景各自一个物理 episode、额外 11 个相机补录 episode，以及未进入执行的初始化启动分别统计**；历史初始化文件不能替代实际执行结果或视频，也不会将初始化停滞改写成成功。

## 历史重试的使用边界

`trajectory_retry_v1.json` 至 `trajectory_retry_v10.json` 保留每轮失败分析和单次优化方案。重试入口会校验前轮 `progress.json`、结果、配置、授权及修正证据 SHA；它们是绑定历史输入的记录，**不是从零复现入口**。没有恢复对应历史证据时，不应直接运行文档中的历史 `--resume` 命令，也不能关闭哈希检查来绕过缺失输入。

本次完整规划的来源为 `reports/replicator_trajectory_retry10_20261028/`。独立审计确认两景本轮各规划一次、152 景历史成功的 1064 个文件保持、133 条人工标注保持。相关审计的路径和 SHA 已写入完整索引；报告本体随单独证据包恢复，不因 Git 中存在这份说明就表示报告或视频已上传。
