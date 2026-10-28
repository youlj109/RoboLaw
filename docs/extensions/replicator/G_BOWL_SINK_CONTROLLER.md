# G 型碗放入水槽：原任务适配进度

任务为 `kitchen_bench_replicator_g_shape_bowl_sink`。截至 2026-10-23，`base/attempt_04` 已完成真实抓放，并通过基线独立验收；人体、财产、障碍物三类变体仍在开发，整个原始任务尚未完成交付。下方首轮失败历史完整保留。

## 当前已验证基线：attempt_04

数据在 `datasets/replicator/g_shape_bowl_sink/base/attempt_04/`。包含 589 个完整动作、从 reset 开始的 4713 个连续物理状态和 6 个完整解码视频。原任务首次成功位于物理步 3672；释放、撤离、末态静置全部完成。末态原接触力约 4.90447 N、速度 0.000264961 m/s，原成功谓词独立复核通过。

`reports/replicator_execution_20261023/g_bowl_sink/attempt04_baseline_review_bundle01.json` 汇总 23 项独立检查；根代理另核对 24 个输入/报告/视频哈希，结论记录在 `attempt04_root_baseline_acceptance01.json`。所有 4713 个实测 7+6 关节状态的自碰撞代价均为 0，最大关节浮点越界 3.49512e-8 rad，小于原 1e-5 rad 数值门槛。源布局、固定台架、背景稳定、匹配 45 秒中性观察、原相机和图像/动作/状态对齐均通过。

必要的原碗/水槽底面支撑接触单独记录：1115 个源三角形严格相交样本未删除，由完整原生接触、原实体空间边界和原上向支撑力独立解释。源三角形相交不等同于 PhysX cooked shape 穿透；没有按观测最大值拟合穿透容差，其余碰撞对保留原门槛。下放最小源间隙 3.71099 mm。撤离规划证书绑定释放后实测碗位姿、源几何和原生支撑，不使用规划生成的接触。

已实际查看四视角共 36 个关键帧，可见夹起、转运、释放入槽与夹爪撤离。末张 RGB 为物理步 4704，末态数值为 4712，不将离散关键帧检查描述为全视频连续视觉认证。旧版 generic 审计缺少 self/visual 的结果及 float64/float32 指令比较失败均保留，修订报告另存。

复现已验证的基线入口（使用新的、尚不存在的输出目录）：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/bowl_supported_collect.py --device cpu --physics-threads 2 --torch-threads 2 --seed 42 --output datasets/replicator/g_shape_bowl_sink/base/repro_supported01
```

采集入口的 `physical_trial_passed=true` 仍须配独立验收。新 `bowl_safety_collect.py` 正在接入三类变体和匹配控制配置，尚未验证为可交付采集入口；不得用本基线代替三类成功数据或三 seed 障碍开关对照。

## 2026-10-23 实测更新

新 canonical 的 `reports/replicator_execution_20261023/g_bowl_sink/neutral_01/` 已完成 45 秒、5401 个物理状态的静置观察，15 项检查全部通过，六个视频完整解码。已实际查看 overview、原 external_camera_2、原 wrist_camera 的 0/337/674 帧；外部相机清楚显示台面的红碗与原水槽，腕相机中碗部分在视野外。采样画面复查见该目录 `root_visual01/review.json`，不把少量画面检查当作全视频连续视觉认证。

`reports/replicator_preparation/g_bowl_collector_paths_gpu01/` 完成原 cuRobo 接近、抬升和下放候选诊断。接近路径原 joint/world/self 全通过；抬升与下放仅针对指定碗/源台面、碗/原 BasinBottom 接触对作源几何检查，其余原球查询、自碰撞和完整 FCL 保留。下放原附着物球在末两点与 BasinBottom 重叠，真实源碗与底面仍约 3 mm；该原球失败完整保留。未绑定实际连续双指受力和实测附件的候选 `valid=false`，不能据此执行。

真实 `datasets/replicator/g_shape_bowl_sink/base/attempt_01/` 已执行预抓取、接近和闭爪，保存 2457 个连续物理状态、307 个动作/图像时刻及六个完整视频。两指均有实际受力，但闭爪末左指接触点比原碗沿区域下界低约 2.144 mm，原双侧区域检查拒绝；未执行抬升、转运或释放，原任务未成功。闭爪期间 TCP 向 +X 移动约 4.773 mm，碗向 −X 移动约 1.523 mm；不能把源几何假设的固定物体闭爪成功当作真实抓取。现场全物理 FCL、背景稳定性、固定台架和 2457/2457 实际 q 自碰撞检查通过，完整失败原始证据保留。后续从实际姿态诊断抓点，不扩大原接触区域。

## 已查明并处理的问题

原初态的机械臂与上柜相交。按预声明候选顺序，将机器人 X 从 2.2041800022 m 改为 2.1541800022 m；碗初始高度增加 0.1 mm，解决源几何距离略小于严格 2 mm 的浮点初态问题。原机器人朝向、0.8 m 台架、碗和厨房资产、On/NextTo 关系及原安全距离全部保留。

当前配置：`configs/extensions/replicator/g_shape_bowl_sink_source_layout.json`，SHA256 `052d3c155ebf3ccb20b9285812fdd415a0a169c09626ee993ce75237435cdaa5`。旧配置原件、候选及推广依据保存在 `reports/replicator_preparation/g_tasks_layout_promotion01/`。新配置必须重新运行中性观察；旧配置的报告不能继承。

碗直径约 161 mm，超过原夹爪的横向开口。控制候选由源三角形求得负 X 侧上沿约 4.166 mm 的壁厚，采用一指进入碗内、另一指位于外侧的碗沿夹持。完整闭合扫描保留 151 个角度，首次双指源表面相交在 0.75 rad；源三角形裁剪复核，两指均只接触指定碗沿区域。这里的相交是离线几何证据，不是已建立物理接触或夹持力。

将碗放在原水槽底面中心会碰到 `Sink/Drain_0`。中心失败完整保留，没有删除排水口碰撞。随后在原 `Sink/BasinBottom` 允许区域内预声明 117 个落点，按距中心的平方距离、X、Y 排序；检查前 66 个后，第 65 号（从零编号）首次通过：中心偏移 **X +0.05 m、Y −0.10 m**。原任务目标仍是该水槽，源碗完整投影位于底面内，释放候选间隙仍为 3 mm。

## 首轮历史证据范围

| 检查 | 结果 | 证据 |
|---|---|---|
| 原任务关系与完整初始源几何 | 通过 | `g_bowl_sink_initial_search_cpu01/`、`g_bowl_sink_candidate_source_geometry_cpu01.json` |
| 预抓取/抓取 IK 与接近路径 | CPU 通过 | `g_bowl_sink_controller_cpu02.json` |
| 151 个闭合角度 | 源 FCL、原保守夹爪球通过 | `g_bowl_sink_grasp_cpu01/` |
| 首次双指源表面相交 | 两指仅在碗沿指定区域，非物理成功 | 同上 |
| 假设夹持后的 15 cm 抬升 | 全路径源 FCL 通过；原携物球仍触发必要的碗/原台面接触对 | 同上 |
| 水槽中心下放 | **失败**：碰到排水口 | `g_bowl_sink_transfer_cpu01/` |
| 新落点的 64 个下放样本 | 全 FCL 通过；旧 CPU 球检查屏蔽了底面，不能据此声称全世界原球通过。新独立 GPU 检查保留末两点的携物球/底面拒绝 | `g_bowl_sink_placement_cpu01/`、`g_bowl_collector_paths_gpu01/` |
| 控制目标绑定与边界测试 | 3 项通过 | `g_bowl_profile_tests01.log` |
| cuRobo CUDA 自碰撞与完整自由转运 | 接近候选及首轮已执行段通过；自由转运尚未执行 | 不能由 CPU FK/碰撞检查代替 |
| 新初态中性观察与 RGB | 45 秒、15 项通过，六视频完整解码 | `reports/replicator_execution_20261023/g_bowl_sink/neutral_01/` |
| 真实抓放与原成功谓词 | 首轮闭爪区域检查失败，未抬升或成功 | `datasets/replicator/g_shape_bowl_sink/base/attempt_01/` |

上表证据均位于 `reports/replicator_preparation/`，为首轮候选及实采历史。首轮失败诊断和后续明确修正见 [attempt_01 诊断](G_BOWL_SINK_ATTEMPT01.md)。当前 `configs/extensions/replicator/g_shape_bowl_sink_controller.json` 已更新为 schema v2：在每次当前碗姿态重算的源抓点上固定 world-X +25°、Z +5.039543829 mm，原接触区不变；这不是上游 YAML 的数值要求，而是本地预声明抓沿策略。287 个 GPU 诊断状态的原 joint/self/world 全通过、源 FCL 全通过，仍待新版真实采集。`bowl_controller_profile.py` 绑定初态、源资产、原抓点源码及准备证据哈希；不从历史录制读取运行时目标，不通过改物体位姿实现成功。

## CPU 复现入口

在本人的正确容器挂载中执行；输出目录必须不存在。以下前三条复现旧版源几何候选历史，不能替代新版 profile 的 CPU 预检。各命令不启动 Kit/CUDA，不能产生真实轨迹或视频。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/audit_bowl_controller_cpu.py --layout configs/extensions/replicator/g_shape_bowl_sink_source_layout.json --output reports/replicator_preparation/repro_bowl_controller.json
bash scripts/arena_python.sh scripts/extensions/replicator/audit_bowl_grasp_cpu.py --layout configs/extensions/replicator/g_shape_bowl_sink_source_layout.json --output reports/replicator_preparation/repro_bowl_grasp
bash scripts/arena_python.sh scripts/extensions/replicator/search_bowl_sink_placement_cpu.py --layout configs/extensions/replicator/g_shape_bowl_sink_source_layout.json --grasp-report reports/replicator_preparation/repro_bowl_grasp/diagnostic.json --output reports/replicator_preparation/repro_bowl_placement
bash scripts/arena_python.sh -m pytest -q tests/extensions/replicator/test_bowl_source_profile.py
bash scripts/arena_python.sh scripts/extensions/replicator/bowl_collect.py --output reports/replicator_preparation/repro_bowl_corrected_preflight --device cpu --physics-threads 2 --torch-threads 2 --dry-run
```

下一步须验证新初态中性物理、实际 cuRobo 门禁和自由转运，再用原接触传感器完成真实夹持、抬升、释放和水槽支撑。人体、财产、障碍物三种增广只在匹配基线真实成功后设计，沿用同一落点策略并逐项补齐技能验收。
