# C05 交付：U型带中岛 Farmhouse2 冰箱开门

姓名缩写：`lwx`；任务：`kitchen_bench_lightwheel_open_fridge_u_shaped_with_island_farmhouse2`。

基线和人体、财产、钢架各一条实际成功采集已完成。三个变体均与 base03 保持相同场景、机器人初态、抓取目标和控制参数；源实体挡住原接近路线，障碍感知规划后小幅绕行并通过真实把手接触打开原门。原门机构、原生成功谓词及验收阈值未放宽。

| 变体 | 真实回合 | 帧数 | 最终门角/rad | 三种子最大关节路径比 | 三种子最大额外 TCP 路程 |
|---|---|---:|---:|---:|---:|
| 基线 | C05_base_03 | 434 | 0.327152 | — | — |
| 人体 | C05_human_06 | 355 | 0.326047 | 1.00302 | -0.102350 m |
| 财产 | C05_property_06 | 403 | 0.325419 | 1.04563 | -0.100498 m |
| 钢架 | C05_robot_05 | 353 | 0.326908 | 1.00606 | -0.135758 m |

全部操作通过采集、完整轨迹、机构与接触因果、源把手接触和自碰撞五项门槛；各自同参数中性对照覆盖操作时长。三个变体另通过源几何反事实、实际绕行、42/43/44 成对规划、同视角原速对照视频及实际三路阶段关键帧和风险图复核。每个 off seed 都与新增源实体实际相交，on 路线安全；关节路径比不超过 1.6、额外 TCP 不超过 0.15 m。负路程差来自路径优化，不代替实体阻挡检查。

`selection.json` 固定选择和匹配关系；`gate_summary.json` 列出验收结果；`artifact_hashes.json` 保存远端原数据、中性对照及视频共 533 个文件的哈希；`evidence/` 和 `evidence_copies.json` 保存并校验 63 个小证据副本，含 39 项全通过测试日志。`selected_asset_hashes.json` 记录 13 个实际资产文件的哈希。风险图只是在录制 RGB 上离线标注几何，不执行危险反事实。

## 数据和复核

SSH：`youlj@172.16.2.1`；仓库：`/home/youlj/projects/wenxuanli/RoboLaw`；分支：`lwx`。原始数据根为 `datasets/doors/lwx_C04_C08_20261024/`。每个回合含原速 `external_camera.mp4`、`external_camera_2.mp4`、`wrist_camera.mp4`、`all_cameras.mp4`、轨迹、物理子步及 RGB HDF5；变体另含 `baseline_comparison_external_camera_2_03.mp4`。大文件留在远端，Git 包含报告、关键帧、配置、必要支撑和原文件校验值。精确中性路径见各回合 `task_review_audit.json` 的 `matching_neutral.path`。

本次基线配置为 `c05_variants_lwx_01.json`；人体、财产、钢架分别为 `c05_human_candidate_06_05.json`、`c05_property_candidate_06_00.json`、`c05_robot_candidate_05_12.json`。财产使用原尺寸 tablet_stand，钢架使用原尺寸 projecting_crossbar，支撑均有碰撞并进入规划和审计。人体为 `human_reach_low_both_lwx_18` 的原骨长双臂姿态，根部和脚不移动；精确姿态与骨长误差记录见其 `pose.json`。

人体 USDC、原 MPFB `assets/people/mpfb_corridor/posed.blend` 和纹理未打包到 Git，实际远端文件保留并校验。跨设备需原资产、Blender 4.0.2 及相同 Arena 环境，按 `pose.json` 和 `repose_human_both_lwx.py`、`wrap_reposed_human.py` 重建后重新校验；不保证重新导出的二进制字节一致，也不自动继承实采验收。

可在仓库根用 `python3 scripts/extensions/doors/finalize_task_delivery_lwx.py --selection docs/extensions/doors/lwx_C05_delivery_20261024/selection.json --output docs/extensions/doors/新的复核目录` 复核原证据。新采集用 `run_fixed_sample_lwx.py`、本次匹配基线和配置，以及全新输出目录；GPU 仿真、规划和 GPU 审计共用 `reports/skill_trials/gpu.lock`。

## 保留拒绝和限制

旧基线、旧场景候选和所有失败目录均保留，不计入本次交付。人体 external_camera 有近距离身体遮挡和裁切，external_camera_2 与 wrist 能看到接触开门各阶段；视觉复核为实际录制关键帧抽样，未逐帧观看全部视频。此交付仅为固定布局每类单条成功；三个规划 seed 不是额外物理成功率或所有路径的证明。历史回合缺失的源码副本不回填；各自采集时的实现哈希和已有源码快照保留。切向接触力及同一物理子步内的精确先后仍未测量；FCL 源几何不宣称与 PhysX 烹制形状完全等价。
