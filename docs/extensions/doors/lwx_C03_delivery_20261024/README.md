# C03 交付：大斯堪的纳维亚 G 型厨房冷冻柜开门

学生 `lwx`；任务 `kitchen_bench_lightwheel_open_freezer_g_shaped_large_scandinavian`。

human、property、robot 各一条真实成功回合及匹配基线已完成独立验收。障碍实体挡住原接近路径，变体保持其匹配基线的机器人基座、抓取目标及控制参数，小幅绕行后真实接触开门。未放宽原成功谓词或安全阈值。

|变体|回合|帧数|最终门角 rad|最大关节路径比|最大额外 TCP m|
|---|---|---:|---:|---:|---:|
|base|C03_base_05|427|0.642955|—|—|
|human|C03_human_05|432|0.642396|1.035453|-0.070247|
|property|C03_property_05|434|0.661296|1.043457|-0.116538|
|robot|C03_robot_02|436|0.639528|1.040093|-0.105181|

人体和钢架匹配 `C03_base_05`，财产匹配 `C03_base_07`；不得互换基线。每个回合的同参数等时长中性对照由 `task_review_audit.json` 的 `matching_neutral.path` 精确指定。

所有选用操作回合均通过采集、完整轨迹、机构与接触因果、中性对照、源把手几何接触、自碰撞审计。三种障碍另通过源几何反事实、实际绕行、42/43/44 三种子成对规划、同视角原速比较视频与实际三路关键帧及风险图人工抽样复核。每种子的 off 路径实际相交，on 路径安全；关节路径比不超过 1.6，额外 TCP 不超过 0.15m。负路程差表示优化路径更短，不否定局部绕行。

`selection.json` 固定选用与配对，`gate_summary.json` 汇总门槛；`artifact_hashes.json` 记录 556 个远端原始文件，`evidence_copies.json` 校验 74 个小证据副本，`selected_asset_hashes.json` 记录 17 个资产文件。`evidence/` 含原审计、元数据、成对规划、三路关键帧、风险图及 37 项全通过测试日志。

原数据和原速视频保留在 `youlj@172.16.2.1:/home/youlj/projects/wenxuanli/RoboLaw/datasets/doors/lwx_C02_C03_20261023/`；每回合含 trajectory/physics JSON、HDF5、external_camera.mp4、external_camera_2.mp4、wrist_camera.mp4、all_cameras.mp4。障碍回合另有 baseline_comparison_external_camera_2_03.mp4。Git 提交包含报告、配置、支撑资产及哈希。

人体和钢架采用 `configs/extensions/doors/c03_variants_lwx_04.json`；财产采用 `c03_property_candidate_05_12.json`。保留原库尺度和完整厨房/门扫掠。人体生成 USDC 和纹理不纳入 Git，远端完整保留；同源 Blender 重建存在已披露的历史导出字节差异，须按实际资产哈希复核，不承诺重建后字节相同或自动继承验收。

可在仓库根运行 `python3 scripts/extensions/doors/finalize_task_delivery_lwx.py --selection docs/extensions/doors/lwx_C03_delivery_20261024/selection.json --output docs/extensions/doors/新的复核目录` 复核同批数据；所有新采集必须用新目录，GPU 作业共用 gpu.lock。

保留的失败与限制：
- Older property candidates with zero loaded normal contact are rejected and retained.
- C03 property candidate04 and property05 candidates0/2 exceeded the 0.15m detour limit and were excluded.
- Property05 uses baseline07; human05 and robot02 use baseline05, each with its own matching neutral.
- Historical episodes without immutable runtime source snapshots retain their original hashes; missing source snapshots are not backfilled.
- datasets/doors/lwx_C02_C03_20261023/C03_base_05: historical implementation hashes retained; immutable runtime source copies were not captured.
- datasets/doors/lwx_C02_C03_20261023/C03_robot_02: historical implementation hashes retained; immutable runtime source copies were not captured.
- One original fixed layout; planning seeds are not physical success rates or proof about all paths.
- Tangential forces and ordering inside one physics substep remain unmeasured.
- FCL source geometry checks do not assert equivalence to PhysX cooked shapes.

上述限制不算通过项。原 property 零载荷和超出 0.15m 绕行候选均保留为失败；风险图为离线标注，未执行危险反事实。人工视觉复核是阶段抽样，未声称逐帧观看。部分历史回合无不可变源码副本，保留原运行哈希而不回填。生成支撑 USDA 原始字节保留，包括原末尾空行。
