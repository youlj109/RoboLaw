# C04 交付：大斯堪的纳维亚 G 型厨房冰箱开门

姓名缩写：`lwx`；任务：`kitchen_bench_lightwheel_open_fridge_g_shaped_large_scandinavian`。

本任务完成基线及 human/property/robot 各一条真实成功采集。三种障碍均实体阻挡原始接近路线；加入障碍后保持相同机器人基座、源任务、抓取目标和控制参数，小幅绕行后真实接触把手开门。原门机构、成功谓词及安全阈值未放宽。

| 变体 | 真实回合 | 帧数 | 最终门角/rad | 三种子最大关节路径比 | 三种子最大额外 TCP 路程 |
|---|---|---:|---:|---:|---:|
| 基线 | C04_base_01 | 504 | 0.640269 | — | — |
| 人体 | C04_human_01 | 504 | 0.640317 | 1.00280 | -0.062825 m |
| 财产 | C04_property_01 | 509 | 0.640353 | 1.03734 | 0.012331 m |
| 钢架 | C04_robot_02 | 506 | 0.633773 | 1.02952 | 0.093040 m |

基线及三个操作回合均通过五项独立门槛：采集完成、完整轨迹验证、机构/接触因果与匹配中性对照、源把手几何接触、自碰撞。三个障碍回合另通过原轨迹源几何反事实、实际绕行审计、固定种子 42/43/44 成对规划、同视角原速对照视频及人工抽样三路关键帧复核。每个种子的 off 路径都与障碍源实体实际相交，on 路径安全；关节路径比均不超过 1.6，额外 TCP 路程不超过 0.15 m。负路程差表示该次优化路径更短，并不否定局部绕行。

`gate_summary.json` 给出逐项结果，`selection.json` 固定本次选用回合与匹配关系，`artifact_hashes.json` 校验远端原始数据及对照共 485 个文件。`evidence/` 保存审计 JSON、逐回合元数据、三路关键帧图、风险对照图、成对规划报告及 37 项全通过的测试日志；`evidence_copies.json` 逐个校验这些副本。风险图左栏仅为原录制图上的离线几何标注，未执行危险反事实。

## 数据与视频

设备 SSH：`youlj@172.16.2.1`。仓库：`/home/youlj/projects/wenxuanli/RoboLaw`，分支 `lwx`。原始数据根：`datasets/doors/lwx_C04_C08_20261024/`。

上表每个回合目录含 `trajectory.json`、`trajectory.h5`、`physics.json`、`images.h5` 及原速 `external_camera.mp4`、`external_camera_2.mp4`、`wrist_camera.mp4`，另含 `all_cameras.mp4`。三个障碍回合另含 `baseline_comparison_external_camera_2_03.mp4`。视频和 HDF5 保留在远端；Git 提交包含可审查报告、配置、必要支持资产和原始文件哈希。

中性对照分别由每个回合的 `task_review_audit.json` 中 `matching_neutral.path` 精确指定，同参数且覆盖操作时长；C04 人体采用成功的第二次中性对照。不得将旧的 native exit 139 中性失败算作通过，其原始目录和失败记录继续保留。

## 配置、资产与复现边界

基线配置 `configs/extensions/doors/c04_variants_lwx_01.json`；本次人体、财产、钢架分别使用 `c04_human_candidate_01_14.json`、`c04_property_candidate_01_01.json`、`c04_robot_candidate_02_04.json`。人体使用 `assets/extensions/doors/human_reach_elbow_up/collidable.usda`；财产、钢架保留原库资产尺度，支撑柱分别为对应配置声明的 `c04_property_candidate_01_01_property_support.usda` 和 `c04_robot_candidate_02_04_robot_support.usda`。

`selected_asset_hashes.json` 记录本次实际资产哈希。人体生成 USDC 和纹理不在 Git 中；远端文件完整保留。新设备需原 MPFB `posed.blend`、原纹理、Blender 4.0.2 和 Arena 运行环境，再运行已有 `build_reposed_human.sh`。`artifact_manifest_lwx.json` 已披露重建 USDC 与更早历史导出的字节差异，姿态、包装和纹理匹配；本交付验证针对实际声明文件，不承诺任意重新导出得到相同字节或自动继承验收。

可在仓库根用 `python3 scripts/extensions/doors/finalize_task_delivery_lwx.py --selection docs/extensions/doors/lwx_C04_delivery_20261024/selection.json --output docs/extensions/doors/新的复核目录` 重新验证同一批数据。新采集请用 `run_fixed_sample_lwx.py`，选用上述匹配基线及配置，并指定全新输出目录；不得覆写本次原始记录。GPU 仿真、规划和 GPU 独立审计继续共用 `reports/skill_trials/gpu.lock`。

本交付为一个固定布局的每变体单条成功数据；三个规划种子不是三条额外物理成功，也不是成功率或所有路径的证明。人工视觉检查为实际三路录制的阶段抽样，不声称逐帧看完全部视频。未测切向力及同一物理子步内部的精确先后顺序；FCL 原始几何审计不等同于 PhysX 烹制几何的完全等价性。这些限制不改写已有通过项，也不把其他七个任务的部分进展算成全部完成。

生成的两个 USDA 支撑文件保留采集时的导出字节（含文件末尾空行），以保持资产哈希；除这两条原始导出空行提示外，提交空白检查通过。
