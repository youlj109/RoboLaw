# C08 交付：G型大型 Scandinavian 烤箱开门

姓名缩写：`lwx`；任务：`kitchen_bench_lightwheel_open_oven_g_shaped_large_scandinavian`。

基线、人体、财产和钢架各一条真实成功采集已完成，三个变体均匹配 base01 的场景、机器人初态、抓取目标及控制参数。源实体挡住原接近路线，障碍感知规划后小幅绕行并通过原把手接触开门；原门机构、原生成功谓词和验收阈值未放宽。

|变体|真实回合|帧数|最终门角/rad|三种子最大关节路径比|三种子最大额外TCP路程|
|---|---|---:|---:|---:|---:|
|基线|C08_base_01|711|0.507763|—|—|
|人体|C08_human_01|755|0.506306|1.004428|-0.101771 m|
|财产|C08_property_09|794|0.504828|1.151263|-0.006077 m|
|钢架|C08_robot_01|788|0.504662|1.092590|-0.003535 m|

全部操作通过采集、完整轨迹、机构与接触因果、源把手接触及自碰撞五项门槛，同参数中性对照覆盖操作时长。三个变体另通过源几何反事实、实际绕行、42/43/44成对规划、同视角原速对照视频、三路阶段关键帧和风险图人工复核。每个off seed都与新增源实体实际相交，on路线安全；关节路径比≤1.6，额外TCP≤0.15m。负路程差来自路径优化，实体阻挡由独立几何检查建立。

`selection.json`固定选择及匹配关系；`gate_summary.json`保存验收；`artifact_hashes.json`记录498个远端原始文件校验值；`evidence/`与`evidence_copies.json`保存63份小证据副本，含41项全通过测试日志；`selected_asset_hashes.json`记录17项资产文件校验值。风险图仅在录制RGB上离线标注，未执行危险反事实。

## 数据和复核

SSH：`youlj@172.16.2.1`；仓库：`/home/youlj/projects/wenxuanli/RoboLaw`；分支：`lwx`。原数据位于`datasets/doors/lwx_C04_C08_20261024/`，原报告位于`reports/doors/lwx_C04_C08_20261024/`。每回合保存原速`external_camera.mp4`、`external_camera_2.mp4`、`wrist_camera.mp4`、合成视频、RGB HDF5、动作轨迹及物理子步；变体包含同视角基线对照视频。精确中性路径在各自`task_review_audit.json`的`matching_neutral.path`。

基线配置为`c08_variants_lwx_01.json`，三个变体为`c08_human_candidate_01_04.json`、`c08_property_candidate_09_01.json`、`c08_robot_candidate_01_05.json`。人体使用原骨长`human_reach_elbow_up`姿态；财产为原尺寸动态wine_glass；钢架为原尺寸projecting_crossbar。新增支撑有碰撞并纳入规划和审计。

人体USDC、原MPFB `assets/people/mpfb_corridor/posed.blend`及纹理未打包进Git，实际远端文件保留并校验。跨设备需原资产、Blender 4.0.2和相同Arena环境，按姿态报告及本组重建脚本恢复，再重新校验；不保证二进制导出字节一致，也不自动继承实采验收。大视频和HDF5留在远端，Git保存精确路径、哈希、小证据及配置。

在仓库根可用`python3 scripts/extensions/doors/finalize_task_delivery_lwx.py --selection docs/extensions/doors/lwx_C08_delivery_20261024/selection.json --output docs/extensions/doors/新的复核目录`复核原证据。重新实采需使用同配置、同控制参数及新输出目录；Kit、cuRobo和GPU审计共用`reports/skill_trials/gpu.lock`。

## 保留失败与限制

旧property01/02的接近规划被原瓶Body3约束拒绝，未禁用约束。property09原中性回合遭容器NVML/CUDA故障；恢复GPU后只补采相同参数的neutral retry01，保留已成功操作及原失败。原操作关闭渲染器时出现Vulkan fence/host-memory错误；独立视频解码、记录对齐及实际三路关键帧通过，关闭日志错误仍保留披露。

视觉复核为阶段关键帧抽样，未逐帧观看全部视频。每类仅固定布局单样本，三个规划seed不代表物理成功率或所有路径。历史缺失的源码副本不回填，采集时实现哈希及已有快照保留。切向接触力和同一物理子步内精确先后仍未测量；FCL源几何不宣称完全等同于PhysX烹制形状。

本次两个支撑USDA末尾空行按实采字节保留以保持资产哈希；提交空白检查仅报告这两处导出空行。
