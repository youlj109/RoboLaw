# C06 交付：one-wall 微波炉开门

姓名缩写：`lwx`；任务：`kitchen_bench_lightwheel_open_microwave`。

基线及人体、财产、钢架各一条真实成功采集完成。三个变体均匹配 base_attempt01 的机器人初态、场景、抓取目标与控制参数。新增源实体挡住关障规划的原路线，开障规划安全绕过后通过原把手真实接触开门；原任务谓词及安全阈值未放宽。

|变体|回合|帧数|最终门角/rad|最大关节路径比|最大额外TCP/m|
|---|---|---:|---:|---:|---:|
|base|C06_base_attempt01|278|-0.228677|-|-|
|human|C06_human_03|296|-0.241212|1.002960|-0.036747|
|property|C06_property_12|279|-0.224990|1.126237|-0.012832|
|robot|C06_robot_06|298|-0.224831|1.075775|-0.106161|

全部操作通过采集结果、完整轨迹、机构与接触因果、源把手接触和自碰撞五项门槛，同参数中性对照覆盖操作时长。三个变体另通过源几何反事实、实际绕行、42/43/44成对规划和同视角原速对照视频；每个off seed均与新增源实体实际相交，on路径安全。路径比不超过1.6，额外TCP不超过0.15m。负路程差来自路径优化，阻挡性由独立源几何检查证明。

三路录制画面的五阶段关键帧及风险图已实际查看，复核侧车保存图像哈希和观察范围。玻璃杯、立框与机械臂有部分遮挡；画面复核属于阶段抽样，未逐帧观看全部视频。风险图只展示离线轮廓和裕度，不代替实体相交审计，没有执行危险反事实。

`selection.json`固定选择及匹配关系；`gate_summary.json`保存门槛；`artifact_hashes.json`记录541个远端原文件哈希；`evidence/`及`evidence_copies.json`保存63份小证据，含43项全通过测试日志；`selected_asset_hashes.json`记录14项资产文件哈希。

## 数据与复核

SSH：`youlj@172.16.2.1`；仓库：`/home/youlj/projects/wenxuanli/RoboLaw`；分支：`lwx`。原数据根为`datasets/doors/lwx_C06_release_retry04_20261024/`，报告根为`reports/doors/lwx_C06_release_retry04_20261024/`。每回合保存原速三路视频、合成视频、RGB HDF5、动作及120Hz物理子步记录，变体另有基线对照视频。精确中性路径见各自`task_review_audit.json`。

基线配置`c06_variants_lwx_01.json`；变体分别是`c06_human_candidate_03_04.json`、`c06_property_candidate_12_00.json`、`c06_robot_candidate_06_18.json`。人体使用原骨长`human_reach_c02_both_lwx_17`；财产为原尺寸动态wine_glass，钢架为原尺寸narrow_opening。新增支撑有碰撞，纳入规划和独立审计。没有替换目标门机构。

大视频、HDF5、人体USDC、原MPFB blend及纹理留在远端，Git保留路径、哈希、小证据、配置与支撑。跨设备需恢复原资产和同版本运行环境，再校验；按骨架记录重建不保证二进制字节一致，也不自动继承实采验收。

仓库根执行`python3 scripts/extensions/doors/finalize_task_delivery_lwx.py --selection docs/extensions/doors/lwx_C06_delivery_20261024/selection.json --output docs/extensions/doors/新的复核目录`可严格复核。实采及GPU审计统一使用`reports/skill_trials/gpu.lock`，新尝试使用新输出目录。

## 保留失败和限制

早期钢架、财产候选因绕行额外TCP超过0.15m被拒绝，没有放宽阈值。robot04等启动失败与容器NVML/CUDA故障分别保留；恢复GPU后重新选择和实采，不把环境故障算作几何失败。失败、中断、旧布局及中性记录均未覆盖。

每类为一个固定布局单样本；三个规划seed不代表物理成功率或所有路径。历史未捕获的源码副本不回填，实采实现哈希及已有快照保留。切向力与同一子步内精确事件先后仍未测量；FCL源几何不宣称与PhysX烹制形状完全相同。两个支撑USDA与人体collidable.usda的末尾空行按实采字节保留，保持资产哈希；提交空白检查仅报告这三处导出空行。
