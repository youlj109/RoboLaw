# lwx C02 交付

任务：`kitchen_bench_lightwheel_open_cabinet_u_shaped_with_island_farmhouse2`。本任务严格 finalizer29 与证据打包通过，原文件819个hash，83份小证据副本，14个资产hash；本组76项单元测试通过。

|变体|实采|匹配基线|帧数|最大额外TCP|最大关节路径比|
|---|---|---|---:|---:|---:|
|base|`datasets/doors/lwx_C02_shortcut_settle_retry27_20261024/C02_base_01`|`—`|311|—|—|
|human|`datasets/doors/lwx_C02_release_retry06_20261024/C02_human_08`|`datasets/doors/lwx_C02_release_retry06_20261024/C02_base_attempt01`|302|0.008871 m|1.018009|
|property|`datasets/doors/lwx_C02_shortcut_settle_retry27_20261024/C02_property_27`|`datasets/doors/lwx_C02_shortcut_settle_retry27_20261024/C02_base_01`|312|0.120186 m|1.209804|
|robot|`datasets/doors/lwx_C02_planning_padding_retry18_20261024/C02_robot_26`|`datasets/doors/lwx_C02_planning_padding_retry18_20261024/C02_base_01`|301|0.120497 m|1.047744|

三种变体均通过：原任务真实接触开门；匹配中性和未操作机构稳定；120 Hz 实际外部/自碰撞审计；实际源碰撞几何反事实；42/43/44固定目标成对规划；每个关障 seed 实际相交、有障路径安全；关节路径比≤1.6、额外TCP≤0.15 m；原速同视角对照视频；实际raw与risk图查看。危险反事实未物理执行。

property27 的原目标pregrasp实测误差为0.135 mm /0.000454 rad，小于原4 mm/.03 rad。与新base27同时增加15步原计划终点保持，未换目标或添加绕行路点。human08与robot26分别使用表中匹配基线，参数一致性逐对验证，不声称三个变体控制参数完全相同。

背景配置继承预声明的 `c02_mechanisms_lwx_01.usda` 驱动/阻尼适配，目标柜门及原生谓词未改。旧human06背景不匹配、旧绕行超限、property26终点超限均保留。打包28同名基线冲突的部分文件和错误保留在 `lwx_C02_packaging_attempt28_20261024`，本次为修复后的29。

`selection.json` 给出精确选择；`gate_summary.json` 列出门槛与局限；`artifact_hashes.json` 索引原始文件和视频；`evidence_copies.json` 索引冻结小证据；`evidence_episode_paths.json` 映射不同源目录到不同目的目录，避免同名基线合并。大原始数据与视频保留于远端 `datasets/doors/`。

本组固定布局单样本与三个规划seed不外推物理成功率；FCL原几何不等同于PhysX cooked shape；实际视觉是阶段关键帧抽查，不能代表逐帧审阅。切向力、子步内严格先后及历史源码快照局限见gate_summary。
