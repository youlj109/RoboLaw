# ylj 场景/规划与当前风险流水线融合

2026-10-28：场景、物理修复、抓取策略与冻结轨迹以 `ylj@e8c8fed` 为来源；风险资产范围、HS/PT/RS 规则、gDist 后端和在线/离线边界以当前风险分支为来源。合并前风险代码、全部 31 个任务策略及用户未提交的资产排除已先保存为独立 Git 提交，运行目录另有源码备份。

## 范围锁与覆盖

`configs/risk_integration/replicator_scope_lock.json` 冻结用户刚核对的 14 个 Replicator 原生任务的精确 PT 目标及碰撞表面。上限分别为 G 型 11/7/10、L 带中岛 9/5/5、L 型 11/6/9、半岛式 13/11、U 型 10/7/8（同环境按芥末瓶、香蕉、碗顺序；半岛式无香蕉）。其来源 scope/raw/reconstruction SHA 均保存。工作区筛选结果再与范围锁取交集，允许减少，禁止换入未确认资产；被排除表面在 GPU 查询前删除，其距离、力/冲量风险特征和规则均不生成。共享抓持、RS 和完整场景重建证据仍保留。

ylj 的 154 个场景包含新加障碍。导入场景配置不等于授权把它们加入风险计算。当前原生任务范围锁没有人体；选择引入新人体的 ylj 变体会明确拒绝启动，不伪装为无人/L0。财产/机器人障碍可保留在物理场景中，但未在范围锁中的目标不参加环境风险评价，输出明确说明覆盖限制。不得把这个限制解释为新增障碍已安全验收。今后需要扩展覆盖时必须单独明确更新资产范围，不能为了通过预检自动扩充清单。

原有 Lightwheel 17 个任务保持原入口、场景和风险配置。31 个任务的正常支撑、目标容器、夹持目标接触豁免、工作区及用户逐项排除均保留。固定底座的展示省略不扩大为删除 RS 几何。

## 使用统一流水线

先按 ylj 发布说明恢复 [原始输入包](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety/resolve/master/robolaw/replicator_154_planning_inputs_20261028.zip)（SHA256 `32563dc7d829d1131c1541858b11c255318c618fc308ae0b6f21bcca43d631cd`）及外部源资产。下载说明来自 `ylj@770c67b`。保留索引中的相对路径与原始字节。Git 本身不包含实际轨迹 NPZ、规划结果和 source_scene 证据，缺失时严格报错；不生成伪轨迹，不跳过 SHA，不重新规划冒充原版本。

5090 上本次验证的融合工作区是 `/media/user/新加卷/RoboLaw-huirongzeng`。它保留 ylj 的原始障碍物文件；旧运行目录中部分同名文件哈希不同，不能混用。机器人、依赖与官方资产可复用已校验的本机缓存，ylj 直接来源文件需在融合工作区内按原 SHA 恢复。原运行目录的旧资产保留，供历史数据使用。运行前进入融合工作区并指定它在容器内的路径：

```bash
cd /media/user/新加卷/RoboLaw-huirongzeng
export ARENA_CONTAINER=arena-kitchen-collect
export ROBOLAW_WORKSPACE=/media/user/新加卷/RoboLaw-huirongzeng
```

`ROBOLAW_WORKSPACE` 只选择此次进程的代码/资产工作区，路径必须已经挂载到现有容器；不设置时原入口仍使用 `/workspace`。

```bash
scripts/arena_python.sh scripts/collect_ylj_risk_task.py \
  --task-contract configs/risk_task_suite/replicator_l_island_banana_bowl.json \
  --scene l_island_banana_bowl__property_tall_glass \
  --output /path/to/new_capture --asset-cache /path/to/cache --dry-run

scripts/arena_python.sh scripts/risk_capture_workflow.py capture \
  --task-contract configs/risk_task_suite/replicator_l_island_banana_bowl.json \
  --ylj-scene l_island_banana_bowl__property_tall_glass \
  --output /path/to/new_run --asset-cache /path/to/cache
scripts/arena_python.sh scripts/risk_capture_workflow.py evaluate --output /path/to/new_run
scripts/arena_python.sh scripts/risk_capture_workflow.py render --output /path/to/new_run
scripts/arena_python.sh scripts/risk_capture_workflow.py video --output /path/to/new_run
```

`ARENA_CONTAINER` 应设为实际安装的容器名。精确 scene ID 必须显式给出，不替用户从 11 个变体中暗选一个。`--ylj-scene` 不与另一套 policy factory、policy config 或 obstacle binding 混用。

## 执行与证据

复用 ylj 的 `build_environment`，仅将厨房命名空间映射为当前任务的环境 ID，保证原有精确资产路径及排除策略可以绑定；不改变布局、质量、摩擦、夹爪限值或规划样本。保留其背景物理 overlay、无关系重新求解、初始状态校验、30 Hz 原轨迹，以及 5 秒初始等待、2 秒夹紧、2 秒释放、3 秒结束等待。原始任务成功与最终目标放置状态单独记录。

在线只保存 120 Hz 原始物理状态/接触/实际命令、30 Hz 提交动作和观测、CPU 网格、重建场景、规则与控制器输入来源，不启用相机写视频、不建风险 GPU BVH、不做风险测距/特征/等级。原任务已有成功谓词仍属于任务执行证据。保留 ylj 的单次初始化和终态不重置策略；仅该入口显式允许在任务 success 后继续完成源轨迹，其他采集入口仍按原终止行为。

离线沿用 `evaluate_native_task.py` 和当前已调优 gDist 完整网格并行后端，再离线渲染及叠加风险依据。原始结果为 `pending_offline`，评价为 `offline_replay`；IR 关闭、缺测未知、已知分项取最高等级。不会改写历史数据或视频。

ylj 快速轨迹未执行 0.4 rad/s 减速处理的已知问题仍以源版本保留并记录，不暗中重采样；这次融合不调整安全规则去适应更快的指令。物理任务成功、几何完整和风险等级三者分别报告。

## 验证

`python -m unittest discover -s tests -p test_ylj_risk_integration.py` 检查精确资产/表面交集、同数量换入资产拦截、新人体拒绝、任务身份、完整 NPZ 通道与时间轴、ylj 插入等待和 154 景输入预检。真实运行还需要输入包及匹配的外部 USD；CPU 测试通过不能代替全部 154 景端到端验收。

本机已下载并核对输入包全部 794 个来源文件，154 景直接来源校验通过。按“不增加风险计算资产”要求，140 个非人体变体可以进入当前融合采集，14 个新增人体变体仍需独立 HS 范围授权。已对 `l_island_banana_bowl__property_tall_glass` 执行完整原轨迹，得到 2,376 个物理步、594 个动作及 595 个离线风险样本；PT 保留原 5 个资产，原任务成功。在线记录 82.24 秒，离线评价 227.07 秒，其中 gDist 完整网格测距 4.18 秒。未知与审计项仍保留，本条验证不替代其余场景的实际运行验收。
