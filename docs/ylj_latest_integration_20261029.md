# ylj 最新规划／场景与现有风险流水线的本地合并

合并工作区：`/media/user/新加卷/RoboLaw-huirongzeng`，本地分支 `huirongzeng`。

- 当前风险工作区先保存为检查点 `8b25e160ffdfe4500dc302d4616e9985ecd17d62`。
- 合并 ylj `092749effe29ea4033799abb1ac0dff311edc57e`，合并提交 `46e5247683f3dd829a8f3b7b0f57d761aa9f47d7`。
- 保留当前 HS 动态距离、PT v4、RS、gDist、120 Hz 原始证据／30 Hz 离线标签、IR 关闭、已知分项聚合与未知语义。
- 保留已核对的 14 个原生任务资产上限、正常支撑／目标物／夹持关系配置，以及每条只新增一种指定障碍的授权。既有 U 型碗→水槽接触适用性问题没有在本次代码合并中另行修改。
- ylj 的规划／场景代码和已发布的 13 轮方案原样合并，不重算规划，不把上游历史成功标志当成这次风险流水线的新物理成功。

## 新版选择与历史复现

`--ylj-scene` 入口默认选择 `--ylj-source current`。其来源锁为
`configs/risk_integration/ylj_execution_source.json`，绑定本次 ylj 提交、最新发布结果及每轮冻结方案的 SHA。

`current` 根据每景 `current_revision_result` 查找唯一对应的实际采用层，选择其布局、原始／重定时 NPZ、规划结果和放置快照。它不会把全部场景都套用第十三轮的一条轨迹，也不会在文件缺失时回退到 2026-10-28 原索引。

当前 154 景的采用层为：首轮 70、第一轮重试 36、第二轮 19、第三轮 23、第四轮 1、第五轮 2、第七轮 1、第十一轮 1、第十三轮 1。

源加载复用 ylj 的 `load_retry_inputs`，保留候选规划、重定时、原始轨迹、版本化布局、源几何及 SHA 校验。每景的 close／release 覆盖参数同样沿用 ylj。新源入口默认动作预算 6000、episode 200 秒；显式指定不足的预算时，在启动仿真前拒绝，不能截断后宣称完整回放。

旧数据保持自身冻结规则与来源。显式复现旧原索引用 `--ylj-source legacy`；它保留原 3000 动作、120 秒默认预算。

## 使用示例

进入上面的工作区，在原来的 Arena 环境中运行：

```bash
export ROBOLAW_WORKSPACE=/media/user/新加卷/RoboLaw-huirongzeng
export ARENA_CONTAINER=arena-kitchen-collect

# 只核对输入，不启动 SimulationApp、物理采集或风险 GPU 查询
bash scripts/arena_python.sh scripts/collect_ylj_risk_task.py \
  --task-contract configs/risk_task_suite/replicator_u_shape_bowl_sink.json \
  --scene u_shape_bowl_sink__human_default_mpfb \
  --ylj-source current --admit-added-obstacle --dry-run \
  --asset-cache /path/to/cache --output /path/to/new_capture

# 恢复匹配 SHA 的输入后，再按需要启动新采集；本次合并没有执行此命令
bash scripts/arena_python.sh scripts/risk_capture_workflow.py capture \
  --task-contract configs/risk_task_suite/replicator_u_shape_bowl_sink.json \
  --ylj-scene u_shape_bowl_sink__human_default_mpfb \
  --ylj-source current --ylj-admit-obstacle \
  --asset-cache /path/to/cache --output /path/to/new_run

# 原始证据落盘完成后，单独进行离线评估／渲染／视频
bash scripts/arena_python.sh scripts/risk_capture_workflow.py evaluate --output /path/to/new_run
bash scripts/arena_python.sh scripts/risk_capture_workflow.py render --output /path/to/new_run
bash scripts/arena_python.sh scripts/risk_capture_workflow.py video --output /path/to/new_run
```

## 外部输入可用性

当前 Git 已提供新版代码、冻结方案、版本化配置和结果引用，但未包含全部实际 NPZ／规划结果／场景快照。
在本次 5090 工作区读取到的 154 景中，70 景直接输入文件存在且 SHA 匹配，84 景缺少所选新版输入，0 景已存在的直接输入 SHA 冲突。
完整缺失路径记录在本次合并报告目录的 `current_source_preflight.json`，应从新版轨迹持有者恢复相同字节，不能用旧规划输入包、旧视频或上游成功标签代替。
直接文件检查不代表全部来源递归校验和物理复现已通过；后续仍执行源加载器的完整门槛。

本次没有重新采集、重算风险、渲染视频或推送 GitHub。现有 154 条数据及 14 个网页 ZIP 保持原版本。
