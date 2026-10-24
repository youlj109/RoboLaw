# 同一物理步的抓持证据补充计算

## 问题与修复

原生任务离线回放会在每个物理步的 `advance()` 中更新抓持证据；到 30 Hz 标签时刻，`features()` 又用该步的网格距离补充判断夹爪是否已分离。两次调用原来共享已被第一次调用修改的跟踪器内部状态。

已复现的错误顺序：

1. 上一步具有双侧手指接触，记录抓持锚点。
2. 本步实测夹爪张开，第一次调用判定 `held=null`、`opening_detected=true`，清除锚点。
3. 同一步的第二次调用看不到已清除的上一锚点，若仍有残留双侧接触，就错误地再次输出 `held=true`。

这会影响搬运物 HS 停止模型选择、PT 搬运距离适用性及掉落特征。不是距离算法或判级阈值的问题。

现在每个 payload 分别保存本物理步开始前的跟踪器状态。同一步的补充计算始终从该状态重算；补充后的状态供下一物理步使用。重复读取同一时刻不会再次推进跟踪器，多个 payload 不共享锚点。标签输出增加：

```text
grasp.sampling_semantics = same_source_step_refined_from_pre_step_tracker_state
```

保留 `arena-grasp-evidence-2` 原有接触、张爪、位姿漂移、分离距离和短暂接触丢失阈值。本次仅修正原生任务调用时序，未改通用参考跟踪器、HS/PT/RS 数值规则、采集频率和在线采集内容。

## 验证与边界

`scripts/check_task_grasp_evidence.py` 提供 10 个状态序列检查：重复接触确认、残留接触下的张爪、重复标签、接触年龄、网格分离向下一步的传播、缺接触覆盖、阶段补充、两个独立 payload、接触丢失时限、初始及终止时刻。修复前的重复调用错误已单独复现。已有 29 个 RS 证据回归案例也通过。

```bash
python scripts/check_task_grasp_evidence.py --output /absolute/new-check.json
```

对已有真实记录完成了明确的候选实现比较：

| 记录 | 原始物理步 | 标签时刻 | 结果 |
| --- | ---: | ---: | --- |
| L 型厨房人体芥末瓶完整记录 | 3168 | 793 | 冻结旧实现逐时刻复现原抓持证据；候选 HS/PT/RS 总等级无变化 |
| Farmhouse2 双香蕉短流程 | 12 | 4 | 两个 payload 独立复现；候选 HS/PT/RS 总等级无变化 |

该人体记录的最低人机距离约 0.981 m，不能用来证明近人危险状态已验收；双香蕉记录是保持动作短流程，不是两个物体均完成搬运。张爪残留接触漏洞由针对性输入序列验证；不能声称这两条历史记录本身出现了该漏洞。

`scripts/compare_task_grasp_replay.py` 校验原始摘要、冻结代码归档、共享规则和特征依赖版本、CPU 网格摘要及标签对齐；先用冻结旧实现复现保存的抓持结果，再执行候选特征和冻结规则。它复用每条记录自身的旧 gDist 几何结果，不重新模拟、不查询 GPU、不改写历史结果，只生成独立比较报告。

```bash
python scripts/compare_task_grasp_replay.py \
  --episode /absolute/raw_episode \
  --risk /absolute/saved_offline_result \
  --candidate /absolute/candidate_task_features.py \
  --output /absolute/new-comparison-directory
```

这是显式候选特征迁移诊断，不是新后端采集结果，也不是替代历史标签的数据集。历史数据的正式重放继续遵守其冻结源码及后端身份。

## 源码身份

- 修复前：`181bf8f8f33330eb91451162e39bdd63d94aadf7d573fc0afed03f89b6696eed`。
- 修复后：`2814a7ea76a1adc605bc2c7eb6971e9d97896c79ec9e3b2c2fa217f4f0e8854f`。

新采集会冻结新实现；IR 保持禁用，未知不填零。本次没有新增仿真采集，也没有启动每组 200 条或其他生产批次。
