# RS 原始证据与完整性校验

本次交付以安全评估正确性为目标，不要求每组采集 200 条。以下检查不构成生产批次，不以任务成功代替风险证据完整。

## 修复的证据处理问题

`kitchen_safety/task_features.py` 原先在同一个异常处理块中计算关节裕量和力矩比。位置缺失时，仍有效的力矩也会被丢掉。现在两个通道独立校验、独立提取：

- 关节映射必须唯一且包含所有声明的受控关节。
- 位置、力矩、限位和限矩数组必须与原始关节身份数组形状一致。
- 参与计算的数值必须有限；限位下界必须小于上界，限矩必须大于零。
- 一个通道失败仅使相应特征为 null，并记录原因，不清除另一个通道的有效值。
- 沿用冻结规则的裕量、归一化力矩和等级阈值；没有新增阈值。

原先自碰撞覆盖使用 `all(row.complete for row in results)`，空结果会被视为完整。现在必须与本条冻结 `rs_pair_scope.json` 核对：

1. 每对机器人刚体都必须进入检查集合或具有明确的结构允许关系。
2. 返回的几何对必须恰好覆盖待检查集合；缺对、重复、越界和不完整结果均不能证明“无碰撞”。
3. `intersection` 必须是布尔值；null、数字 0 或 1 不能冒充完整检测。
4. 无正证据且覆盖不完整时，自碰撞标志为 null，不能输出 false。
5. 已确认的未允许相交或原始接触仍保留为 true，即使其他对缺测；等级仍按冻结 `RS-SELF-001` 给 L2，同时报告覆盖不足。

特征 `self_collision_evidence` 增加 expected/observed pair count、missing_pairs 和 errors，便于区分正常负结果与没有检测。它不改变距离算法、查询数量、关节控制或物理时间步。

## 可复现检查

在已安装项目依赖的 Python 环境中运行，输出路径必须不存在：

```bash
python scripts/check_task_rs_evidence.py --output /absolute/new-rs-check.json
```

29 个针对性案例覆盖合法输入、缺测、非有限值、错误数组形状、无效限位/限矩、映射错误、缺对/重复/未知相交、结构允许接触及部分覆盖下的正碰撞。

可以只读现有原始与离线数据进行明确的候选实现比较：

```bash
python scripts/check_task_rs_evidence.py \
  --episode /absolute/raw_episode \
  --risk /absolute/offline_result \
  --output /absolute/new-rs-comparison.json
```

该模式校验原始摘要、时刻对齐、实测关节特征及冻结自碰撞对覆盖，另通过实际 `TaskRules` 验证五个合成缺测/正证据案例。不会启动物理或 GPU 查询，不写回原始数据，不生成替代历史风险标签。它不是完整 HS/PT/RS 历史重放，也不验证全部任务轨迹。

2026-10-24，11 个 articulation 任务已有的 77 个短流程时刻均通过比较；正常有效输入的关节特征保持一致，每个时刻保留 119 对 RS 检查。数值规则、IR 禁用及未知语义保持不变。机器可读进度见 `task_suite_validation_status.json`。

## 溯源与边界

修改前 task_features SHA-256：`43959bd4f0012157b87d634823c9916b8983a0cf40571615f80ed28e348841a1`。

修改后：`181bf8f8f33330eb91451162e39bdd63d94aadf7d573fc0afed03f89b6696eed`。

新采集冻结新代码。历史数据仍由自己的冻结源码及 gDist 构建回放；修改特征提取后的显式重算必须使用新目录并标明迁移。上述比较报告不授权覆盖历史标签。

权威故障通道不可用仍报告缺测。这些检查证明 RS 证据处理的特定性质，不能替代 HS/PT 资产适用性、动态风险关系和跨任务代表性验证，也不宣称系统已通过真实机器人安全认证。

补充运行验收：修复后新采 Coastal 冰箱 6 动作/24 物理步/7 标签；capture、audit、evaluate、operation-scope verify 及本检查器均通过。原始摘要 SHA-256 为 `0d660e6022083954fa6152de37f7743825c6030ab694178aac6dff7bb6930122`。该短段验证新源码实际接入流水线，不是完整开门任务。
