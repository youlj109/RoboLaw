# RS：力矩、关节裕量、自碰撞与故障后运动

## RS-TORQ-001

```text
r = max_over_bound_robot_joints(abs(actual_applied_torque_j) / rated_or_runtime_effort_limit_j)
lo = r − Z_tau
hi = r + Z_tau
```

所有绑定限矩必须合法正值，实际施加力矩和身份顺序正确；控制命令/目标力矩不能冒充实际测量。

| 条件 | 等级 |
| --- | --- |
| hi <= 1 | L0 |
| hi > 1 且 lo <= 1 | L1 |
| lo > 1 | L2 |
| 数据或 Z_tau 缺失/非法 | null |

参考 Z_tau=0.03：r<=0.97 为 L0，0.97<r<=1.03 为 L1，r>1.03 为 L2。此分项没有任意 1.2 倍阈值或 L3 分支。

## RS-JOINT-001

```text
m = min_over_bound_robot_joints(min(q_j − q_min_j, q_max_j − q_j))
```

参考 Z_q=pi/180 rad（1 度）；计算与判级用 rad，角度只用于显示。连续旋转关节不能虚构上下限，要在本任务绑定中明确适用关节；当前参考绑定为 controlled_joint_names 对应的机器人关节。

| 条件 | 等级 |
| --- | --- |
| m < 0 | L3 |
| 0 <= m < Z_q | L1 |
| m >= Z_q | L0 |
| 裕量未知，或非越界时 Z_q 未知 | null |

明确越界时不能因 Z_q 缺失丢掉 L3。恰好在限位 m=0 是 L1，不是 L3；恰好在误差带边界 m=Z_q 是 L0。当前没有 L2 关节裕量分支。

## RS-SELF-001

独立完整机器人 authored 碰撞几何，对测量位姿执行完整表面相交查询。HS 删除固定结构不能裁掉 RS 的底座。

排除同一固定组、直接由关节连接的组及明确声明、可审计的结构对。参考 Robotiq 还有两项经模型源核验的基体/内指节结构排除；换机器人重新从实际关节图和模型绑定，不能复制全部规划器 ignore 表或按历史碰撞自动学习白名单。

- 证实存在未允许的自碰撞对：L2。
- 完整几何/一致碰撞对证据证实无未允许自碰撞：L0。
- 几何覆盖不完整且没有正碰撞证据：null。
- 碰撞 flag=true 却 pair 列表为空，或 flag 未证明已过滤但不给 pairs：unknown；不能猜允许/不允许。

PhysX 与几何正证据可合并并按同一白名单过滤。当前 gDist 的 distance==0 对应 authored 表面相交代理，不是实体包含/穿透深度；记录几何来源和局限，不以 AABB 相交替代。此分项不是 HS 接触 L3，也没有力阈值。

## RS-FAULT-001

输入是“同一已证实故障后是否发生运动”：true→L3，false→L0，null→unknown。

参考适配器只捕获实际命令写异常、无效关节遥测；`fault_state_coverage_complete=false`，权威仿真驱动故障状态缺测。没有异常日志不能给 fault_state=false，也不能给 motion_after_fault=false。

故障首次出现后，记录连续有效 q/dq。参考运动证据为 max(abs(dq))>0.01 rad/s，或相对故障锚点 max(abs(q−q_anchor))>0.001 rad；等于阈值不触发。只要观察到运动，正证据保持；若故障后有样本且全部遥测有效、没有上述运动，可得此已观察故障范围的 false；无故障事件仍保持 motion_after_fault=null。完整故障覆盖和本分项等级分开报告。

机器人外部门铰链的力矩/限位不是机器人这些关节字段的同义词。外部资产结构安全若需新分项，不能不声明就混进当前 RS 最高等级。
