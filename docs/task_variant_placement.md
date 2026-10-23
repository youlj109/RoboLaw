# 跨任务风险障碍放置

适用于已有原生任务契约的 pick-and-place 和 articulation。障碍配置只决定资产、放置约束及本次抽样，不预设 HS/PT/RS 的等级，不改风险规则。

## 两种放置方式

- `world_pose`（默认）：给出 `position_xyz`、`rotation_xyzw`，用于明确核对过本次任务的世界位姿。
- `native_relations`：给出当前任务契约的 `task_sha256` 和 `placement_relations`，由原生场景放置器联合求解。本模式禁止同时给世界位姿，不复制其他厨房的坐标。

共同字段为 `asset_id`、本地 `usd_path`、布尔 `static`、`category`（HS/PT/RS）、`placement_evidence`。`asset_id` 不能覆盖已有任务资产。人体特殊表面省略仍按独立的 `hs_surface_omissions` 精确绑定，不随关系放置自动扩展。

每条关系显式指定 `kind`、`params`；`on`、`next_to`、`face_to` 还要指定 `reference`，必须是当前任务已声明且已加载的资产 ID。支持 `rotate_around_solution`；不接受未知参数。须至少有一条位置关系，单独朝向约束不足以放置障碍。

资产的放置关系与风险豁免分别管理。例如玻璃杯在台面上，并不会因此被加入搬运物的正常目标容器豁免。

## 随机参数

当前 Arena live sampler 没有将 `RandomAroundSolution` 的范围应用到最终布局。本入口明确拒绝该标记，避免不同 seed 输出相同布局却被误认为完成随机化。

`placement_parameter_ranges` 在原生求解和摆放验证之前抽样，每项为：

```json
{
  "relation_index": 1,
  "parameter": "distance_m",
  "uniform": [0.07, 0.09]
}
```

允许随机化的字段：

| 关系 | 参数 | 单位或范围 |
| --- | --- | --- |
| `next_to` | `distance_m` | 米，严格大于 0 |
| `next_to` | `cross_position_ratio` | 无量纲，区间位于 [-1, 1] |
| `rotate_around_solution` | `yaw_rad` | 弧度 |

验证容差、损失权重和风险阈值不参与这项抽样。每个 task/variant 必须显式定义范围，不继承旧场景范围。求解器可能同时调整本任务其他可放置资产，因此参数区间不能解释成所有物体的世界坐标位移上限。

采样使用本次 `placement_seed`（缺省使用 `seed`），冻结 `python.random.Random/MT19937` 身份、Python 版本、分布和实际参数值。`capture_context.json` 保存抽样后的放置声明，`variation_snapshot.json` 保存最终实际位姿。种子不代替这些实际记录。

## 静态障碍与原始证据

原生通用位姿写入器假定对象是刚体，不能直接对 `AssetBaseCfg` 调用 `write_root_pose_to_sim`。本入口对静态关系资产采用独立 USD 根变换写入器，保留原始静态碰撞体、缩放和子网格；不转换为运动学刚体。当前限单环境采集。

写入后保存环境内 xyzw 位姿、世界矩阵及矩阵回读误差。HS/PT 冻结静态世界三角形；动态障碍继续保存逐物理步真实刚体状态。场景配置阶段不计算风险等级；距离、特征、HS/PT/RS 与视频仍离线。

`placement_validation` 表示原生摆放检查结果；`on` 关系不证明运动过程中稳定支撑。离线支撑、抓持、掉落继续依赖实际接触和运动证据。任务成功和正式采集配额须另行验收。

## 当前验证范围

- Replicator L 型芥末瓶任务＋高水杯：两组 seed 的采集、原始审计、离线 gDist 评价均通过，实际位置和朝向不同，原生摆放检查通过。41 个碰撞表面接入 19 条 PT 关系，每个安全时刻有 57 项关联分项结果。
- Coastal 开冰箱＋固定钢夹具：静态关系放置、短段采集、原始审计和离线评价通过。静态物体写入的矩阵回读误差为 0；在原始采集关闭后另作 PhysX 射线诊断，7/7 个夹具碰撞部件均在冻结网格的对应位置命中。诊断没有推进物理步，也没有向风险评价注入射线距离。
- 完整任务轨迹、其余风险障碍、其他任务的风险变体及四维联合随机范围仍待验收。以上保持动作测试不计入 200 条正式配额。

`category: RS` 表示场景所选障碍类别，不直接赋予 RS 危险等级。最终等级仍来自实际已知且适用的冻结规则。材质损伤、权威故障状态与不适用的停止模型缺口不能由场景标签补齐。
