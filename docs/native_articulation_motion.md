# 关节资产运动诊断与安全链路验证

## 用途和范围

`kitchen_safety.task_articulation_probe:create` 是明确标记的诊断控制源。机械臂保持实测姿态，外部关节通过 PhysX 接受有限诊断力/力矩，使门板、按钮或旋钮发生实际运动。它不证明机器人完成开门、按键或旋钮任务；即使原生任务成功条件触发，也不能计作机器人任务成功轨迹。

诊断不会瞬移位置、替换碰撞网格、修改驱动刚度/摩擦或风险阈值。它检查测量位姿、COM 运动、接触、离线完整网格距离和 HS/PT/RS 输出是否跟随实际运动。既有 11 个保持探针只有微小求解器扰动，不能替代这项验证。

## 命令与原始记录

配置绑定本任务契约哈希与 operation_id。关节在当前任务资产内通过完整 PhysX DOF 路径唯一解析，单位从实际关节类型决定：转动关节 Nm、移动关节 N；不使用同名叶节点跨资产匹配。

- 命令包含 `version, task_sha256, operation_id, effort_si, effort_limit_si, diagnostic_only`。
- `effort_limit_si` 是诊断命令上限，不是材料耐受值或新增风险阈值。
- 每个 120Hz 物理步之前提交；保存非目标通道原值、名义值、实际读回值、root/dof/index、单位、源步、action 和诊断标识。
- 原始动作前 observation 保存 `diagnostic_actuation`，后续 physics.commands 中记录 `command_scope=diagnostic_external_articulation`；摘要包含 `diagnostic_external_actuation_steps`。
- NumPy、Torch、Warp 视图各用自身张量类型传输命令。Warp 在这里是 PhysX 接口，不是风险测距后端；风险距离仍全部离线 gDist。
- 离开一个非零驱动关节前必须显式提交零值，不能切换关节或取消诊断后让旧力继续作用而不记录。
- 默认任务策略不传该字段；普通采集不提交额外诊断命令。

## 可复现运行

仓库提供 3 个任务绑定的配置，位于 `configs/risk_diagnostics/*.motion.json`。以下在项目 Python 环境执行，RUN 指向不存在的新目录：

```bash
python scripts/risk_capture_workflow.py capture \
  --output "$RUN" --asset-cache "$CACHE" \
  --task-contract configs/risk_task_suite/lightwheel_open_fridge.json \
  --policy-factory kitchen_safety.task_articulation_probe:create \
  --policy-config configs/risk_diagnostics/lightwheel_open_fridge.motion.json \
  --max-actions 30 --seed 42
python scripts/risk_capture_workflow.py audit --output "$RUN"
python scripts/risk_capture_workflow.py evaluate --output "$RUN"
python scripts/verify_articulation_motion_probe.py \
  --episode "$RUN/capture/native_000/raw_episode" \
  --risk "$RUN/risk/native_000" --minimum-displacement 0.01 \
  --output "$RUN/motion_verification.json"
```

按钮验证使用 `--minimum-displacement 0.0001`（m）；门/旋钮为 0.01（rad）。这是诊断的最小可见运动门槛，不能用作安全等级阈值。目标 task-contract 与 motion 配置必须配套。其他任务须重新提供明确的诊断命令上限与操作绑定，不复制旧厨房路径。

配置为 3 个零力动作、20 个驱动动作、7 个零力动作。若原生环境提前终止，记录实际动作数；不能因为未完成整个计划就补造后续记录。此次按钮、旋钮的原生成功条件由诊断驱动触发，始终不作机器人执行能力声明。

## 实测验证结果（2026-10-24）

| 诊断 | 物理步 / 标签 | 实测关节位移范围 | 操纵资产相关 PT 距离最大变化 |
| --- | --- | --- | --- |
| Coastal 冰箱门 | 104 / 27 | 0.33178 rad，约 19.0° | 22.68 cm |
| Large Scandinavian 左侧取消按钮 | 16 / 5 | 3.017 mm | 3.017 mm |
| Large Scandinavian 烤箱温度旋钮 | 32 / 9 | 2.52446 rad，约 144.6° | 1.602 cm |
| Coastal 冰箱门＋明确绑定人体 | 120 / 31 | 0.16752 rad，约 9.60° | 10.64 cm |

以上物理步全部保存诊断实际提交值与回读值。每条离线结果的关系网格与 RS 自碰撞覆盖完整；HS/PT/RS 原规则保持不变。

人体诊断含 5 条“门板→人体表面”关系，共 155 个关系时刻；距离最大变化 7.15 mm。网格结果有效，但门板停止模型未绑定，对应 HS 动态距离与 PT 距离分项仍为未知，不能借用机器人停止模型。人体诊断类别已知最高 HS/PT/RS 均 L0，只表示其已知分项，不能宣传全部规则完整或场景安全。

距离按既有 30Hz 标签时刻计算；按钮的快速运动可能发生在标签间，表中运动范围由 120Hz 原始状态验证。接触仍按四物理步窗口汇总。这次没有悄悄提高距离频率或宣称覆盖两个标签之间的全部连续最小距离。

25 项命令/绑定/张量前端/清零检查通过。修改后的默认入口另跑 24 物理步、7 标签，采集、审计、离线评价和范围检查通过，外部诊断命令数为 0。

首轮因 Warp/Torch 张量类型不匹配失败，单独保存；修复后另开目录采集，没有覆盖失败记录或切换风险距离后端。

## 验收边界

这些结果补充三类关节运动的安全数据链路证据，不代表全部 32 个任务的所有风险变体、摆放范围和机器人操作轨迹均已验收。门板停止模型、非固定结构接合面、权威驱动故障与材料损伤输入仍按各自契约处理。评估正确性验证没有每组 200 条采集配额要求。
