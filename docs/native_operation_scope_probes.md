# Articulation 操作表面的运行时检查

本检查针对开门、开柜、按键及旋钮任务的**条件距离范围**。它使用真实物理采集和离线 gDist，但不执行完整任务，不计正式采集配额，也不证明操作成功、资产材料参数或停止模型已标定。

## 检查哪些条件

正常操作的距离省略来自 `configs/risk_task_semantics.json` 中当前任务的确切配置：任务 hash、操作 ID、资产 ID、关节、操作碰撞表面和夹爪部件。只有实际记录的 `operation_id` 匹配且 `phase == operate` 时生效。省略指定把手、按钮或旋钮表面，不扩大到整个门板、冰箱或橱柜。

每个操作检查四种上下文，并重复进入和退出操作阶段：

| 上下文 | 操作表面距离的预期处理 |
| --- | --- |
| `settle`，无操作 ID | 保留原范围 |
| `operate`，匹配当前操作 ID | 仅省略配置中的表面 |
| `operate`，无操作 ID | 保留原范围 |
| `settle`，有操作 ID | 保留原范围 |

部分省略后仍有剩余目标表面时，继续查询剩余表面并保留距离规则；全部目标表面均省略时，不提交该关系的距离查询，不保存零值、空距离或相应 `PT-DIST-001`。两种情况都保留力、四物理步冲量及其规则。RS 独立关节图的自碰撞对不得因 PT 的操作豁免变化。

## 运行

在已经恢复项目资产、依赖和 gDist 的 Isaac Sim 项目 Python 环境中：

```bash
python scripts/probe_native_operation_suite.py \
  --output /absolute/path/new-operation-probes \
  --asset-cache /absolute/path/asset-cache
```

默认覆盖当前操作表面目录中的全部任务。可用 `--tasks <task_id> ...` 明确选择子集；默认 seed 为 42。输出目录必须不存在，防止覆盖原始证据或失败结果。每阶段默认超时 600 秒；超时终止本阶段进程组，保留记录，然后继续下一任务。

控制输入由 `kitchen_safety.task_phase_probe_policy:create` 生成。它仅支持明确绑定的 DROID 7 关节加二值夹爪、30Hz 动作接口：每步使用本次实际测量关节位置作为保持目标，夹爪命令为打开，按冻结上下文标记阶段。不会复制旧任务状态或伪造物体动作。阶段标记用于检查条件分支，不能作为“已经操作了门或按钮”的证据。

每个单操作任务提交 6 个动作，记录 24 个 120Hz 物理步及 7 个安全时刻（包括末端观测）。其他多操作任务按声明逐项生成上下文，不能假设永远只有 `op_00`。

## 在线与离线边界

1. 在线保存真实状态、提交命令、接触和冻结场景。在线风险网格查询和判级计数必须为零。
2. 运行原始数据审计。
3. 使用本条冻结实现、规则和 gDist 身份离线计算网格距离与 HS/PT/RS。
4. `scripts/verify_operation_scope_probe.py` 读取已完成的原始数据与离线结果，检查范围和输出。该校验器自身不启动物理或网格查询。

单条已有诊断数据也可单独检查：

```bash
python scripts/verify_operation_scope_probe.py \
  --episode /absolute/path/run/capture/native_000/raw_episode \
  --risk /absolute/path/run/risk/native_000 \
  --output /absolute/path/new-verification.json
```

不要用当前实现直接重算源版本不同的历史数据。本命令可以审阅已有历史结果；重新计算须使用其冻结实现，或按迁移流程另建目录。

## 校验强度与输出

校验器逐帧核对原始 SHA、动作与物理时间、任务范围 hash，以及：

- 确切省略表面集合，与实际查询剩余表面集合；未被选中的关系不能消失。
- 完全省略的距离字段、几何结果和距离规则均不存在；不能写伪 L0 或零距离。
- 距离部分省略时，剩余表面仍有完整 gDist 结果。
- 力/冲量特征和 `PT-FORCE-002`、`PT-IMP-002` 保留。
- RS 自碰撞对完整且无重复；IR 保持禁用。
- 进入操作、缺少操作 ID、阶段不匹配及退出操作的覆盖。

根目录 `suite_probe_report.json` 记录任务、阶段 PID、退出码、超时和墙钟时间；每条 `operation_scope_verification.json` 保存逐帧结果及证据 hash。失败阶段不被后续成功覆盖。`passed` 仅指这些检查通过，`full_task_trajectory` 和正式配额标志保持 false。

缺测仍为未知，等级仍由冻结规则确定。该检查不会补造权威故障状态、门板停止参数，也不会将正常铰链/滑轨的整个相邻物体对自动豁免。结构正常接触表面的绑定、完整操作轨迹、人体/其他障碍变体及随机化边界需要独立验收。

## 2026-10-24 verification result

All 11 articulation task probes completed capture, raw audit, and offline evaluation. Eight passed the strict scope/output verifier. Large Scandinavian microwave, oven, and oven temperature knob failed because gDist reported BVTT capacity overflow and returned missing geometry. These are failures, even though the evaluation process exited successfully. The short measured-hold probes validate conditional scope routing; they do not establish complete task-motion acceptance. Evidence: `task_suite_validation_status.json`, key `operation_scope_suite_20261024`.

The deliverable is the multi-task HS/PT/RS evaluation system. The user explicitly clarified that collecting 200 episodes per group is not required. Material randomization is visual-only; friction and other physical properties remain unchanged. Necessary targeted safety validation remains required.

后续修复结果：`native_operation_scope_capacity_safe_20261024` 中11项已全部通过新采集、审计、离线评价及严格验证。旧8通过/3失败的记录保留为历史证据；新后端身份与检查范围见 `gdist_capacity_safe.md`。
