# G bowl/sink：目标底面接触的诊断与修复方案

根代理已批准实现下述独立、有界诊断；实际 GPU/Kit 执行仍由根代理排队。它不是新的生产验收阈值，正常采集器未修改，attempt_03 仍失败。

## 已确认的证据

第三次真实执行把运输落点误差从 attempt_02 的 49.400 mm 降到 0.587 mm，通过原 5 mm 门禁并完成下放。释放动作第 3 个物理子步（3619）首次发生原碗与原 `Sink/BasinBottom` 的源三角形相交：底部顶点低于水平源顶面 0.135684 mm，FCL 17 个三角形接触最大深度同值；法向沿原底面竖直方向。同一子步，原 bowl→BasinBottom 过滤传感器首次记录 6.41877 N 向上支撑力。

这些是“原目标首次接触”的证据，不是稳定放置成功。碗线速仍为 0.101775 m/s，高于原任务 0.003 m/s；原成功谓词所有样本均未成功。完整原始状态为 0..3619，共 3620 行，452 个完整动作加最后动作 3/8 子步。剩下的 5 步没有执行，不能补造。453 组 RGB 最后一组是释放前 3616，未捕获 3619 的图像。

当前存在两个应分开的规则问题：

1. `bowl_destination_query.py` 的候选下放保持源底面 ≥2 mm；但继承的 `task_collision._measurement` 在 `lower`、`release`、`retreat`、`final_settle` 全部把该指定对的距离要求变为 0。真实下放末端因此在 0.75951 mm 处仍通过。这与“未接触下放保留 2 mm”的要求不一致。
2. 同一旧规则即使把距离设为 0，仍拒绝每次源三角形相交。因此它也可能把刚体开始获得原目标支撑时的接触直接当成失败。不能仅凭一次 0.135684 mm 观测，事后把该值或稍大值设成许可阈值。

## 原源参数与缺失证据

`attempt03_source_support_policy01.json` 核对了原 USD 与本次实际 composed kitchen overlay：碗使用 `convexDecomposition`，BasinBottom 使用 `boundingCube`；两者没有 authored `physxCollision:restOffset` / `contactOffset`。原 `BowlYcbRobolab` 注册项没有额外 collision spawn 配置。未 author 不代表 native 值为 0。

本次 raw 未保存 native collider offsets 或 bowl/floor 的逐接触点和 separation，因此不能从源三角形深度推断 native cooked penetration。源码中的 contact offset 只决定何时开始生成接触；rest offset 决定静态分离距离，两者都不是可直接挪用的“允许穿透阈值”。

现有 6 mm band 原先只绑定 mustard/CapSurface_03 初始源支撑，本任务明确适配到 bowl/CapSurface_02。它既不是上游 sink 的规定，也不是 native offset 测量；不自动扩展到 BasinBottom。

## 建议的生产规则划分（尚未实现）

- `lower`：指定 bowl/BasinBottom 使用原源三角形，原 2 mm 非接触检查在每个实际子步生效；保留所有机器人/底面、碗/墙壁与 drain 等其他对的原检查。若当前控制器无法保持它，应明确失败并修控制器。
- `release` 及后续静置：只有原 bowl/原 BasinBottom 这一对可以进入单独的支撑接触检查。必须绑定原资产/精确底面几何、过滤传感器身份、实际对象姿态、native 接触点/法向/separation 和源接触区域。所有其他对维持原门禁。生产中的有限接触判据需要上述测量及独立验证，当前没有可宣称有效的 penetration 容差。
- 原任务判定不改：源 spawned bounds、位置关系、支撑力 ≥0.1 N、与 +Z 的 45° 支撑锥、线速 <0.003 m/s、原 placement 计数与完整任务序列均独立保留。

## 独立、有界诊断建议（供根代理审核）

这不是正式采集分支。输出固定 `accepted=false`、`physical_trial_passed=false`，保存旧 strict gate 的每次原始失败，不据此改写 attempt_03。

1. **先读取 native 数据。** 从新的正常 reset 起，用现有 PhysX tensor 只读接口取得 bowl 的 `get_rest_offsets()` / `get_contact_offsets()` 与实际 shape 数；静态 floor 若无法通过已验证接口读取，明确记为未测，不从 USD 空值补值。创建唯一 bowl→原 BasinBottom 的 `create_rigid_contact_view`，按既有 `get_contact_data(dt)` 模式逐子步保存原始 contact point、normal、force、separation、count/start，拒绝溢出与不完整缓冲。原任务过滤力传感器同时保留，用于核对方向与身份。不得设置任何 offset、质量、几何或物体位姿。
2. **仍从真实完整流程走到合法释放前状态。** 预抓取、抓持、实测附件、抬升、重新运输与下放路径照常检查。下放实际源距离必须 ≥2 mm。为避免旧固定末端 hold 把碗继续带进 2 mm 区域，建议在下放最后一个计划点已执行后，按预先声明的动作边界检查：源 gap 首次进入 `[3,5] mm`、原落点误差 ≤5 mm、原接触区最新十步双指证据通过、其余实际源检查通过时，结束多余的末端 hold 并开始打开手指。这是**诊断用的待审控制停止规则**；3 mm 来自原目标，+2 mm 来自现有高度检查范围，不是新增接触容差。若没有这样一个合法状态，直接停止，不跳过下放 2 mm 门禁。仅作可行性说明：旧 raw 在计划下放末点之后，3576/3584/3592/3600 的 gap 为 8.439/5.679/3.711/2.474 mm；新诊断必须依据自己的当前测量，不读这些历史位姿当目标。
3. **只观察明确目标接触，最长 7 个控制动作。** 原周期 1/15 s、8 子步，最多 56 子步（0.4667 s）。持续相同的原末端关节目标与开爪动作，不补额外 arm 轨迹；每个物理子步完整记录原三相机对齐、对象、接触、背景与原任务谓词。只在 `release` 中、**仅指定 bowl/BasinBottom 对**产生原 strict failure 时，可把这一失败送入诊断 ledger 而非立即终止；仍不得计通过。
4. **诊断继续条件须逐步明确。** 非指定对全部照旧；原 floor 资产/几何/传感器身份固定，所有源接触点位于原底面 footprint，法向及 native 过滤支撑力满足原 +Z 支撑锥，原对象未穿过底面实体的下边界。缺少 native 接触点、法向或目标身份、超时、其他碰撞/安全/背景门禁失败，立即停止。底面上下边界只是本次有界实验的几何停止边界，**不是生产许可穿透深度**。不引入 0.136 mm 或 6 mm 的新目的地容差。
5. **终态独立报告。** 输出每步原 strict 判定、源顶面 signed gap、FCL 接触深度、native separation、支撑力和完整对象速度，区分 source/cooked 表示差异、瞬态求解穿入与静态残留。即使原成功谓词短暂为真，本诊断仍不得转成正式成功样本。生产规则只有在这些证据明确后另行讨论、CPU/原 CUDA 检验及新真实基线验证。

## 证据路径

- `reports/replicator_execution_20261023/g_bowl_sink/attempt03_floor_contact01.json`
- `reports/replicator_execution_20261023/g_bowl_sink/attempt03_floor_contact01_output_verification.json`（原脚本已成功写报告，随后仅 stdout 的 NumPy bool 序列化失败；这里独立核对有效报告，原报告和脚本均保留）
- `reports/replicator_execution_20261023/g_bowl_sink/attempt03_source_support_policy01.json`
- `reports/replicator_execution_20261023/g_bowl_sink/attempt03_partial_evidence_review01.json`
- `reports/replicator_execution_20261023/g_bowl_sink/attempt03_raw_montage01.jpg` 及逐原帧 SHA manifest。

## 独立实现与运行入口

新增 `kitchen_safety/extensions/replicator/bowl_destination_probe.py`，仅独立诊断入口导入；正常 `bowl_collect.py` 与 `BowlCollisionWorld` 保持原样。新入口为 `scripts/extensions/replicator/bowl_destination_contact_probe.py`，固定声明在 `configs/extensions/replicator/g_bowl_destination_contact_probe.json`。这三者在 App 启动前捕获。声明不允许通过 CLI 增大 7 动作 / 56 子步窗口。

CPU 检查覆盖原 2 mm 下放、完整 native 缓冲、错误目标/受力/法向/区域/时间边界、不得使用缺失的真实 raw contact、继续指定对后仍拒绝其他碰撞。测试中的合成 native 点只用于测试分支，绝不写成真实接触证据。原 attempt_03 实态 3592 的合法间距通过，3608 的不足 2 mm 间距拒绝，3619 缺 raw 接触点时拒绝。

根代理调度实际诊断时使用以下命令；目录必须是新的 `reports` 路径，不能写入 `datasets`。正常结束也固定 `accepted=false`、`physical_trial_passed=false`、`task_success=false`。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/bowl_destination_contact_probe.py \
  --device cpu --physics-threads 2 --torch-threads 2 --seed 42 --headless \
  --output reports/replicator_execution_20261023/g_bowl_sink/destination_contact_probe_01
```

除继承的原始完整记录/相机/原谓词/provenance 外，新增输出：

- `destination_native_contacts.json`：每个实际物理状态的原始点/法向/力/separation、精确目标身份、offset 可读性。
- `destination_source_observations.json`：lower/release 逐状态精确源距离、接触三角形和原 strict 判定。
- `destination_strict_failure_ledger.json`：原指定对所有拒绝及诊断继续条件；正常碰撞失败仍即时停止。
- `result.json`：合法释放动作边界、原成功谓词仅作为观察、是否完成有界观测，以及任何完整性/安全/视频失败。

当前安装 API 已仅通过源码核对：`RigidObject.root_view` 是正确接口，`root_physx_view` 是弃用别名；`get_contact_data(dt)` 实际返回六项缓冲。`omni.physics.tensors/api.py:5040` 将 `normal_force × normal` 定义为 sensor/filter 受力贡献，因此新诊断对正加载 native 点使用**有符号 +Z 支撑锥**，FCL 有序对的源几何法线仍单独检查无向轴。零加载点完整保留且检查位置，但不作为受力方向或支撑证据；负载值为负时保守拒绝。

每个实际子步均保存 `Σ(normal_force × normal)`、原过滤传感器力及差值；差值仅作为诊断，不凭猜测设置新的数值容差。native 整体向下而原传感器向上的负例必须拒绝。安装源码路径、行号和 SHA 见 `reports/replicator_preparation/g_bowl_destination_probe_cpu01/native_api_readonly01.json`。这仍不能代替第一次真实 native buffer 可用性检查。
