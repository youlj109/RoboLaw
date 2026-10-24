# 离线评价的强制原始证据预检

## 修复的问题

以前原生 `evaluate` 独立入口检查冻结实现、规则身份和部分时序，但不会调用完整原始审计。调用者若跳过独立 `audit`，物理时钟、时间戳、接触覆盖或实际命令存在性等错误可能进入 GPU 后端。正常流水线先执行 audit 不能证明直接入口也有这些检查。

现在 `audit_native_task.audit_episode()` 提供同一只读审计函数；`evaluate_native_task.py` 在通过冻结代码身份检查后、加载 gDist 后端前强制调用。时钟、顺序、哈希等结构错误拒绝评价，不分配风险查询后端、不创建安全标签；状态/接触覆盖不足单独报告并允许规则按有效证据判级。准入后保存 `risk/native_000/raw_preflight.json`，并在 `evaluation_summary.json.raw_preflight` 记录 passed、evaluation_admissible、coverage_gaps、raw_sha256、physics_steps、observations、report_file。`passed=false` 且 `evaluation_admissible=true` 表示存在覆盖缺口，不能宣称原始证据完整。

独立 `audit` 命令继续可用，用于提前诊断。其退出码 0 表示 `evaluation_admissible=true`，不表示 `passed=true` 或数据覆盖完整；覆盖不足可继续离线判级，结构错误返回非零。评价入口仍自行复核本次源数据，不信任缓存的通过标志。末尾原始哈希核对继续保留。全部工作发生于离线，不增加在线风险计算。

## 不改变的语义

这道检查核对原始文件完整性、时间和实际通道覆盖，不将每个指标的缺测视为损坏。例如权威故障状态缺失、未绑定的停止模型仍由规则报告未知；原始记录损坏/时钟错误则拒绝评价。布局检查失败独立报告，不能用 raw preflight passed 覆盖。规则阈值、HS/PT/RS 聚合及 IR disabled 均未修改。

新采集冻结两个脚本的哈希。历史数据按原冻结实现回放；若要用新入口复算，另建显式迁移与输出目录，不改旧归档。

## 验证结果（2026-10-24）

- 13 个入口案例：1 个正常输入及 2 个覆盖不足输入到达模拟后端加载哨兵；其余 10 个结构异常在此之前拒绝。覆盖不足包括状态或接触通道不完整；结构异常包括物理/动作时钟、三类时间戳、无命令、观测非因果、无终止采样、执行中止及哈希不符。哨兵不运行 GPU 或生成等级。
- 对照旧评价器：上述 10 个结构异常均会到达后端加载，不表示它们一定都能通过旧流程末尾的所有检查。
- 独立缺测聚合案例通过：HS 接触未知时，已知距离 L2 和最终 HS L2 均保留。
- 真实原始记录的独立合成缺测副本验证通过：仅将 12 步接触覆盖标为不足，保留 4,818 条已知 PT 距离规则，8,128 条力与 8,128 条冲量规则均保持未知；RS 已知等级保留。原始源哈希前后不变。这是明确标识的测试副本，不声称真实传感器发生过故障。
- 原有 24 项故障时序/审计案例通过。
- 已有 32 个任务的原始证据只读预检全部通过；未重写原始数据或历史标签。
- 新 Coastal 冰箱保持动作案例，直接执行 `capture → evaluate`，不先执行独立 audit：12 个物理步、4 个标签通过。预检看到 36 次实际命令提交；在线采集约 14.08 秒，离线评价约 2.77 秒（包含预检）。这是短程接口验证，不是开门成功证明。

新原始 SHA-256：`3d1f85cf45eb3760ac485611b3659b52ebb5afc86e671f4ce516eb2289896dd8`。

```bash
python scripts/check_native_evaluation_preflight.py --output /absolute/new-preflight-check.json
```

记录位于运行主机 `task_suite_expansion_20261023/native_final_evaluation_preflight_20261024`，机器可读交付结果见 `task_suite_validation_status.json` 的 `native_evaluation_preflight_20261024`。无需每组 200 条。
