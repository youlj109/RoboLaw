# 只采原始证据，离线计算风险

`collect_kitchen_vectorized.py --risk-capture-mode raw-only` 是显式选择的新模式，默认 `online` 模式保持原有行为。raw-only 不输出 L 等级，不执行逐帧网格测距、HS/PT/RS 判级或场景摆放判定。结果的 `evaluation_status` 为 `pending_offline`。

## 同步保留与异步写出

- 120 Hz：机器人 q/dq、部件位姿、实际力矩与限位，瓶子/碗状态，实际 PhysX 指令写入，原始 actor/collider 接触事件及接触点、法向、冲量、分离量。
- 30 Hz：动作前的实测观测，包括完整厨房状态、机器人状态、动作和原有任务成功信号。物理步号 `4*t-1` 将当前观测关联到刚完成的物理状态。
- 初始化一次：完整碰撞表面按几何指纹保存为 `raw_meshes/*.npz`，冻结对象/关节清单、坐标关系、规则配置、豁免、相机配置和原始观测结构。网格来自原有绑定流程，构建阶段仍可初始化 GPU/BVH；采集循环内没有几何查询。
- 数据交给容量 64 的独立写进程；快照复制后才提交，队列满则等待，写进程异常则失败，禁止静默丢弃。结束时排空队列、关闭文件并 fsync，生成计数和 SHA-256。

原有控制器的规划、任务状态判断和成功检测仍在线执行，因为它们决定后续动作。风险观察器没有控制权限。权威故障状态和损伤传感器仍缺测，raw-only 不会补造这些输入。

## 文件

| 文件 | 内容 |
|---|---|
| `raw_evidence.jsonl` | `initial_state`、`observation`、`physics`、`fault_event` 原始快照 |
| `raw_meshes/<fingerprint>.npz` | 缩放已烘焙的完整 authored 碰撞网格，含顶点和三角面 |
| `raw_capture_schema.json` | 频率、坐标、厨房元数据、离线恢复约定 |
| `safety_observer_binding.json` | 冻结的规则和绑定，明确 pending_offline |
| `raw_writer_result.json` | 写入数量、文件校验和、fsync 状态 |
| `raw_capture_summary.json` | 实际保存数量，在线测距/判级计数，队列和写出状态 |
| `trajectory.h5` | 与原始观测精确对应的标准 30 Hz 轨迹 |
| `physics_states.npz` 等 | 原有 clearance 审计输入；可稍后执行原有审计 |

`--defer-post-collection-checks` 仅允许搭配 raw-only 使用，将原有执行后 clearance 检查改成保存其输入。`passed` 为 null，状态为 pending_offline，不能视为已通过风险或生产准入检查。

## 使用

必须提供 `--observer-config` 和 `--record-render-state`，确保离线所需状态、频率和绑定完整。沿用原有 episode specs 和 collector overrides 可以保持同一任务轨迹。

```bash
bash scripts/arena_python.sh scripts/collect_kitchen_vectorized.py \
  --episode-specs /path/to/episode_specs.json \
  --collector-overrides /path/to/collector_overrides.json \
  --num-envs 1 --output datasets/raw_run01 --collection-purpose diagnostic \
  --observer-config configs/safety_observer_arena.json \
  --risk-capture-mode raw-only --defer-post-collection-checks \
  --record-render-state --defer-clearance-diagnostics \
  --device cpu --physics-threads 0 --torch-threads 1 --episode-length-s 180
```

验证原始数据（不生成风险等级）：

```bash
PYTHONPATH=. .venv-lerobot-export/bin/python scripts/validate_raw_capture.py \
  --episode datasets/raw_run01/env_000 --output reports/raw_run01_validation.json
```

## 计时口径

`collection_loop_s` 包含必要规划、物理执行和原始数据提交。`raw_capture_loop_and_flush_s` 再加写进程排空、关闭及 fsync。`raw_capture_ready_since_process_start_s` 还包括环境初始化和网格冻结。这些计时都不包括 AI 对话间隔；离线评价、审计、渲染和视频导出另计。

当前 raw-only 是完整原始证据采集入口，完整离线标签导出流程需使用这些冻结输入执行，不能把没有生成标签的 raw 数据宣称为已完成风险标注。

## 2026-10-22 完整轨迹实测

同一人体芥末瓶任务成功，2,292 组动作观测、9,168 个物理步，动作、关节、机器人/瓶子/碗/厨房部件姿态与原在线采集逐值完全一致。原始采集循环 308.141 秒，队列排空和 fsync 后 308.181 秒；包含必要路径规划 126.884 秒，物理执行和原始证据采录 177.747 秒。从启动到原始证据落盘 318.077 秒，采集程序退出 321.122 秒。后续原始完整性验证另需 11.098 秒。

冻结 564 份碰撞网格，保存 48,753 份原始接触报告、4,484,170 个接触点及 27,504 次实际指令写入；原始流约 1.128 GB。在线几何查询和风险判级次数为 0，风险标签仍 pending_offline。相对在线采集循环的 1,215.181 秒，原始采集阶段提速约 3.94 倍；离线处理耗时不能视为消失。测量与验证见 `validation_raw_capture_20261022.json`。
