# 另外四个 Replicator mustard 任务：独立运行入口

本页记录 CPU 准备情况，不代表已经完成物理采集或安全增广验收。入口为
`scripts/extensions/replicator/mustard_family_collect.py`，不修改已认证的 G 型 collector。

## 当前范围

| 原始布局 | CPU 已完成 | 当前运行输入 | 仍待验证 |
| --- | --- | --- | --- |
| L 型带中岛 | 源关系、初态源几何、完整初始机器人、实际 G 抓法比较、原初始 q 到抓取路径、分支 10、闭合区域与名义 lift 检查 | `l_island_controller01.json` | 新厨房中性、cuRobo、真实受力抓取及完整任务 |
| 半岛 | 源关系、初态源几何、完整初始机器人、迁移抓点的原初始 q 到抓取路径 | `peninsula_controller01.json` | 闭合与真实受力、中性、cuRobo、完整任务 |
| L 型 | 源关系、初态源几何、完整初始机器人、独立开放夹爪抓取路径 | 暂无物理运行控制配置 | 闭合区域、完整规划与物理验证 |
| U 型 | 源关系、初态源几何、完整初始机器人、独立开放夹爪抓取路径 | 暂无物理运行控制配置 | 闭合区域、完整规划与物理验证 |

控制配置及 CPU 报告位于 `reports/replicator_preparation/mustard_family_cpu02/`。
它们保留所有旧失败；不复制 G 型绝对坐标，也不改原始站台、yaw、On/NextTo 关系或 2 mm 非接触门槛。
L 型带中岛使用分支 10 作为 IK 数值初值，仍从原 DROID 初始 q 执行。半岛使用该厨房独立求出的 IK 分支。

## 已完成的本轮 CPU 检查

- 44 个测试通过，包含 18 个源支撑查询测试、7 个 provenance 测试、6 个部分执行记录测试和 3 个独立新进程启动测试。
- `l_island_collector_dry05/preflight.json`、`peninsula_collector_dry05/preflight.json` 均通过，均标记 `execution_verified=false`。
- `collector_source_inventory_cpu05.json` 保存 101 / 100 项当前源码及输入起止哈希；每份原始 `source_relation_audit.assets_sha256` 的字面路径均显式捕获。该报告未构建真实场景。
- 实际 assembled DROID 的 runtime 路径、原 source-relation 路径和期望 SHA 在运行时另行绑定。任何缺失、歧义或内容差异都会拒绝。
- 背景实际 recipe 路径由环境运行元数据提供。L 型带中岛沿用 KettleA 64/4 与冰箱外壳安装；半岛的 KettleB 不应用 KettleA 参数。

`dry02` 预检曾因修改 recipe 元数据导致准备报告哈希失配而失败。已恢复原 recipe 字节，将路径标识放入环境运行元数据，再生成 `dry03`；未覆盖历史报告或放宽哈希检查。

真实 `l_island_mustard/family_neutral01` 已尝试启动，但在 Kit 初始化时因预先导入独立 USD 0.25.5 与 Kit USD 0.25.11 冲突而失败，没有物理记录或视频，不能算中性保持。修复将启动前路径限制为纯 JSON、哈希和源码快照；原完整几何核验在 Kit 初始化之后、环境构建之前执行，门槛不变。测试确认预启动输入阶段没有 pxr/omni 模块，官方 Arena bootstrap 后没有 pxr/omni.usd。还用假 context 验证了强制 exit 0 前失败证据已经落盘。

## 按序执行

所有 Kit / 渲染 / cuRobo 运行由机器调度者执行。即使 `--device cpu`，相机与 cuRobo 仍需 GPU；不要与其他采集或训练并行启动。下面统一使用已有成功条件 CPU PhysX、2 个物理线程、2 个 Torch 线程。程序内部使用公共 GPU 锁，新输出目录必须不存在。

先运行 L 型带中岛的 45 秒中性保持：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/mustard_family_collect.py \
  --controller-profile reports/replicator_preparation/mustard_family_cpu02/l_island_controller01.json \
  --neutral-seconds 45 --device cpu --physics-threads 2 --torch-threads 2 \
  --output reports/replicator_execution_20261023/l_island_mustard/family_neutral02
```

只有该目录 `result.json` 的 `neutral_probe_passed=true`，才运行物理任务：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/mustard_family_collect.py \
  --controller-profile reports/replicator_preparation/mustard_family_cpu02/l_island_controller01.json \
  --neutral-episode reports/replicator_execution_20261023/l_island_mustard/family_neutral02 \
  --device cpu --physics-threads 2 --torch-threads 2 \
  --output reports/replicator_execution_20261023/l_island_mustard/family_base01
```

半岛独立运行自己的中性任务：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/mustard_family_collect.py \
  --controller-profile reports/replicator_preparation/mustard_family_cpu02/peninsula_controller01.json \
  --neutral-seconds 45 --device cpu --physics-threads 2 --torch-threads 2 \
  --output reports/replicator_execution_20261023/peninsula_mustard/family_neutral01
```

通过后，用同一配置将 `--neutral-seconds 45` 替换为
`--neutral-episode reports/replicator_execution_20261023/peninsula_mustard/family_neutral01`，输出改为该任务的 `family_base01`。

仅复核 CPU 输入可加 `--dry-run` 并使用新输出目录；该选项不启动 Kit、CUDA 或物理步。

## 源支撑接触与执行证据

新 lift 查询只处理当前任务 mustard 与其原始支撑面这一对。它要求最后 10 个连续真实 close 子步、双指有效载荷、接触点位于原瓶身区域、当前六夹爪关节、实际 TCP 与瓶体姿态、实测 attachment，以及当前支撑 mesh 拓扑和变换全部绑定一致。名义 CPU 几何检查不能授权实际执行。

cuRobo 原始世界约束拆分为机器人对全部世界、瓶体对非支撑世界、瓶体对原支撑三部分；完整自碰撞、关节限制、其他世界对、独立源几何 FCL 均保留。全路径仍按原门槛检查；没有修改 PhysX 碰撞、质量、姿态或物理约束，也没有目标碗接触豁免。

所有真实物理子步从 reset 初态开始保存，包括 8 个 warmup 动作和 5 秒初始沉降；保留实际原始谓词、相机时轴、接触和背景状态。输出含原始视频及解码副本，失败路径与未执行候选保留。运行中的源码及原始输入必须满足起止哈希一致。

每段规划保存自身 action 的 before/after 物理计数。实际执行途中停止时标记已执行；没有任何物理步才标记未执行；计数缺失则标记 unknown。失败不是“未执行”的同义词。`family_runtime_freeze03.json` 包含部分执行记录及启动顺序修复，替代 freeze02。

`launch_state.json` 记录实际启动阶段，`prelaunch_provenance.json` 在 Kit 前保存输入快照；启动失败保存 `launch_failure.json` 和明确失败的 `result.json`。如果已有实际采集结果，不覆盖它。异常证据在 context 退出前保存，避免 Kit 直接终止进程导致报告丢失。正常 Kit 关闭也可能直接退出，不能仅凭退出码或最后一个启动标记认定物理成功。

`physical_trial_passed` 仅表示本次 collector 内部条件通过，`accepted` 始终为 false。最终仍需匹配任务时长的中性保持、门可动性、固定台架/源关系、真实 q 自碰撞、相机标定与人工画面检查；随后才开展人、财产、障碍物变体的必要性配对与实采。当前两个 baseline 均未在本入口真实执行。
