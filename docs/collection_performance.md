# 抓放采集性能实测与复现

**最新同配置测量：推荐先用4个原生物理环境。本轮16条输入中，N4得到16条成功的完整三视角RGB轨迹，70.705条/小时；N6得到15条，65.237条/小时。** 两组使用同一套优化后的代码、相同输入顺序、4个后验验证worker、队列容量8、持久渲染和原三台相机，测试顺序为N6→N4。[最终比较报告](../reports/collection_speed_20261018/optimized_count16_comparison.json)的配置差异仅为环境数。

| 环境数 | 物理成功／尝试 | 完整RGB验证通过 | 整批墙钟 | 成功轨迹／小时 | 整卡显存采样峰值 |
|---|---:|---:|---:|---:|---:|
| **4** | **16/16** | **16** | **814.651 s** | **70.705** | **17698 MiB** |
| 6 | 15/16 | 15 | 827.756 s | 65.237 | 18260 MiB |

N4本轮成功产率高8.38%，且总墙钟少13.104秒。N6的第11号输入在最终放置检查失败，`arena_success=false`、`final_on_destination=false`，垂直位置差0.154597m超过原0.15m上限；其完整物理尝试耗时保留，不渲染为成功数据。N6性能驱动器按 `completed_with_failures` 返回1，属于完成测量后的失败结果汇总；19个实际子进程全部退出0并回收。N4的21个实际子进程也全部退出0并回收。

N6物理角色累计640.023秒，略快于N4的655.434秒，但首条渲染分别要等230.070和148.811秒。N4后续因没有可渲染源轨迹等待20.703秒，N6为0；更大的批次没有在这次有限任务窗口中提高整体产率。两组共55371张JPEG完成全量解码，状态链和三视角对齐均通过。所有计时包含实际启动、失败、规划、物理执行、后验安全检查、RGB渲染与写盘、数据验证以及子进程退出；不包含离线MP4编码。

结论限定为八份不同花瓶布局与外观重复两遍的16条性能负载，各档一次；它不是十类资产或每ID20条、持续200条的全局最优证明，也不把试验中的成功比例当作策略成功率。正式调度器额外的挡路资格和独立距离审计尚未包含在上述产率中。正式200条目录保持暂停，本目录数据不计其配额。原先N4=67.796、N8=63.327的测量使用优化前代码，保留在下节，不混入本表。

```bash
python3 scripts/benchmark_deferred_rendering.py \
  --episode-specs reports/collection_speed_20261018/persistent_count16_input.json \
  --batch-size 4 --validation-workers 4 --queue-capacity 8 --pipeline \
  --persistent-renderer --refresh-render-products \
  --output reports/performance_runs/NEW_OPTIMIZED_N4
```

### 正式采集入口的小批集成验证

可选的 `vectorized` 后端已经接入原采集入口。[独立 smoke](../reports/collection_speed_20261018/production_backend_smoke_vase4_n2/manifest.json)使用 N2、目标4条、最多6次尝试，实际4次全部通过原正式验收，包括挡路资格、独立安全距离审计和三视角数据检查。共2374个动作、7122张JPEG；首条导出三个单视角和一个拼接视频，四个MP4各完整解码630帧。物理第二批与第一批渲染实际重叠，两个物理批次及持久渲染进程均正常结束。

[外部完整墙钟](../reports/collection_speed_20261018/production_backend_smoke_vase4_n2_run.json)为302.227秒，包含正式审计和预览编码，约47.646条/小时。此项仅验证入口可用，环境数、输入、批量长度和验收范围都与上表不同，不能用来比较N2与N4优劣。[原命令加 `--resume`](../reports/collection_speed_20261018/production_backend_smoke_vase4_n2_resume_run.json)以退出码0在1.068秒内完成；具体文件与计数对照见[续采审计](../reports/collection_speed_20261018/production_backend_smoke_vase4_n2_resume_audit.json)。这4条是独立验证数据，不加入已暂停的200条目录。

最新N4/N6另有[元数据与视觉抽查](../reports/collection_speed_20261018/optimized_count16_metadata_visual_audit.json)：31条已渲染轨迹的实际外观和相机参数与参考一致，抽查首帧未见串色或黑帧；不声称逐帧像素复现。全量JPEG解码通过不等同于全量画面人工检查。

## 此前持久渲染管线的 N4/N8 对照

**2026-10-19：持久渲染管线的同16条输入实测中，N4 的完整流程吞吐较高：67.796 条/小时；N8 为63.327 条/小时，两组均16/16通过。** 输入为原八份不同花瓶随机 spec 按原顺序重复两遍，每次都重新规划并执行物理，重复仅用于性能测量。该结论限于这次16条花瓶任务的有限批次，尚未验证十类场景或200条持续采集的统一最优数量。正式扩增批次仍暂停，测试数据不增加200条配额。

上述“通过”包含任务成功、原逐物理步安全检查和完整 RGB 数据验证，**不包含正式调度器额外执行的挡路资格要求与独立距离审计**。67.796 条/小时因此是该性能管线的产率，不能当作正式200条配额的最终验收产率。正式采集的向量化后端已完成上述独立小批验证，仍由原 `verify_attempt` 决定是否计入配额。

| 实用配置 | 任务完成 | 原物理、安全及完整 RGB 通过 | 完整流程墙钟 | 通过/小时 | 整卡显存采样峰值 |
|---|---:|---:|---:|---:|---:|
| N4、队列6 | 16/16 | 16/16 | 849.606 s | 67.796 | 17374 MiB |
| N8、队列8 | 16/16 | 16/16 | 909.569 s | 63.327 | 21139 MiB |

[两组最终比较审计](../reports/collection_speed_20261018/persistent_count16_comparison.json)确认输入顺序和参数一致，两组共有 **56718张三视角JPEG** 通过既有完整解码验证，40个实际物理、RGB验证及共享渲染子进程均正常退出并回收。N4比N8少用59.963 s，完整流程吞吐高7.06%；每档只执行了这一次16条试验，不能据此声称统计稳定或全局最优。

N8的物理角色累计600.358 s，低于N4的691.525 s；渲染开始后的等源时间为0.027 s，低于N4的50.695 s。但N8首批填充需284.321 s、末批排空309.192 s，N4分别为156.402 s和158.044 s，因此这次有限批次N4更快。N8只包含两批物理执行，不能把它的一次顺畅衔接或最后纯渲染阶段的速度当作200条持续采集的实测吞吐。

[后期外观抽查](../reports/collection_speed_20261018/persistent_count16_late_appearance_comparison.json)实际查看两组第8、9、14、15条的三视角首帧，共24组与同spec独立渲染的对照，未见明显串色、黑帧或视场变化。此结论限于抽样帧及本次不透明花瓶，不代表透明玻璃资产验证或逐像素一致。

[N4 原始汇总](../reports/collection_speed_20261018/persistent_count16_n4/summary.json)与[只读审计](../reports/collection_speed_20261018/persistent_count16_n4/read_only_audit.json)确认 **9538 个动作、三视角共 28614 张 JPEG 全部完整解码**，状态链和相机对齐通过。原逐物理步几何安全、携瓶倾角、瓶身接触和末态检查均保留；物理仍为 120 Hz、64 次位置迭代，动作/RGB 为 15 Hz，相机安装、内参及 1280×720 不变。配置为 CPU PhysX、4 个后验检查 worker，以及一个持续接收成功轨迹的单环境渲染 Kit；最多同时运行一个物理 Kit 和一个渲染 Kit，不为每条轨迹重新启动渲染进程。原三台相机各自使用原 render product，下一条外观应用之前刷新 RP，不使用分组相机。

两组共享渲染器均 **构建场景1次、复用15次**，各完成16次外观应用。N4有4个物理子进程、16个完整RGB验证子进程和1个共享渲染进程，N8分别为2、16、1个；总墙钟包含启动、规划、物理执行、原检查、渲染、写盘、完整验证和最终退出。N4未开始消费的已完成源轨迹峰值仅4，小于队列容量6，三次物理批间间隔约11–13 ms，没有容量背压证据；`put`本身没有单独计时，不能虚构其耗时为0。N8使用队列8，因此比较的是两套实用配置，并非严格只改环境数的单因素实验。

这两份已结束的旧 N4/N8 试验共32个 `images.h5`，以及最新已结束的 N6 试验15个图像文件，已归档到 USB 硬盘，共47个、约12.34 GiB，原位置保留主机符号链接和 `RGB_MOVED.md`。完整映射见 [归档清单](../reports/collection_speed_20261018/rgb_archive_locations_20261019.json)。物理档案、相机参数、验证报告及其他元数据仍在原处；现有 Docker 没有挂载 USB，因此容器内重验这些图像前，需要恢复原图像文件或挂载该归档路径。正式扩增数据未移动。

持久服务在接入这轮流水线之前也完成了[两条真实请求验证](../reports/collection_speed_20261018/persistent_service_vase2_smoke/summary.json)：两次请求后进程均保持存活，最后正常退出，含启动及关闭总墙钟 **82.140 s**。[另做的完整验证及画面对照](../reports/collection_speed_20261018/persistent_service_vase2_smoke/appearance_validation.json)通过3678张JPEG、状态链和相机对齐，并实际查看两条各首/中/末帧的三视角，共18对图；第二条正确切换为自己的暖黄外观，未沿用第一条的粉紫/冷白色调。全量解码不等于逐帧视觉完全相同，该画面结论限于抽查范围；82.140 s也不包含后续独立验证时间。

此前 **49.90 条/小时**来自八条输入、每条新建渲染Kit、2个检查worker的旧管线；当前则是16条输入、持久渲染及4个检查worker。批次长度会改变填充/排空的摊销，物理轨迹也重新生成，不能把差异全部归因于某一项优化，或称为环境数已经优化到峰值。下面保留旧结果；最新 N4 的复现命令如下，输出目录需替换为全新路径：

```bash
python3 scripts/benchmark_deferred_rendering.py \
  --episode-specs reports/collection_speed_20261018/persistent_count16_input.json \
  --batch-size 4 --validation-workers 4 --queue-capacity 6 --pipeline \
  --persistent-renderer --refresh-render-products \
  --output reports/performance_runs/NEW_PERSISTENT_N4
```

## 此前八条输入的独立渲染管线比较

**旧管线的 1/2/4 环境实测中，4 个原生物理环境吞吐最高，为 49.90 条/小时。** 物理环境处于同一个 Kit，另有一个单环境 Kit 逐条渲染三视角；最多同时两个 Kit。相同八份不同花瓶随机布局、外观和规划种子各执行一次，全部生成新的物理轨迹，没有复用原成功结果。正式扩增批次仍暂停，测试数据不增加 200 条配额。

| 物理环境数 | 任务完成 | 原物理、安全及完整 RGB 通过 | 完整流程墙钟 | 通过/小时 | 整卡显存采样峰值 |
|---|---:|---:|---:|---:|---:|
| 1 | 7/8 | 7/8 | 553.725 s | 45.51 | 11856 MiB |
| 2 | 6/8 | 5/8 | 461.687 s | 38.99 | 12045 MiB |
| **4** | **8/8** | **8/8** | **577.206 s** | **49.90** | **15241 MiB** |

[汇总及原始结果入口](../reports/collection_speed_20261018/randomized_count8_summary.json)保留全部失败、完整流程计时和显存采样；[独立集中审计](../reports/collection_speed_20261018/randomized_count8_comparison.json)确认三组共 35871 张 JPEG 的既有完整验证结果、相同输入和配置，以及 54 个自有采集/渲染/验证子进程均正常退出。三组均为 CPU PhysX、threads=0、完整渲染状态记录、两个后验验证 worker、队列容量固定为六条；每条在独立新 Kit 中按自己的原始随机光照、材质渲染，再做完整 RGB 联合验证。不使用分组相机或持久 RP 刷新。相机安装、内参、1280×720 分辨率、120 Hz 物理、15 Hz 动作/RGB 和原安全要求均保持不变。总墙钟包括初始化、规划、执行、失败、预热、写盘、验证和全部子进程退出；实际测试顺序为 2、4、1。

4 环境本轮通过吞吐比 1 环境高 9.64%，比 2 环境高 27.98%。它的总耗时也更长，输出分别为八、七、五条，不能把这些差值解释成相同轨迹整体耗时下降，或认定增加环境数提高了物理安全性。四环境启动填充为 173.710 s，此后渲染/验证角色持续有成功源轨迹可处理，没有等待下一条源轨迹；两环境在初始填充后另有约 100.068 s 的源轨迹等待。四环境的最终排空仍需 203.884 s，因此这里只报告八条有限批次的实际吞吐，不把稳态外推当成实测。

**该旧配置下的已测最佳数量为 `--batch-size 4`。** 这是一个资产类别、八份不同布局、每档一次的结果，没有证明 6/8 环境或十类任务的全局最优，也没有统计重复稳定性。之前固定原 USD 外观的 N10=72.35/h 属于不同负载，不能直接用于独立随机光照数据。复测旧配置的单命令如下；输出目录需替换为全新路径：

```bash
python3 scripts/benchmark_deferred_rendering.py \
  --episode-specs reports/collection_speed_20261018/randomized_count8_input.json \
  --batch-size 4 --validation-workers 2 --queue-capacity 6 --pipeline \
  --output reports/performance_runs/NEW_RANDOMIZED_N4
```

## 后验检查进程数：相同档案的 CPU 实测

### 规划计算优化的实际轨迹对照（2026-10-19）

同四份输入、N4、CPU PhysX、4 个后验 worker、不生成 RGB 的[实际前后对照](../reports/collection_speed_20261018/planning_optimization_actual_n4/comparison.json)中，主进程规划使用批量关节旋转、只计算末端变换的 FK，以及改变失败路径检查顺序的完整采样检查后，外部墙钟从 **155.324 s 降至 147.412 s（5.09%）**，其中 `planning_dispatch_s` 从 **82.656 s 降至 75.021 s（9.24%）**。两个分支的 spawned 后验 worker 均使用当前优化实现；这不是整个历史旧版本与新版本的端到端对照，也不包含 RGB。

两组均 4/4 物理成功，扩增挡路资格均为 3/4。2380 个动作、全部 obs/next_obs、18080 个逐物理步状态、FCL 结果、逐动作距离、规划候选和计划均逐值一致。一个环境的136条 finger-frame 接触记录内点排列顺序不同，完整接触点/力/距离多重集合相同；不声称原始接触 JSON 顺序也一致。这里只做了一次 A/B，收益尚未通过重复测量确定稳定性。最初的测试入口在 Kit 启动前导入 SciPy 导致退出139，该启动失败保留于 `before.log`，有效旧分支为 `before_retry`。

同轮[包围盒预筛选独立 A/B](../reports/collection_speed_20261018/union_aabb_prune/ab_randomized8/summary.json)完成8轮、32次完整档案验证，144928个物理状态检查及18116条动作距离记录的全部非计时结果与原档案精确一致。旧/新单轮中位数为18.71/18.49秒，旧分支另有一轮明显偏慢；不把累计差值当作稳定提速。预筛选只减少不可能进入安全距离的候选，原精确几何判定、阈值和采样均保留。

### 验证进程数

2026-10-19 补充的[独立 CPU 重放](../reports/collection_speed_20261018/validation_workers_n4_replay/report.json)直接读取上述 N4 两批的完整 USD、逐物理步状态及动作距离诊断档案，按原 collector 参数重建相同八份检查任务。每批分别启动全新的 `spawn` 进程池，计时包含进程启动、全部任务和退出等待，不包含输入参数读取及报告写盘。

| 验证 worker 数 | 第一批4条 | 第二批4条 | 八条合计 |
|---|---:|---:|---:|
| 2 | 36.238 s | 35.743 s | 71.981 s |
| 4 | 19.080 s | 19.097 s | 38.177 s |

本次 4 worker 比 2 worker 少用 **33.804 s、46.96%**。两组均为 8/8 通过，包含 **36232 个物理采样检查、56168 次 `valid` 调用和 4529 条动作距离诊断**；除计时字段外，所有返回字段、每条距离和检查次数均与原始结果及另一组逐项完全一致，没有减少采样或改变阈值。

这是每批按 2→4 固定顺序各测一次的独立 CPU 检查，没有并发渲染或物理采集，缓存和资源争用条件也不同，**不能把 46.96% 当作完整流程的提速**。上方 N=1/2/4 吞吐表仍是旧 2 worker 配置；后续持久渲染与4个检查worker的新 N4 完整结果见本文顶部。可用[重放脚本](../scripts/benchmark_validation_workers.py)复现，输出目录需为新目录：

```bash
bash scripts/arena_python.sh scripts/benchmark_validation_workers.py \
  --source reports/collection_speed_20261018/randomized_count8_n4 \
  --output reports/performance_runs/NEW_VALIDATION_WORKERS \
  --workers 2 4
```

## 固定原 USD 外观的原生环境数比较

**固定外观同配置扫参：4/6/8/9/10/11 环境中，10 环境观测到最高通过吞吐，10/10 通过、72.35 条/小时。** 这是当前配置和已测数量的单次观测结果，不是全局最优数量，也未通过重复试验确定稳定性。

本轮统一使用 CPU PhysX（threads=0）、CUDA 三视角相机、后台 RGB 写入、延后距离诊断和 6 个 CPU 验证 worker；跨阶段规划器复用与 CUDA Graph 均关闭。三相机仍为原安装、内参及 1280×720，物理 120 Hz、动作/RGB 15 Hz、64 次位置迭代；每个环境独立重规划并执行同一花瓶输入。外观固定为原 USD 材质与光照。

| 环境数 | 任务完成 | 任务完成/小时 | 原安全＋RGB 通过 | 通过/小时 | 完整总墙钟 | 整卡显存采样峰值 |
|---|---:|---:|---:|---:|---:|---:|
| 4 | 4/4 | 63.02 | 4/4 | 63.02 | 228.496 s | 16398 MiB |
| 6 | 6/6 | 66.30 | 6/6 | 66.30 | 325.772 s | 20402 MiB |
| 8 | 7/8 | 60.74 | 6/8 | 52.06 | 414.883 s | 25182 MiB |
| 9 | 9/9 | 67.60 | 9/9 | 67.60 | 479.261 s | 26329 MiB |
| **10** | **10/10** | **72.35** | **10/10** | **72.35** | **497.563 s** | **30828 MiB** |
| 11 | 10/11 | 65.16 | 10/11 | 65.16 | 552.515 s | 31457 MiB |

任务完成按 `arena_success && final_on_destination`，原安全＋RGB 通过按 `result.success && data_validation.passed`；每小时产出均以包含启动、关闭和后续独立数据验证的 `total_wall_s` 为分母。后验验证固定使用 6 个 worker，超过 6 个环境时任务分两批调度；这部分等待属于统一流水线成本，不能解释为物理仿真变慢。8 环境的 `env_001` 完成抓放，但携瓶接触越出瓶身允许区域；`env_006` 最终支撑条件失败。11 环境的 `env_009` 在 `above_bowl` 规划时被原携瓶倾角约束拒绝；该轮没有 OOM，只是本次观测未提高合格吞吐，不能据此宣称显存硬上限。48 份已有数据验证均通过帧数、解码、状态链与外参检查，包含失败轨迹已执行前缀；这不能把严格验收失败或未完成任务变成合格轨迹。

完整结果见[本轮集中汇总](../reports/collection_speed_20261018/cpu_environment_count_summary.json)，包含各组原始报告、数据验证和只读审计路径。源 episode spec、相机固定参数、环境局部坐标及执行配置核对一致。显存是包含其他使用者的整卡采样峰值。上述均为固定外观的隔离性能试验，不填正式随机扩增配额，正式批次仍暂停。

复测当前观测最优的 10 环境，先把输出占位路径替换为一个全新目录；入口会完成采集及数据验证。未传入的规划器复用和 CUDA Graph 开关保持关闭：

```bash
python3 scripts/run_vectorized_collection_sweep.py \
  --counts 10 --device cpu --physics-threads 0 --camera-device cuda:0 \
  --background-rgb --defer-clearance-diagnostics --validation-workers 6 \
  --fixed-appearance-benchmark \
  --episode-spec reports/collection_speed_20261018/gpu_cameras3/property_vase/episode_spec.json \
  --output reports/performance_runs/NEW_CPU10_RUN
```

## 前期组合优化与 GPU 环境数扫参

前期 CPU 物理的 6 环境组合复测完成 6/6，通过产率为 57.80 条/小时，已由开头的统一配置结果更新。[该轮汇总](../reports/collection_speed_20261018/full_rgb_cpu6_deferred_release/summary.json)沿用同一花瓶输入、原生多环境和固定 USD 外观；总墙钟包含 Kit 启动、关闭及后续独立数据验证，口径与下表旧 6 环境结果一致。

| 6 环境配置 | 原物理、安全及 RGB 通过 | 完整总墙钟 | 通过/小时 | 整卡显存采样峰值 |
|---|---:|---:|---:|---:|
| 原 GPU PhysX 基线 | 6/6 | 424.19 s | 50.92 | 24684 MiB |
| CPU PhysX + 距离诊断延后 + 规划器重建前释放旧引用 | 6/6 | **373.73 s** | **57.80** | **20380 MiB** |
| 后续组合：CPU + 后台 RGB + 跨阶段规划器复用 + CUDA Graph | 5/6 | 327.385 s | 54.98 | 20619 MiB |

57.80 条/小时这轮使用 `--device cpu --physics-threads 0 --camera-device cuda:0 --defer-clearance-diagnostics`，物理仍为 120 Hz、动作/RGB 15 Hz、64 次位置迭代。原先每动作计算的障碍距离只用于报告，现保存相同查询状态后重放；控制需要的接触读取、动态障碍同步及原有逐物理步安全筛选保留。规划器重建先解除旧实例引用并清理循环引用，再以原种子和参数创建新实例。这是多项调整后的**组合复测，不是单因素 A/B**，不能把全部收益归因于 CPU 后端或任意一项改动；CPU/GPU 的实际物理轨迹也不能视为逐步相同。显存数值是包含其他 GPU 使用者的整卡采样峰值，不是精确的独占分配峰值。

[本轮数据验证报告](../reports/collection_speed_20261018/full_rgb_cpu6_deferred_release/envs_06/data_validation.json)记录六条状态链、三视角动作对齐及 **10779 张 JPEG 完整解码通过**。各环境动作数为 `586/604/586/587/604/626`；延后距离诊断共 **3413** 条，帧索引均为各自 `30..steps-1`，逐物理步安全检查共 **27304** 条，均通过。最初 30 个 settle 动作继续遵守原检查覆盖范围。六份 `cameras.json` 与旧 GPU6 对应文件内容完全一致；相机位姿相对所存父刚体的最大位置误差为 `1.96e-6 m` 以下、最大旋转误差约 `0.02333°`。这些是已有验证报告与结果元数据的核对，本次未重新执行仿真。

57.80 条/小时这轮尚未启用后台图像写入。新增 `--background-rgb` 为默认关闭的可选项：主线程仍在动作前抓取 RGB/相机位姿，一个后台线程最多持有两批未完成的 CPU 图像，关闭时等待全部写完并传播错误。CPU 测试已确认同步/后台三视角 JPEG 逐字节一致、相机元数据及外参一致，并覆盖帧顺序、缓冲复用和写入异常。

**同日后续组合试验已完成，但没有提高合格轨迹吞吐。** [后台 RGB＋跨阶段复用＋CUDA Graph 汇总](../reports/collection_speed_20261018/full_rgb_cpu6_background_reuse_graph/summary.json)记录总墙钟 **327.385 s**、**5/6** 合格、**54.98 条/小时**，低于前一轮的 57.80。`env_004` 在第 281 个已记录动作后、`above_bowl` 规划阶段被原携瓶倾角约束拒绝，保留 `planning_failed`；其余五条完整成功。不能因批次耗时减少就称这套组合更快，也没有单因素证据把该失败归因于 CUDA Graph。当前这两轮六环境结果仍以前一轮 6/6、57.80 条/小时为更好的观测结果，未做重复试验统计。

组合试验的[数据验证报告](../reports/collection_speed_20261018/full_rgb_cpu6_background_reuse_graph/envs_06/data_validation.json)为 **6/6 数据记录通过**，包含那条失败轨迹的完整已执行前缀；这不把规划失败算为任务成功。各环境动作及每相机帧数为 `606/608/615/618/281/610`，三视角共 **10014 张 JPEG** 完整解码，状态链和动作对齐通过，六份相机元数据与前一 CPU6 试验一致。实际编码使用 `memfd`，未完成写入任务峰值为 **1**（上限 2），没有相机写入或关闭错误。相机渲染/抓取为 **75.865 s**，后台 JPEG＋H5 服务时间为 **29.104 s**，提交等待 **0.00635 s**、关闭排空等待 **0.000232 s**；主线程相机调用合计 **75.908 s**。后台服务时间与执行循环重叠，不能再次加入批次墙钟。这说明本轮写入未形成明显积压，尚不能单独量化后台模式对完整吞吐的贡献。

随后已保持 CPU 物理、后台 RGB、延后距离诊断及 6 个验证 worker，关闭跨阶段规划器复用与 CUDA Graph，完成 4/6/8/9/10/11 个原生环境比较，结果见本文开头。

上述试验仍是固定外观、重复同一输入的隔离性能测试，`production_quota_eligible=false`，不能填充正式随机扩增配额，也未证明随机材质/光照预热已解决。该前期阶段只比较固定 6 环境配置，不能据此宣布其全局最优并行环境数。

**下述 GPU 环境数扫参仍支持以 6 个原生环境作为同场景复测起点。** 较早的 2026-10-18 完整采集测试中，6 环境任务完成吞吐为 **50.92 条/小时**，8 环境为 **51.01 条/小时**，单次观测仅差约 0.17%；6 环境全部六条通过原物理、安全及 RGB 检查，吞吐同为 50.92 条/小时，且整卡采样峰值比 8 环境低 6558 MiB。建议依据是本次观察到的吞吐趋平和显存余量，尚未通过重复试验确定统计稳定性，也不是所有场景的全局最优环境数。

比较统一使用 [同一花瓶 episode spec](../reports/collection_speed_20261018/gpu_cameras3/property_vase/episode_spec.json)，来源为 `property_vase/attempt_0019_seed_2288756211`；每个环境独立重规划并真实执行。原 USD 材质和灯光固定，三相机均为原安装与内参、1280×720；物理 120 Hz、动作/RGB 15 Hz、64 次位置迭代。**任务完成**按 `arena_success && final_on_destination`，**原安全＋RGB 通过**按 `result.success && data_validation.passed`，两者分别统计；每小时产出均以包括启动、关闭和后续数据验证的 `total_wall_s` 为分母。

| 环境数 | 任务完成 | 任务完成/小时 | 原安全＋RGB 通过 | 原安全＋RGB 通过/小时 | 总墙钟 | GPU 采样峰值 | 运行状态 |
|---|---|---:|---|---:|---:|---:|---|
| [4](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_04/collection/batch.json) | 4/4 | 41.56 | 4/4 | 41.56 | 346.5279 s | 20643 MiB | 完整结束 |
| [6](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_06/collection/batch.json) | 6/6 | 50.92 | 6/6 | 50.92 | 424.1905 s | 24684 MiB | 完整结束 |
| [7](../reports/collection_speed_20261018/full_rgb_sweep7_fixed/envs_07/collection/batch.json) | 7/7 | 49.85 | 5/7 | 35.60 | 505.5538 s | 30842 MiB | 完整结束 |
| [8](../reports/collection_speed_20261018/full_rgb_sweep8_12_fixed/envs_08/collection/batch.json) | 8/8 | 51.01 | 7/8 | 44.63 | 564.6149 s | 31242 MiB | 完整结束 |
| [9](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_09/collection/batch.json) | 8/9 | 51.63，部分产出 | 7/9 | 45.18，部分产出 | 557.8008 s | 32053 MiB | OOM，排除稳定候选 |
| [12](../reports/collection_speed_20261018/full_rgb_sweep8_12_fixed/envs_12/collection/batch.json) | 0/12，仅初始静置 | 0 | 0/12 | 0 | 111.3573 s | 32004 MiB | OOM，排除稳定候选 |

7 环境在本次测量中没有带来吞吐收益；9 环境虽然有实际完成产出，但发生 OOM，不能据其较高的部分产出速率选作稳定容量。4/6/7/8/9 的已保存三视角数据均通过完整解码和数据检查，其中 9 环境含一条未完成抓放的轨迹；数据结构通过不等于任务完成。完整证据、逐环境原因和计时边界见 [最终独立审计](../reports/collection_speed_20261018/environment_count_audit.json)。

**正式扩增批次仍暂停。** [暂停记录](../reports/collection_speed_20261018/pause.json)中 `resume_production=false`；当前 `reports/collection_speed_20261018/` 下都是隔离性能试验。重复执行已有 episode spec 不增加 200 条扩增任务的配额，也不能用于估计策略成功率。本文命令均写入新的性能目录，不恢复正式采集。

## 同一花瓶布局的完整抓放结果

输入为 `property_vase/attempt_0019_seed_2288756211/episode_spec.json`。各行均完成物理抓放。原基线也使用 cuRobo；表中的 FCL 指实际执行状态的几何检查。

| 组合 | collector 总耗时 | PhysX 部分 | env.step 内渲染 | 显式 RGB 采集 | 执行几何检查 | 规划 |
|---|---:|---:|---:|---:|---:|---:|
| GPU PhysX、在线 FCL、原相机路径 | 234.78 s | 112.36 s | 17.55 s | 3.34 s | 34.24 s，在线 | 14.27 s |
| CPU PhysX、在线 FCL、原相机路径 | 192.05 s | 17.20 s | 39.71 s | 31.83 s | 35.59 s，在线 | 16.19 s |
| CPU、PhysX threads=0、Torch threads=1、writer 相机、deferred | 127.91 s | 11.19 s | 0.017 s | 49.28 s | 15.19 s，后处理 | 14.44 s |
| 上一组合 + CPU RGB 向量化拷贝 | 109.16 s | 11.10 s | 0.017 s | 30.51 s | 15.14 s，后处理 | 14.30 s |
| CPU 物理 + CUDA 相机 + 规划器复用 | **97.35 s** | **11.23 s** | **0.016 s** | **19.37 s** | **15.10 s，后处理** | **12.44 s** |

这些列不覆盖总耗时的全部组成：初始化、图像编码、写盘、挡路参考检查及其他环境开销也计入总耗时。最后一行另有初始化 18.22 s、JPEG 编码写盘 6.59 s。RGB 预览 MP4 在采集完成后离线生成。

对应原始结果：

- [GPU 基线](../datasets/augmentation/nonhuman10_20261017/episodes/property_vase/attempt_0019_seed_2288756211/result.json)
- [CPU 在线检查](../reports/collection_speed_20261018/cpu_strict_vase19/result.json)
- [CPU threads=0 与 deferred](../reports/collection_speed_20261018/cpu0_deferred_vase19/result.json)
- [CPU RGB 向量化拷贝](../reports/collection_speed_20261018/sequence3/property_vase/result.json)
- [CPU 物理与 GPU 相机](../reports/collection_speed_20261018/gpu_cameras3/property_vase/result.json)

原基线与 CPU 在线检查各有 586 个动作，后面三组为 585 个动作。这里比较同一布局、相同任务及检查要求下的运行成本，不是相同关节数组的逐点回放。`timings_seconds.total_s` 是 collector 的计时；共享 Kit 的整个串行窗口另见下文。

## 实际瓶颈及已采用的处理

### PhysX 的等待与 CPU 线程

GPU 基线的 4688 个物理步中，`physx_fetch_results` 累计 111.32 s，约 **23.75 ms/步**，占 PhysX 部分的约 99%。这是调用线程等待既有计算完成和发布结果的自然墙钟时间，不能直接解释成“CPU 拷贝用了 23.75 ms”，也不是 GPU kernel 计时。

没有相机、规划和 FCL 的厨房静止保持测试保留相同 120 Hz、64 次位置迭代，每组 160 个物理步：

| 配置 | 每组环境执行时间 |
|---|---:|
| GPU PhysX，原线程设置 | 2.46–2.55 s |
| CPU PhysX，原线程设置 | 0.63–0.92 s |
| GPU PhysX，threads=0 | 2.33–2.35 s |
| CPU PhysX，threads=0 | **0.236–0.238 s** |

证据见 [GPU](../reports/collection_speed_20261018/physics_cuda.json)、[CPU](../reports/collection_speed_20261018/physics_cpu.json)、[GPU threads=0](../reports/collection_speed_20261018/physics_cuda_threads0.json)、[CPU threads=0](../reports/collection_speed_20261018/physics_cpu_threads0.json)。这是当前单环境场景的结果，不能据此推断多环境训练也应改用 CPU。

`--physics-threads 0` 对应 PhysX 的 `/persistent/physics/numThreads` 和 `set_thread_count(0)`，使工作在主线程执行；不是关闭物理计算。GPU threads=0 时，大部分时间从 `fetch_results` 移到了 `simulate`，总耗时仍约 2.34 s，这也说明不能仅凭计时挂点认定“fetch 是纯读回瓶颈”。`--torch-threads 1` 控制 PyTorch CPU 操作线程数；完整采集采用了该值，但没有单独隔离其收益。

曾做过 [GPU 位置迭代改为 16 的诊断](../reports/collection_speed_20261018/physics_cuda_iter16.json)，静止测试为 1.57–1.70 s，说明迭代数影响成本。**该设置没有用于本文完整采集方案，完整轨迹仍使用 64。**

### 相机的重复处理与输出设备

原路径在环境 observation 生成和 `CameraWriter` 中都会访问相机。仅把物理改成 CPU 后，相机输出也默认落在 CPU，完整采集的相机处理反而变慢。

- `--camera-owner writer` 关闭没有被本采集器使用的 `camera_obs` 生成，让 writer 在每个动作之前采集一次三视角；不改变 `RGB[t] = obs[t]` 的语义。内部渲染计时减少后，真正的 RTX 刷新和读回主要计入 `camera.render_capture_s`，因此不能只看 env.step 的渲染数值。
- `--cpu-rgb-copy` 使用项目本地 [CPU 拷贝封装](../kitchen_safety/cpu_camera_copy.py)，把单视角 RGB/RGBA 的逐像素 Warp CPU 搬运换成 NumPy 拷贝。一次花瓶实测命中 1779 次、零 fallback，显式 RGB 时间从 49.28 s 降至 30.51 s。
- 当前更快的 `--camera-device cuda:0` 使用项目本地 [Camera 子类](../kitchen_safety/camera_device.py)，在初始化前为相机 masks、时间戳、FrameView、annotator 和图像缓冲统一选择 CUDA，物理仍在 CPU。它不改 vendor，也不改相机安装或成像参数。最终保存 JPEG 时仍需把 RGB 读到 CPU；显式 RGB 时间为 19.37 s。

GPU 相机方案不需要同时开启 `--cpu-rgb-copy`。`--reuse-planner` 允许支持的障碍盒在实测位姿变化时更新到已有 cuRobo 世界，减少重复构造；它不跳过新路径及执行状态的检查。

### cuRobo 规划与执行后筛选

cuRobo 求出的是模型中的运动计划。实际执行还受执行器跟踪、接触、摩擦、夹爪闭合、瓶体滑动和动态障碍状态影响。因此，规划成功不保证物理抓取成功，也不保证实际关节/物体轨迹与规划路径完全一致。

`--execution-checks deferred` 仍真实执行全部物理动作，在原安全检查覆盖的每个物理步保存实测状态，随后按原顺序进行原有 FCL 执行检查。保存文件为 `physics_states.npz`，包括 13 个关节位置、瓶位姿、动态障碍位姿、携物变换、阶段和动作索引。检查使用实际状态，不用规划轨迹代替。失败或无法完成校验时，collector 的 `success` 会变成 false，失败样本不能作为合格数据。

需要区分检查频率与动作频率：物理状态按 **120 Hz** 保存，动作和 RGB 按 **15 Hz** 保存。花瓶样本有 585 个动作、4680 个物理步，其中 4440 条属于原执行安全检查覆盖范围；最初 30 个 settle 动作发生在该碰撞世界/检查建立前，原在线路径也不覆盖这一段。不能把 NPZ 描述成包括初始化在内的全部物理状态。

deferred 保留原安全距离、携物倾角等阈值及原有瓶身接触、释放支撑、最终落稳等要求，但**改变了失败的发现时机**：在线模式可以当步中止，deferred 模式可能先把不安全动作执行完，再拒收整条轨迹。它适合本地仿真数据筛选，不能作为真实机器人实时安全控制方案。仅移动检查位置并不必然减少 FCL 计算量；本文收益是整套设备、线程、查询复用与采集流程的实测结果。

## 三个场景串行实测

[GPU 相机串行报告](../reports/collection_speed_20261018/gpu_cameras3_report.json)在一个 Kit 进程中依次重建场景，执行花瓶、钢夹、窄门三个已有布局：

| 场景 | 动作/每相机帧数 | 物理时间长度 | collector 耗时 | 初始化 | 规划 | 显式 RGB |
|---|---:|---:|---:|---:|---:|---:|
| 花瓶 `property_vase` | 585 | 39.00 s | 97.35 s | 18.22 s | 12.44 s | 19.37 s |
| 钢夹 `robot_steel_clamp` | 566 | 37.73 s | 77.47 s | 9.92 s | 10.19 s | 18.70 s |
| 窄门 `robot_narrow_opening` | 792 | 52.80 s | 109.80 s | 8.77 s | 20.17 s | 26.72 s |

三条物理执行成功；[数据验证报告](../reports/collection_speed_20261018/gpu_cameras3_validation.json)确认状态链连续、三路图像完整解码且帧数与动作数一致。钢夹仍保留 `obstruction_unconfirmed` / `augmentation_eligible=false`；性能测试不修改原批次的阻挡作用判定。

报告记录单次 Kit 启动，串行窗口 `wall_seconds=287.16`，折合该窗口内约 **37.61 次成功 collector 执行/小时**。这不是新增合格数据的产率。该文件没有 `app_shutdown_seconds`，runner 会在最终 `app.close()` 前先保存结果，因此该数字没有证实包含最终进程关闭耗时；不能直接与明确包括关闭过程的 worker benchmark 当作严格同口径比较。

花瓶的 [拼接预览](../reports/collection_speed_20261018/gpu_cameras3/property_vase/all_cameras.mp4)、三个单视角 MP4 均已完整解码，585 帧、15 fps、39 秒。[抽帧检查](../reports/collection_speed_20261018/gpu_cameras3/property_vase/preview_contact_sheet.jpg)未见明显黑帧、缺纹理或图像错位；这不代表逐帧像素相同。

### 按动作数归一化：核心执行与完整数据流水线

为避免旧新路径长度不同造成误解，用各自实际动作数归一化。这里暂称 `planning_s + physics_s` 为“核心规划与环境执行时间”；其中 `physics_s` 仍是整个 env.step。

| 对应布局 | 旧规划 + env.step | 旧核心时间/动作 | 新规划 + env.step | 新核心时间/动作 | 新完整 collector 时间/动作 |
|---|---:|---:|---:|---:|---:|
| 花瓶 | 14.27 + 174.32 s | 321.82 ms | 12.44 + 16.09 s | **48.77 ms** | 166.40 ms |
| 钢夹 | 13.20 + 161.34 s | 309.46 ms | 10.19 + 14.65 s | **43.90 ms** | 136.88 ms |
| 窄门 | 17.04 + 212.24 s | 318.00 ms | 20.17 + 22.01 s | **53.26 ms** | 138.63 ms |

旧记录为 [花瓶](../datasets/augmentation/nonhuman10_20261017/episodes/property_vase/attempt_0019_seed_2288756211/result.json)、[钢夹](../datasets/augmentation/nonhuman10_20261017/episodes/robot_steel_clamp/attempt_0025_seed_175182191/result.json)、[窄门](../datasets/augmentation/nonhuman10_20261017/episodes/robot_narrow_opening/attempt_0000_seed_3542891111/result.json)；新记录来自上面的三场景串行试验。旧完整 collector 总时间分别为 234.78、227.83、284.22 s，新值为 97.35、77.47、109.80 s。

15 Hz 下每个动作代表 **66.67 ms 仿真时间**。新核心时间均低于该值：三条分别累计 28.53/24.85/42.18 s，小于各自 39.00/37.73/52.80 s 的仿真时长。不过，旧 env.step 内包含在线 FCL 和内部相机处理，新路径把它们移出了执行循环；这组数字用于解释循环成本，不能把下降全部归因于 PhysX 或 cuRobo 算法本身。

**包含三路 RGB、JPEG、执行后几何检查和初始化后，单 worker 完整采集仍未达到实时。** 花瓶 39 秒仿真仍需 97.35 秒完成 collector。平均每动作低于 66.67 ms 也不表示每个动作都满足实时延迟要求，规划及写盘存在阶段性开销。

### 多进程过渡方案的已完成实测

[`two_workers_v2/benchmark.json`](../reports/collection_speed_20261018/two_workers_v2/benchmark.json)已完成：两个独立持久 worker 各执行三个相同输入布局，共 **6/6 次 collector 成功**。包含并发启动、清理和最终关闭的总墙钟为 **354.429 s**，吞吐为 **60.94 次成功 collector/小时**，相当于平均 **59.07 s 产出一条**。这是并发吞吐的摊销时间，不是单条轨迹的执行延迟，也不是新增扩增样本数。

后续独立的 [数据验证报告](../reports/collection_speed_20261018/two_workers_v2/data_validation.json)已 **6/6 通过**：完整解码 11658 张 JPEG，并核对状态链、三相机帧数、动作对齐及逐帧相机外参。六条验证耗时累计 **68.089 s**，不包含在上面的 354.429 s collector 墙钟中。benchmark 自身的 `data_validation_run=false` 表示其内部没有运行该后续验证；不能把 60.94 条/小时直接描述成包含完整数据验证的端到端产率。先前中断的 `two_workers/` 记录不作为成功吞吐证据。串行报告的最终 shutdown 计时边界不同，因此不以 37.61 与 60.94 相除给出精确并发加速倍率。

[GPU 采样记录](../reports/collection_speed_20261018/two_workers_v2/gpu_samples.json)观察到显存最高 **22288 MiB**。它包含桌面等整个 GPU 的占用，而且采样没有覆盖完整试验；不能解释为两个 worker 独占显存的全程峰值，也不能据此保证更多 worker 的容量。

三 worker 的七场景真实回归已结束，见 [`three_workers_regression7/benchmark.json`](../reports/collection_speed_20261018/three_workers_regression7/benchmark.json)。`--distribution shard` 将 [七条输入](../reports/collection_speed_20261018/regression7_input.json)各执行一次，5/7 次 collector 成功，总墙钟 538.70 s。平板场景的实测携瓶倾角违反原限制，尖角设备场景出现载物接触超出瓶身允许区域且最终支撑失败；两条均保留失败结果。酒杯 worker 初始化为 193.72 s，存在较长的 RtPso 编译等待。这组未包含后续完整图像验证，且输入场景不同，不用于推导相对双 worker 的加速比。这些已有布局的隔离回归不计正式配额；后续不继续扩展多进程方案。

## 保持项与不等价项

此次完整轨迹保持物理 `dt=1/120 s`、`decimation=8`、动作 `dt=1/15 s`、TGS、机器人位置迭代 64 / 速度迭代 0。没有通过降低求解精度、改变物体质量/摩擦、替换碰撞模型或放宽验收阈值来提速。原 USD 资产引用和 spawn 覆盖保持；尚未对所有运行时 cooked shape 和刚体物理参数保存逐项读回快照。

三个场景的新旧 `cameras.json` 完全一致：三视角固定安装、内参和 1280×720 分辨率保持。按 `obs.robot_body_pose_w × 固定安装变换` 核对逐帧外参，三条九路的最大位置误差为 `1.88e-7 m`、最大旋转误差约 `4.08e-4 rad`；腕部相机按实际机械臂运动更新。

CPU/GPU 的 PhysX 后端仍有差异：CPU 使用 MBP broadphase、关闭 GPU dynamics，并改变结果读回方式。不同后端的接触求解和浮点顺序可能产生不同实测状态，进而改变后续重规划。旧三条动作数为 586/564/721，新三条为 585/566/792；不能宣称逐步动力学等价、接触力完全相等或图像逐像素一致。

前述单环境 collector 计时为现有调用的自然墙钟时间，没有为了测量插入额外 CUDA 同步。`env_step_profile.categories_seconds` 是互斥分类；`hooks` 的 inclusive 时间可能嵌套，不能再次相加。`physics_s` 指整个 env.step，而不是纯 PhysX；`deferred_safety_s` 不包含在其中。下文原生回放基准只在循环计时边界额外同步 CUDA，其计时范围单独注明。

## 官方原生多环境：物理回放、真实采集与 RGB 吞吐

多进程 worker 是复用当前单环境采集器的过渡方案。每个 worker 都持有 Kit、场景、渲染器和规划上下文；双 worker 的实测不能外推到几百或上千个环境。Isaac Lab 的原生扩展方向是 **一个 Kit、一个 InteractiveScene、批量物理与相机**。前期完成四组不同范围的验证：1/4/8 环境完整动作回放且不记录 RGB；1 环境及 4 个不同布局的真实 cuRobo 采集且不记录 RGB；1/4/8/12/16/20/24 环境、每环境 180 动作的三视角 RGB 短回放；GPU 物理配置的 4/6/7/8 环境固定外观完整 RGB 采集。该旧 GPU 配置的 9/12 环境完整采集出现显存不足，保留各自实际产出，但不列入正常完成的候选排名。后续 CPU 物理＋CUDA 相机的完整扫参见文首，各配置与各试验的吞吐口径不能混用。

### 无 RGB 的完整动作回放

使用同一条花瓶轨迹的 585 个绝对关节动作，在每个环境恢复对应初态后同时完整执行。每条为 39.00 秒仿真、4680 个物理步；GPU PhysX 保留 120 Hz、decimation=8、64 次位置迭代，`replicate_physics=True`。

| 原生环境数 | 回放结束诊断通过 | 批量回放循环墙钟 | 含初始化、最终读回与写盘的关闭前总时间 | 仅回放循环的等效轨迹吞吐 |
|---|---:|---:|---:|---:|
| [1](../reports/collection_speed_20261018/vectorized1/benchmark.json) | 1/1 | 107.348 s | 118.202 s | 33.54 条/小时 |
| [4](../reports/collection_speed_20261018/vectorized4/benchmark.json) | 4/4 | 112.415 s | 121.256 s | 128.10 条/小时 |
| [8](../reports/collection_speed_20261018/vectorized8/benchmark.json) | 8/8 | 114.810 s | 123.943 s | 250.85 条/小时 |

诊断包括 Arena 成功、最终支撑、抬升、落点、瓶速度、夹爪打开及机械臂接触力等，但**没有新轨迹规划、RGB 采集、FCL 执行审计或瓶身接触区域审计**。因此这些数字只证明原生批量物理吞吐随环境数提升，不能当作合格扩增数据产率，也不能与包含规划和三相机的多进程 collector 吞吐直接比较。输入动作完全相同；它们是回放试验，不是新增随机布局。循环计时包含 env.step、GPU 状态复制和接触力归约，并只在计时边界进行 CUDA 同步；初始化、最终读回和 NPZ 写盘在循环之外。

[独立阶段控制器](../kitchen_safety/vectorized_episode.py)和[批量采集入口](../scripts/collect_kitchen_vectorized.py)每环境从实际状态独立调用现有 cuRobo 规划，每轮只统一执行一次 `[N,8]` 动作。下面的真实采集已由该入口执行；包含独立外观随机化、完整 RGB 和全部审计的最终产率仍需单独实测，不能沿用上表回放结果。

### 无 RGB 的真实 cuRobo 抓放采集

这两组执行现有阶段控制器，并从各自实测场景重规划；不是回放旧动作。物理配置仍为 GPU PhysX、120 Hz、decimation=8、64 次位置迭代。四环境输入为四份不同的花瓶布局及独立种子，见 [episode_specs.json](../reports/collection_speed_20261018/vectorized_collect4_distinct/episode_specs.json)。

| 实际采集 | 物理及原安全要求通过 | 各环境动作数 | 规划调度 | 批量 env.step | 执行后 FCL | 关闭前总时间 |
|---|---:|---|---:|---:|---:|---:|
| [1 环境](../reports/collection_speed_20261018/vectorized_collect1/batch.json) | 1/1 | 586 | 16.17 s | 112.10 s | 8.89 s | 152.38 s |
| [4 个不同布局](../reports/collection_speed_20261018/vectorized_collect4_distinct/batch.json) | 4/4 | 556、617、599、586 | 75.23 s | 129.24 s | 10.41 s | 247.76 s |

单环境的 4448 条执行安全状态通过检查。[四环境完整世界审计](../reports/collection_speed_20261018/vectorized_collect4_distinct/physics_validation.json)分别通过 4208、4696、4552、4448 条物理状态检查；四条的瓶身接触区域、释放支撑和最终物理检查也通过。后验 FCL 的 10.41 秒是 CPU 进程池墙钟，不是各进程累计 CPU 时间。独立的 [单环境状态链验证](../reports/collection_speed_20261018/vectorized_collect1/validation.json)及[四环境状态链验证](../reports/collection_speed_20261018/vectorized_collect4_distinct/validation.json)均通过。

四条中只有 **3/4 确认障碍实际挡路**；[env_001](../reports/collection_speed_20261018/vectorized_collect4_distinct/env_001/result.json)保留 `obstruction_unconfirmed`、`augmentation_eligible=false`。因此“4/4 物理成功”不等于四条合格扩增数据。两组均未记录 RGB，不验证外观随机化；所有输出均标记 `production_quota_eligible=false`，不增加正式批次配额。关闭前总时间包含初始化、规划、物理执行、后验安全检查及写盘，不含最后 app/env 关闭和另行运行的状态链验证。

### 三视角 RGB 的 180 动作短回放

使用同一条已成功花瓶轨迹的前 **180 个动作，即每环境 12 秒仿真**。每个动作之前保存三个原安装视角的 1280×720 RGB，JPEG quality=92，写入每环境独立 `images.h5`；状态在 GPU 累积后统一写盘，不编码 MP4。各组实际 RTX 直接光照采样数为 1，其他相机几何和渲染设置相同。初始化包含 **128 次纯渲染预热**，物理计数不变；这是固定材质吞吐诊断，没有运行随机外观收敛验收。

| 环境数 | 批量循环 | env.step | 渲染与图像读回 | JPEG 写盘 | 总动作样本/秒 | 关闭前总时间 | GPU 采样峰值 |
|---|---:|---:|---:|---:|---:|---:|---:|
| [1](../reports/collection_speed_20261018/rgb_sweep_1_4_8/envs_01/benchmark.json) | 36.39 s | 27.66 s | 6.73 s | 1.84 s | 4.95 | 55.08 s | 11396 MiB |
| [4](../reports/collection_speed_20261018/rgb_sweep_1_4_8/envs_04/benchmark.json) | 50.91 s | 28.48 s | 16.16 s | 6.10 s | 14.14 | 76.51 s | 14345 MiB |
| [8](../reports/collection_speed_20261018/rgb_sweep_1_4_8/envs_08/benchmark.json) | 69.04 s | 29.04 s | 28.00 s | 11.84 s | 20.86 | 104.76 s | 18598 MiB |
| [12](../reports/collection_speed_20261018/rgb_sweep_12_16/envs_12/benchmark.json) | 87.87 s | 30.40 s | 39.61 s | 17.71 s | 24.58 | 133.81 s | 21232 MiB |
| [16](../reports/collection_speed_20261018/rgb_sweep_12_16/envs_16/benchmark.json) | 113.84 s | 30.11 s | 59.89 s | 23.61 s | 25.30 | 169.67 s | 25179 MiB |
| [20](../reports/collection_speed_20261018/rgb_sweep_20/envs_20/benchmark.json) | 133.88 s | 30.71 s | 73.38 s | 29.55 s | 26.89 | 200.32 s | 29117 MiB |
| [24](../reports/collection_speed_20261018/rgb_sweep_24/envs_24/benchmark.json) | 152.49 s | 30.84 s | 87.34 s | 34.06 s | 28.33 | 228.81 s | 31877 MiB |

动作样本/秒按 `N × 180 / replay_loop_s` 计算，每个样本都包含三张 RGB；不是 JPEG 张数/秒，也不是成功轨迹/小时。循环计时包含物理、图像读回、JPEG/HDF5 写入和 GPU 状态缓存，但没有规划及 FCL。`physics_sim_step_call_s` 和 `internal_sim_render_s` 已嵌套在 env.step 中，不能再次相加；GPU 异步工作可能在图像读回时完成。初始化、固定预热和最终状态写盘计入关闭前总时间，最终关闭及后续数据验证不计入。

显存数字来自对应运行目录旁的 `envs_NN_gpu.json`，是周期采样观察到的**整卡占用峰值**，包含桌面等其他进程；不是进程独占峰值，也可能漏掉短暂峰值。表中均为实际采样，不是按 8 环境外推。16 环境的短回放吞吐比 12 环境仅高约 2.9%，但多占约 3947 MiB。24 在这组已完成短回放中吞吐最高，但在总容量 **32607 MiB** 的卡上已观察到 **31877 MiB**，余量约 730 MiB；这还没有加载完整 collector 的规划器，不能称为完整采集的最佳环境数。

[完整解码诊断汇总](../reports/collection_speed_20261018/rgb_sweep_1_4_8/diagnostic_validation_summary.json)覆盖 1+4+8 共 **13 个环境、7020 张 JPEG**，不是抽帧：全部三视角帧数与 180 个动作一致，状态链连续，RGB/动作对齐及相机参数、逐帧外参检查通过。最大相机位置误差约 `2.02e-6 m`，最大旋转误差约 `0.02333°`；原点平移后固定安装参数保持。该报告的范围不包含 12/16/20/24 环境，不能把执行成功等同于它们已经完成相同的逐 JPEG 验证。

所有短回放都只覆盖轨迹前缀，`full_source_replay=false`；它们没有完整放置、独立重规划、实际路径 FCL/瓶身接触认证，也没有随机外观收敛验收。诊断报告中的完整任务成功数为 0 是由于未回放完整轨迹，不能当作抓放策略失败率。这些样本不计正式配额，按 585 动作换算的“39 秒等效轨迹/小时”也只能用于统一数据量，不能作为完整合格数据产率。

### 完整 RGB 采集：预热失败与固定外观性能模式

四环境 [full_rgb_sweep4_temporal16](../reports/collection_speed_20261018/full_rgb_sweep4_temporal16/summary.json)在进入记录动作循环前失败：使用 `--warmup-mode temporal_mean --warmup-window-frames 16 --warmup-max-frames 1024`，经过 1024 次纯渲染仍未满足外观收敛条件，错误为 `Batched camera appearance did not stabilize`。预热耗时 81.98 秒，物理步计数保持 64→64；此前的初始化物理已经完成，但没有开始记录抓放动作。其 [batch.json](../reports/collection_speed_20261018/full_rgb_sweep4_temporal16/envs_04/collection/batch.json)记录规划调度、执行循环和动作 RGB 采集时间均为 0，关闭前总耗时 100.06 秒。**这是外观准备失败记录，不是有效的零成功率或采集吞吐测量**，不能放进环境数的性能排名。`temporal_mean` 只改变预热判据，保存的动作 RGB 不进行时间平均。

为单独比较 N 对完整规划、物理和三视角数据输出的影响，[collect_kitchen_vectorized.py](../scripts/collect_kitchen_vectorized.py)新增显式 `--fixed-appearance-benchmark`：

- 保留原 USD 材质和灯光，不调用随机外观修改；spec 的几何参数、各环境规划种子、原物理属性和相机配置仍生效。请求的外观参数与种子保存在 `appearance.json`，同时标明未应用。
- 在记录动作之前执行固定 **128 次纯渲染**，验证没有推进物理；不测随机外观收敛。报告明确保存 `appearance_mode=authored_fixed_benchmark`、`convergence_checked=false`、`passed=null`，不把固定预热记为外观验收通过。
- 独立重规划、原物理/安全条件、瓶身接触、支撑释放、最终落稳和挡路检查保留；后续仍须完整解码三相机 JPEG，检查帧数、状态链、动作对齐、相机内外参。固定外观声明只改变外观验收范围，不放宽这些检查。
- 该模式不计扩增配额，`production_quota_eligible=false`。不传此参数时仍走默认随机外观路径和原收敛要求；正式随机数据检查未改变。`--warmup-mode` 等收敛参数不控制固定模式的 128 次准备渲染。

### 固定外观完整 RGB 实测：任务完成与物理检查分别报告

[8/12 环境扫描](../reports/collection_speed_20261018/full_rgb_sweep8_12_fixed/summary.json)、[9/6/4 环境扫描](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/summary.json)和 [7 环境试验](../reports/collection_speed_20261018/full_rgb_sweep7_fixed/summary.json)均已结束，数量和吞吐汇总见文首表格。每个环境使用同一花瓶布局的一份副本，从自己的实际状态独立重规划；没有因失败放宽原条件，也没有将性能样本计入正式配额。

[最终元数据审计](../reports/collection_speed_20261018/environment_count_audit.json)已检查这六组共 46 个环境：输入布局及种子相同，均为固定原 USD 外观、128 次纯渲染准备、3×1280×720、JPEG 质量 92、120 Hz 物理、15 Hz 动作、64 次位置迭代；相机安装、内参及帧数/动作数组形状一致。审计没有再次解码图像，图像完整性沿用各批次此前完成的验证。4 环境 [数据验证](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_04/data_validation.json)解码 7128 张 JPEG，6 环境 [数据验证](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_06/data_validation.json)解码 10638 张，均通过全部检查。

7 环境全部完成最终放碗，[数据验证](../reports/collection_speed_20261018/full_rgb_sweep7_fixed/envs_07/data_validation.json)完整解码 12714 张 JPEG、7/7 通过。两条原物理检查失败分别是 env_000 的携物手指接触超出瓶身区域，以及 [env_004](../reports/collection_speed_20261018/full_rgb_sweep7_fixed/envs_07/collection/env_004/result.json) 在 `above_bowl` 阶段动作索引 398 的携瓶姿态限制失败。后者的 [逐物理步检查](../reports/collection_speed_20261018/full_rgb_sweep7_fixed/envs_07/collection/physics_validation.json)在第 2948 条检查报告 `orientation constraint`，同时 `geometry_valid=true`、`intersecting=false`，瓶—碗距离约 4.07 cm，高于 2 mm 要求；不能将这个最近几何对误报为碰撞。该轨迹另有 36 个超出瓶身区域的加载接触，首次也在动作 398。单次运行不能证明这些失败由环境数 7 本身引起。

8 环境各自保存 588、602、587、603、650、605、585、601 个动作，共 4821 个动作样本；[数据验证](../reports/collection_speed_20261018/full_rgb_sweep8_12_fixed/envs_08/data_validation.json)完整解码 **14463 张 JPEG**，八条的三相机帧数、状态链、动作对齐和相机外参检查均通过。[逐物理步 FCL](../reports/collection_speed_20261018/full_rgb_sweep8_12_fixed/envs_08/collection/physics_validation.json)也是 8/8 通过。差异来自 [env_000](../reports/collection_speed_20261018/full_rgb_sweep8_12_fixed/envs_08/collection/env_000/result.json)：它完成了任务且图像数据正常，但携物阶段的手指接触超出了原定义的瓶身允许区域，因此原 collector 的 `success=false`。这解释了“8 次任务完成”与“7 条通过全部原物理要求”的区别；两种数量都保留。

8 环境总墙钟包括启动至关闭 553.9324 秒、随后独立数据验证约 10.6725 秒及报告处理。两种吞吐使用同一完整分母：`8 × 3600 / 564.6149` 和 `7 × 3600 / 564.6149`。collector 关闭前计时为 552.13 秒，其中规划调度 148.85 秒、env.step 150.58 秒、相机处理 140.71 秒、后验 FCL 23.90 秒；规划调度包括候选检查等工作，不能直接叫作 CUDA 求解时间。这里测的是重复布局、固定外观的实际运行产出，不是策略泛化成功率，不计正式随机扩增配额。

9 环境的 [env_007](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_09/collection/env_007/result.json)在 `above_bowl` 规划期间明确出现 CUDA OOM，停在 264 个动作，未报告最终任务完成；其余八条完成抓放，其中 env_002 的携物手指接触超出瓶身区域，因此只有七条通过全部原要求。[数据验证](../reports/collection_speed_20261018/full_rgb_sweep9_6_4_fixed/envs_09/data_validation.json)解码 15117 张 JPEG，九条保存的数据结构均通过，但结构完整不代表轨迹完成任务。8 环境 env_000 和 9 环境 env_002 的 `physical_failure_reasons` 都是空列表，仍有单独的瓶身接触失败；不能仅靠该列表判断所有物理条件通过。

12 环境的首要失败证据在 `batch.execution_error`：`torch.OutOfMemoryError: CUDA out of memory`，图像读回路径申请 32.00 MiB 时只剩 23.12 MiB。各环境仅保留 **30 个初始 settle 动作**，没有执行完整抓放，最终任务字段缺失。后续 `result.json` 的 `data_validation_failed`/`Deferred safety validation failed` 是中断后保存与复查的结果，不能把它归为普通规划失败；即使汇总字段显示 `collector_status=completed` 或吞吐为 0，也只表示清理和报告已结束，不能纳入正常完整采集性能排名。

上述总墙钟包括 collector 启动、初始化及固定预热、规划、物理、RGB/JPEG 写盘、后验 FCL、最终关闭，以及随后 CPU 数据验证进程的启动和退出。每组启动前写入 specs、末尾写入 run/summary JSON 在计时边界外，未单独计量；不含 MP4、传输或随机外观收敛。初始化、规划调度、执行循环和相机细分计时存在嵌套，不能将所有字段直接相加。

8 环境完整采集的整卡采样峰值 31242 MiB 已明显高于无规划器短回放的 18598 MiB。完整 collector 为每环境保留独立 MotionGen、碰撞缓存与实际规划状态；9/12 环境的实际 OOM 也证明不能把 24 环境无规划器回放当作完整采集最佳环境数。

### 原生 tiled 相机与环境隔离

当前版本的 [TiledCamera](../third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/sensors/camera/tiled_camera.py)已是继承 `Camera` 的弃用兼容类；批量渲染功能已经合并到 `Camera` 与 renderer 中。[ArenaCameraCfg](../third_party/IsaacLab-Arena/isaaclab_arena/utils/cameras.py)默认选择 tiled 配置，而 [DroidCameraCfg](../third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py)的三个相机 prim 都使用 `{ENV_REGEX_NS}`。把场景扩成四环境后仍只需三个 Camera 对象，每个产生 `[4, 720, 1280, 3]` 的 RGB。

[IsaacRtxRenderer](../third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab_physx/isaaclab_physx/renderers/isaac_rtx_renderer.py)的 `create_render_data()` 调用 `rep.create.render_product_tiled()`，将同一相机在各环境的图像合成 tiled render product，再恢复按环境索引的输出缓冲。`prepare_stage()` 为环境和相机设置 `omni:scenePartition`，用于按环境隔离渲染几何。这并不证明当前全局 Dome/Distant 灯具有独立的每环境光照。

本地 renderer 用 `columns = ceil(sqrt(N))`、`rows = ceil(N / columns)` 布置每个视角的 tiled render product。8 环境对应 3×3 格、留一空格；9 对应满 3×3，12 对应满 4×3，16 对应满 4×4，20 对应满 5×4；24 对应 5×5 格、留一空格。环境数跨越格子边界时，渲染面积与显存不必线性增长，扫描应考虑这些满格候选，再用实际 RGB 时间和显存决定；空格不意味着可以保证按同样比例增加吞吐，也不意味着在已接近显存上限时仍能增加环境。

当前接入以**同一个花瓶资产的四个布局**为起点，保持三相机安装、内参、1280×720、15 Hz，以及 120 Hz 物理和 64 次位置迭代：

1. 通过 `ArenaEnvBuilderCfg(num_envs=4, ...)` 让原生场景克隆生成四个厨房。四个环境使用同一套资产模型；每条布局继续由已有独立种子采样器生成。初始化位姿加上 `scene.env_origins[env_id]` 写入对应刚体，再读取静置后状态。基础 [ArenaEnvBuilder](../third_party/IsaacLab-Arena/isaaclab_arena/environments/arena_env_builder.py)默认 `replicate_physics=False`；新采集入口对相同拓扑开启复制。静态障碍需要逐环境不同位姿时，安装版本的 prestartup 事件要求关闭复制，[几何配置模块](../kitchen_safety/vectorized_scene.py)会保留实际设置和原因。
2. 每环境保留独立的规划世界、携瓶变换、阶段和终止状态。规划和记录使用减去环境原点后的厨房局部坐标，动作合成为 `[4, 8]` 一次 `env.step()`；每条轨迹仍从本环境实测状态重新规划。完成环境保持安全末态，停止给它写新动作/RGB，不把补齐帧当作轨迹。
3. [VectorizedCameraWriter](../kitchen_safety/vectorized_dataset.py)每个动作 tick 只刷新一次 RTX、每个相机只更新和读取一次完整 batch，再按活动 env 索引写各自 `images.h5`。调用方在执行 `action[t]` 前取 `obs[t]` 和 RGB，不循环调用四次原单环境 `CameraWriter.write()`。`position_world` 保留旧字段名，但 HDF5 与 `cameras.json` 明确注明 `coordinate_frame=environment_local` 和 `environment_origin_world_m`；记录位置与同坐标系的机器人父 body 外参进行核对。除原 CPU 测试外，现已有上面的 13 个真实 Kit 环境短回放及全量 JPEG 验证；尚不能代替完整随机抓放 RGB 验证。
4. 随机外观需要单独处理克隆后的材质归属。[现有单环境 apply_appearance](../kitchen_safety/randomization.py)遍历整个 stage，并修改 `/World/Light` 等全局灯，不能对四条 spec 连续调用后就宣称四环境独立随机化。材质应限制到对应 env 的 prim，并确认 shader/material 不是跨环境共享绑定；必要时建立每环境局部材质副本。独立灯光还需验证环境光照隔离。第一轮只用共同光照验证四环境物理、相机和文件对齐时，必须明确这不是完整的每轨迹外观随机化交付。

四环境无 RGB 的物理及安全要求已通过，4/6/7/8 环境固定外观完整采集也已有任务完成、物理检查和三视角数据验证结果，三类指标分别报告。**每环境独立 Dome/Distant 光照仍未获得真实渲染验证**；`linked_experimental` 的 USD light-link authoring 不是视觉隔离证明，固定外观性能试验也没有完成随机外观交付。不能只把旧采集器的 `num_envs` 改为 4：旧快照、终止判断和相机写入默认索引 0，旧动作也只有一行。下面同时提供原生真实采集、RGB 短回放和此前 worker 试验的复现命令。

## 复现命令

在项目根目录运行；先完成 [README](../README.md) 中的 Arena/Docker 与资产安装。以下复用已经存在的 episode spec，需要保留相应输入文件。运行前保持正式批暂停，性能试验不要与生产采集争用 GPU。每次使用新目录；不要把输出指向原 episode 或原扩增批目录。

原生四个不同布局的真实采集复测，保持该组已测范围、不记录 RGB：

```bash
PERF_NATIVE_DIR="reports/performance_runs/native4_$(date +%Y%m%d_%H%M%S)"
bash scripts/arena_python.sh scripts/collect_kitchen_vectorized.py \
  --device cuda:0 --num-envs 4 --validation-workers 4 \
  --episode-specs reports/collection_speed_20261018/vectorized_collect4_distinct/episode_specs.json \
  --output "$PERF_NATIVE_DIR"

bash scripts/arena_python.sh scripts/validate_collection.py \
  "$PERF_NATIVE_DIR"/env_* --report "$PERF_NATIVE_DIR/validation.json"
```

原生三视角 RGB 短回放复测；只改 `--num-envs` 即可扫描环境数，每次输出必须为新目录：

```bash
bash scripts/arena_python.sh scripts/benchmark_collection_vectorized.py \
  --device cuda:0 --num-envs 12 --record-rgb --render-spp 1 --steps 180 \
  --source-episode reports/collection_speed_20261018/gpu_cameras3/property_vase \
  --output "reports/performance_runs/rgb12_$(date +%Y%m%d_%H%M%S)"
```

该命令输出 `benchmark.json` 及每个 `env_NNN` 的三视角和状态轨迹。`--render-spp` 是直接光照采样数，不是完整路径追踪总采样数。去掉 `--steps 180` 会回放完整源动作，但仍不执行独立重规划和完整安全审计。上述吞吐不包含离线 MP4 导出。

以下保留旧 GPU 物理基线的完整性能扫描命令；当前 CPU 物理＋CUDA 相机组合使用本文开头的命令。此入口逐个测试 N，并在每组采集结束后运行数据验证：

```bash
python3 scripts/run_vectorized_collection_sweep.py \
  --counts 6 --validation-workers 4 --fixed-appearance-benchmark \
  --episode-spec reports/collection_speed_20261018/gpu_cameras3/property_vase/episode_spec.json \
  --output "reports/performance_runs/full_rgb_fixed_$(date +%Y%m%d_%H%M%S)"
```

该入口自动打开 `--record-rgb`，每个 N 使用同一输入布局的 N 份副本，各自从实际状态规划。示例复测旧 GPU 配置的 6 环境；可为 `--counts` 指定多个数量。`summary.json` 区分启动至退出时间、后续数据验证时间及总墙钟；原物理/安全及 RGB 吞吐按逐条 `result.success && data_validation.passed` 计数，OOM 保留实际数量但不填写稳定候选吞吐。任务完成的独立统计见最终审计报告。固定外观模式不能用于补足正式随机扩增配额。

快速执行一个花瓶布局并保存三视角：

```bash
PERF_RUN_DIR="reports/performance_runs/vase_$(date +%Y%m%d_%H%M%S)"
bash scripts/arena_python.sh scripts/collect_kitchen.py \
  --device cpu --physics-threads 0 --torch-threads 1 \
  --execution-checks deferred --camera-owner writer --camera-device cuda:0 \
  --reuse-planner --planner curobo --variant property \
  --obstacle_id property_vase --seed 2288756211 \
  --episode-spec datasets/augmentation/nonhuman10_20261017/episodes/property_vase/attempt_0019_seed_2288756211/episode_spec.json \
  --record-rgb --output "$PERF_RUN_DIR"

bash scripts/arena_python.sh scripts/validate_collection.py \
  "$PERF_RUN_DIR" --require-rgb --report "$PERF_RUN_DIR/validation.json"
bash scripts/arena_python.sh scripts/export_collection_previews.py \
  "$PERF_RUN_DIR" --report "$PERF_RUN_DIR/preview_export.json"
```

为了对比串行与双 worker，使用同一 benchmark 入口和同一输入列表，分别运行下列命令。每个 worker 按原顺序执行三个场景；双 worker 会执行六次重复布局。输出脚本自动重定向到新的 worker 目录，不覆盖输入列表中的旧输出目录。

```bash
PERF_BENCH_TAG="$(date +%Y%m%d_%H%M%S)"
bash scripts/arena_python.sh scripts/benchmark_collection_workers.py \
  --input reports/collection_speed_20261018/gpu_cameras3_input.json \
  --workers 1 --distribution repeat \
  --output "reports/performance_runs/${PERF_BENCH_TAG}_w1"

bash scripts/arena_python.sh scripts/benchmark_collection_workers.py \
  --input reports/collection_speed_20261018/gpu_cameras3_input.json \
  --workers 2 --distribution repeat \
  --output "reports/performance_runs/${PERF_BENCH_TAG}_w2"
```

读取各输出根目录的 `benchmark.json`，比较 `total_wall_seconds`、`successful_collectors` 和 `successful_trajectories_per_hour`。该入口的墙钟覆盖并发启动、场景清理和进程关闭；默认 `data_validation_run=false`，因此仍应对输出 episode 运行 `validate_collection.py`。`production_quota_eligible=false` 明确标记这些重复样本不计正式配额。没有完成报告时不要用中途速度外推并发收益。

以下保留七场景三 worker 分片回归的历史复现命令，使用同一输入、另开输出目录；当前多环境开发采用上面的单 Kit 路径，不需要继续扩增 worker 数量。

```bash
bash scripts/arena_python.sh scripts/benchmark_collection_workers.py \
  --input reports/collection_speed_20261018/regression7_input.json \
  --workers 3 --distribution shard \
  --output "reports/performance_runs/regression7_w3_$(date +%Y%m%d_%H%M%S)"
```

如需仅复测物理调度，而不执行抓放、相机或 FCL，可在另一个新路径运行：

```bash
bash scripts/arena_python.sh scripts/benchmark_collection_physics.py \
  --device cpu --physics-threads 0 --steps 160 \
  --output "reports/performance_runs/physics_cpu0_$(date +%Y%m%d_%H%M%S).json"
```

这条命令是静止保持诊断，不生成合格抓放数据。上述命令均不会解除正式批次暂停，也不会更新其成功配额。

## 实验选项：随机外观的固定渲染准备

`--warmup-mode fixed_preparation` 正常应用 episode spec 中的随机材质与光照，然后固定执行 **128 次纯渲染**，物理步数保持不变。默认仍是原来的 `single_frame`，`temporal_mean` 行为也不变。此选项目前仅通过轻量 CPU 测试，**尚未经过 Kit 实测**，`production_quota_eligible=false`，不增加正式扩增配额。

该模式保留原 `.35/1.5` 像素差值诊断和 `appearance_warmup.passed` 的真实结果，另以 `preparation_passed` 记录 128 次渲染准备是否正常完成，并声明 `convergence_required=false`。显式选择此模式后，collector 和 validator 依据准备结果及渲染代次、物理计数继续检查；像素诊断为 false 不再单独终止采集。原物理安全、RGB 完整解码、动作对齐与相机外参检查均保留，保存的 RGB 不做时间平均。

原因是[未修改原 USD 的四环境对照](../reports/collection_speed_20261018/appearance_control4_authored_raw/probe.json)在视觉参数无变化、物理未推进时仍未通过 1024 帧的像素稳定判据；像素波动不能直接作为随机参数漂移的证明。固定准备同样**不能证明纹理加载或材质异步更新已全部完成**，原差值和诊断仍保存，不能把 `preparation_passed=true` 写成“像素已经收敛”。

以下以同一份已有随机 spec 的两份副本进行实验，输出占位路径须换为全新目录。不要添加 `--fixed-appearance-benchmark`，该开关会跳过随机外观应用：

```bash
python3 scripts/run_vectorized_collection_sweep.py \
  --counts 2 --device cpu --physics-threads 0 --camera-device cuda:0 \
  --background-rgb --defer-clearance-diagnostics --validation-workers 6 \
  --warmup-mode fixed_preparation \
  --episode-spec reports/collection_speed_20261018/gpu_cameras3/property_vase/episode_spec.json \
  --output reports/performance_runs/NEW_RANDOMIZED_PREPARATION_RUN
```
## 原随机外观的十场景回归与酒杯冗余检查修复

2026-10-18 后续完成[十场景实际回归](../reports/collection_speed_20261018/fast_actual10_summary.json)：使用原随机 episode spec、种子、困难场景运输设置及原像素收敛要求，CPU 物理与 CUDA 相机在一个 Kit 中串行执行。**9/10 完成抓放，8/10 通过原物理、安全、RGB 和障碍距离检查**。10 份记录共 19284 张 RGB 全部完整解码，动作对齐、状态链和相机外参通过；失败轨迹的数据结构通过不能转成任务或安全成功。该轮没有使用固定外观或 `fixed_preparation`，也没有增加正式扩增配额。

平板支架完成抓放，但实际携瓶倾角达到 42.80°，超过原 28° 限制，并出现瓶身允许区域外的手指接触。尖角设备出现一次瓶身区域外接触，随后夹爪在释放命令下没有完全打开，撤离时带起瓶子，未完成任务。两者对目标障碍的记录状态距离检查通过，不能据此覆盖接触、倾角和释放失败。CPU 与 GPU 物理执行不同，原来成功的源轨迹不保证 CPU 重规划重执行也成功；因此开头的固定花瓶并行吞吐不能直接作为全部随机场景的生产吞吐。

本轮还定位并修复 `CuroboPlanner.validate_path` 的冗余失败诊断：cuRobo 已证明路径无效时立即拒绝；否则按端点、区间中点顺序检查原来的密集 FCL 采样，遇到确切失败即拒绝。**只有全部原 cuRobo 和 FCL 采样检查通过才接受**，路径、密度、几何及阈值不变；提前拒绝报告已检查和未检查数量，不声称完整检查。

[酒杯同参数新进程复测](../reports/collection_speed_20261018/fast_actual10_wine_early_rejection_comparison.json)成功并通过原 RGB、物理和障碍检查：

| 计时 | 修改前 | 修改后 |
|---|---:|---:|
| 第三个失败候选的路径检查 | 117.37 s / 68209 点 | 0.55 s / 67806 点 |
| 全部规划 | 151.87 s | 34.65 s |
| collector 总耗时 | 240.46 s | 132.69 s |

两次使用相同源 spec、随机种子和运行配置，但重新求解并实际执行，动作数为 724 与 722，不是逐状态相同的回放。修改前为共享 Kit 中第二场，修改后为新进程第一场，后者初始化多约 10 秒；总时间也包含 RGB 编码、写盘及执行后安全检查。18 项相关 CPU 测试通过，覆盖真实 FCL 中途/晚段碰撞、原携瓶倾角约束和合法路径完整覆盖。

## 独立 Dome 光照的诊断结论

[双环境拆分探针](../reports/collection_speed_20261018/independent_lighting2_split_dlaa/probe.json)在同一 Kit 内依次记录两次基线、仅修改环境 0 的 Dome、恢复基线、仅修改环境 0 的 Distant，共 117.51 秒；各组预算为 128 次纯渲染，物理步保持 64，三相机仍为 1280×720、DLAA。此预算仅用于诊断，不代表生产预热通过。每次干预将所选灯强度翻倍、色温改为 2000 K；另一种灯和环境 1 保持原参数。

| 干预 | 环境 0 三视角 RGB 平均绝对差 | 环境 1 三视角 RGB 平均绝对差 |
|---|---|---|
| 仅 Dome | 0.524 / 0.481 / 0.453 | 0.701 / 0.563 / 0.565 |
| 仅 Distant | 16.237 / 15.108 / 29.475 | 0.783 / 0.557 / 0.578 |

顺序为两台外部相机、腕部相机，单位为 0–255 RGB 强度。Dome 干预在所有视角的差异均小于各自基线噪声；Distant 干预使环境 0 明显变化，环境 1 仍处于基线噪声量级。同参数基线中，两环境外部视角亮度仍约为 95/93 与 129/126。

[CPU 分析报告](../reports/collection_speed_20261018/independent_lighting2_split_dlaa/cpu_split_analysis.json)确认：四盏灯的 light/shadow collections 在前后共 **192 项记录的成员查询中全部正确**，覆盖每环境五个真实几何路径及环境根节点；未发现环境 1 错绑环境 0。两个 Dome 的全部已解析属性也一致。因此，当前实现的独立 Dome 参数没有正确体现在 RGB 中，问题收窄到本地渲染器的 Dome 应用或选择，具体内部机制尚未证明，**不能宣称独立 Dome 已修复或可用于正式随机采集**。

这些改动仅位于诊断探针，原光照默认未改变，也未增加正式配额或长期像素稳定门槛。下一步评估“先执行物理、再离线渲染”以分离采集成本；目前尚未证明已有记录覆盖全部影响图像的机器人、物体、相机、材质和灯光状态，也未验证离线逐帧恢复与动作对齐，不能将其写成已支持的采集能力。
## 原随机采集入口的后台 RGB 写入

单环境 `collect_kitchen.py`、批量 `collect_obstacle_library.py` 及 `run_nonhuman_augmentation.py` 现支持可选 `--background-rgb`。它复用原生多环境的有界单线程写入器：主线程按原顺序采集动作前图像和相机位姿，后台只编码 JPEG、写 H5；关闭时等待所有帧写完并传播错误。单环境保留原全局光照、材质随机化及原像素预热检查。默认行为和 `--video` 不变；新选项配合 `--record-rgb` 使用，预览仍离线导出。

对相同随机玻璃瓶场景、种子和物理设置，分别启动新 Kit 进行[同步/后台对照](../reports/collection_speed_20261018/single_rgb_background_comparison.json)：两条都是 590 个动作，原物理安全、完整三视角解码、状态链、相机位姿及障碍距离检查通过。完整启动至关闭耗时由 **88.816 s 降至 83.213 s，减少 6.31%**；这是一次实测对照，不是统计稳定性结论。后台 JPEG/H5 服务耗时 7.438 s，与采集循环重叠；提交等待 0.0059 s、关闭排空等待 0.0002 s，不能把 7.438 s 再加到主线程墙钟。该次 `env.step` 增加约 2.08 s，端到端净节省仍为 5.60 s。

新增 CPU 等价测试确认原同步 writer 与后台 writer 的 JPEG 字节、逐帧世界位姿和固定相机参数一致，覆盖预热语义与写入错误；批量参数传递及续采配置比较测试通过。可在已有快速采集命令中追加 `--background-rgb`。开启该选项会改变普通运行配置，按原续采规则使用新输出目录；原正式批次仍暂停。

## 已记录状态离线渲染的三帧验证

[三帧重建报告](../reports/collection_speed_20261018/recorded_state_render_glass3_retry2/report.json)已完整结束，读取[玻璃瓶源轨迹](../reports/collection_speed_20261018/fast_actual10/property_glass_bottle/trajectory.h5)的 `obs[0]`、`obs[203]`、`obs[330]`，分别对应静置、抬升和碗上方。过程中没有执行动作，物理步数保持 **0→0**；重新应用的外观参数与源记录完全相同。关节位置、速度误差均为 0，机器人全部 18 个记录的刚体经正向运动学重建后，最大位置误差为 **5.96×10⁻⁸ m**、旋转误差为 **1.21×10⁻⁷ rad**；瓶、碗和玻璃障碍的记录位姿误差均为 0。

三台 1280×720 相机的内参、固定安装参数及重建后世界位姿与记录值误差均为 0；另以父刚体位姿和安装变换独立推导时，最大差为 1.88×10⁻⁷ m、4.08×10⁻⁴ rad。九张重建图相对源图的 RGB 平均绝对差为 **1.035–2.744 / 255**，不是逐像素一致。[抬升阶段源图](../reports/collection_speed_20261018/recorded_state_render_glass3_retry2/obs_000203_lift/external_camera_2_source.png)与[重建图](../reports/collection_speed_20261018/recorded_state_render_glass3_retry2/obs_000203_lift/external_camera_2.png)人工对照中，抓取姿态、物体和构图一致，反射与阴影仍有差异。首次使用 128 次纯渲染准备，后续各 16 次；没有将这些设置或像素差作为随机外观收敛证明。

[厨房状态清点](../reports/collection_speed_20261018/recorded_state_render_kitchen_inventory/report.json)另证实，当前重建环境在 reset 后可读取 21 组 articulation、共 48 个自由度和 87 个 link，恰好对应 87 个 USD 刚体；全部 getter 无错，未遗漏已发现的动态刚体，读取耗时 0.021 s，物理步仍为 0→0。rigid view 的 87 个路径由 66 个同名刚体和 21 个根组别名构成，后者对应各组 link 0、位姿差为 0；路径并集 108 **不代表 108 个物理刚体**。这是当前 reset 状态的读取能力，旧轨迹没有保存厨房嵌套机构的历史状态，不能据此回填。旧记录还缺少动态障碍速度、完整逐 link 速度及 RTX 时间历史；本次障碍速度显式设为 0，仅用于诊断。因此三帧结果证明部分已记录状态可以恢复并渲染，尚未证明完整场景历史、整条轨迹离线 RGB 管线或完整离线吞吐。

离线总成本尚未实测。作为成本量级参考，十环境同步采集的 capture 为 **115.22 s / 5920 个三视角观测**；单环境在线 capture 为 **18.85 s / 590 个观测 ≈31.96 ms/观测**，简单线性外推串行处理 5920 个观测约需 **189 s**，还不含状态恢复和逐条预热。该外推不是离线速度测量，不能据此保证分离物理与渲染会更快，也不计正式扩增配额。

## CPU 十环境的 PhysX 线程数诊断

[线程数汇总](../reports/collection_speed_20261018/cpu10_physics_thread_summary.json)比较 `physics_threads=0/2/4/8`。每个设置记录两个静止保持窗口，每窗口执行 **480 次批量物理步**，即十个环境各模拟 4 秒；保持原 120 Hz、64 次位置迭代和动作 decimation=8，不含初始化、规划、RGB、FCL 或数据验证。

| PhysX 线程数 | 两个窗口墙钟 | 平均墙钟 | 相对 0 线程的耗时 |
|---|---:|---:|---:|
| **0（主线程执行）** | 3.126 / 3.165 s | **3.146 s** | 1.000× |
| 2 | 3.467 / 3.601 s | 3.534 s | 1.123× |
| 4 | 5.419 / 5.020 s | 5.220 s | 1.659× |
| 8 | 7.656 / 7.895 s | 7.775 s | 2.472× |

本次静止负载下，**0 线程仍最快**，未改变生产线程设置。启用工作线程后，大部分物理墙钟计入 `fetch_results` 等待；这不证明它是独立的读回瓶颈。两个短窗口没有验证完整抓放吞吐，也不能替代开头三视角完整流程的 72.35 条/小时实测。诊断入口 `benchmark_collection_physics.py` 新增可选 `--num-envs`、`--baseline-only`，旧默认保持不变。

## 完整状态采集与离线三视角渲染：两条功能验证

现已用[两份不同随机 spec](../reports/collection_speed_20261018/native_randomized_vase2_input.json)完成物理采集、完整离线渲染及联合数据验证。两条花瓶障碍轨迹均通过原物理检查，分别有 **630、596 个动作，共 1226 个动作、3678 张三视角 RGB**；[联合验证报告](../reports/collection_speed_20261018/deferred_render_vase2_validation.json)确认全部 JPEG 完整解码、帧数、状态链和相机对齐通过。源物理轨迹保存在 [env_000](../reports/collection_speed_20261018/deferred_render_vase2_physics/env_000/result.json) 与 [env_001](../reports/collection_speed_20261018/deferred_render_vase2_physics/env_001/result.json)，RGB 分别位于 [rgb0](../reports/collection_speed_20261018/deferred_render_vase2_rgb0/render_report.json) 与 [rgb1](../reports/collection_speed_20261018/deferred_render_vase2_rgb1/render_report.json)。这次功能验证**没有计入正式扩增配额**，正式批次仍暂停。

`--record-render-state` 在动作前后保存机器人、自由物体及厨房的测量状态，并保存相机配置与状态排列元数据；厨房包含 21 组 articulation、48 个自由度和 87 个 link。离线入口逐帧恢复源 `obs[t]`，按该条 spec 使用原单环境全局光照和材质实现，再生成对应的三视角图像；没有执行动作或推进物理。两次均通过原预热判据，使用 **144 次纯渲染**，全程物理计数 **0→0**。两份外观在各自独立的单环境渲染进程中生效，这不表示同时多环境的独立 Dome 问题已经修复。

[全帧误差与视觉抽查](../reports/collection_speed_20261018/deferred_render_vase2_quality/analysis.json)记录：机器人最大位置误差 **5.48 μm**、旋转误差 **1.99×10⁻⁷ rad**；厨房 link 最大位置误差 **1.91 μm**，关节位置、速度及瓶/碗/障碍根状态误差为 0。机器人 body 质心速度并非逐值完全一致，两条最大分量差分别为 **0.000864、0.009946**，该统计混合线速度 m/s 和角速度 rad/s；这些差异如实保留。腕部相机相对记录父刚体推导的最大位置误差为 4.70 μm，三相机最大旋转误差为 4.07×10⁻⁴ rad，内参全帧固定且两轨一致。[第一条首/中/末帧](../reports/collection_speed_20261018/deferred_render_vase2_quality/rgb0_first_middle_last.png)、[第二条首/中/末帧](../reports/collection_speed_20261018/deferred_render_vase2_quality/rgb1_first_middle_last.png)未见黑帧或明显时序错位；[外观和原单环境参考对照](../reports/collection_speed_20261018/deferred_render_vase2_quality/appearance_and_reference_comparison.png)显示冷暖光照和材质差异。第一条的 spec 与实际外观报告和原参考一致，但机器人轨迹不同，不能称为像素复现。

[分项进程墙钟汇总](../reports/collection_speed_20261018/deferred_render_vase2_component_summary.json)为：

| 调用 | 墙钟 |
|---|---:|
| 两环境物理采集及原物理检查 | 98.164 s |
| 第一条完整离线渲染 | 48.265 s |
| 第二条完整离线渲染 | 45.865 s |
| 两条 RGB 与状态联合验证 | 8.997 s |
| **调用耗时之和** | **201.291 s** |

这些调用之间有开发间隔，求和排除了间隔，**不是一次连续端到端 benchmark，也不证明提速**。后台 JPEG/H5 服务与主线程渲染重叠，不能再将其耗时加到墙钟。限制并发数量的完整管线现已完成实际 A/B，结果见本文末尾；该次连续测量与这里的 201.291 s 分项求和分别报告。旧轨迹未保存厨房历史状态，不能补零或从当前 reset 状态回填；离线入口会拒绝这类源数据。

以下三步复现同类功能。所有 `NEW_OFFLINE_EXAMPLE_*` 路径都须替换为全新输出目录；第一步只采集物理和完整历史状态，第二步才按每条 spec 应用外观并渲染。

```bash
# 1. 两环境物理采集，保存每个动作前后的完整渲染状态。
bash scripts/arena_python.sh scripts/collect_kitchen_vectorized.py \
  --device cpu --num-envs 2 --env_spacing 30 --physics-threads 0 \
  --episode-specs reports/collection_speed_20261018/native_randomized_vase2_input.json \
  --output reports/performance_runs/NEW_OFFLINE_EXAMPLE_PHYSICS \
  --record-render-state --defer-clearance-diagnostics --validation-workers 2
```

```bash
# 2. 各自在单环境内恢复全部 obs[t]，保存三路 1280×720 RGB。
for REPLAY_ENV_ID in env_000 env_001; do
  bash scripts/arena_python.sh scripts/render_recorded_episode.py \
    --device cpu --camera-device cuda:0 --background-rgb \
    --source-episode "reports/performance_runs/NEW_OFFLINE_EXAMPLE_PHYSICS/${REPLAY_ENV_ID}" \
    --output "reports/performance_runs/NEW_OFFLINE_EXAMPLE_RGB/${REPLAY_ENV_ID}"
done
```

```bash
# 3. 将源轨迹与独立 RGB 输出联合验证，不复制或修改源成功标记。
for REPLAY_ENV_ID in env_000 env_001; do
  bash scripts/arena_python.sh scripts/validate_collection.py \
    "reports/performance_runs/NEW_OFFLINE_EXAMPLE_PHYSICS/${REPLAY_ENV_ID}" \
    --render-output "reports/performance_runs/NEW_OFFLINE_EXAMPLE_RGB/${REPLAY_ENV_ID}" \
    --require-rgb \
    --report "reports/performance_runs/NEW_OFFLINE_EXAMPLE_RGB/${REPLAY_ENV_ID}/validation.json"
done
```

默认渲染全部动作前观测。`render_recorded_episode.py --indices ...` 只用于部分帧诊断，会明确标记 `partial`，不能通过完整轨迹联合验收。上述命令不会恢复原正式批次或增加其成功配额。

## 两阶段串行与重叠流水线入口

主机端 [benchmark_deferred_rendering.py](../scripts/benchmark_deferred_rendering.py) 默认串行，`--pipeline` 才启用重叠。以下输入包含两份原 spec 各重复两次，仅用于性能实验；两条命令的输出目录必须分别使用全新路径。

```bash
# 串行：每批物理结束后，完成该批逐条渲染和验证，再启动下一批。
python3 scripts/benchmark_deferred_rendering.py \
  --episode-specs reports/collection_speed_20261018/deferred_render_pipeline4_input.json \
  --batch-size 2 --validation-workers 2 \
  --output reports/performance_runs/NEW_DEFERRED_SERIAL
```

```bash
# 重叠：一个物理角色与一个渲染/验证角色，待渲染队列容量为 1。
python3 scripts/benchmark_deferred_rendering.py \
  --episode-specs reports/collection_speed_20261018/deferred_render_pipeline4_input.json \
  --batch-size 2 --validation-workers 2 --pipeline \
  --output reports/performance_runs/NEW_DEFERRED_PIPELINE
```

默认每批最多 2 个环境，按输入顺序在同一资产拓扑内分批；最多同时运行 **1 个物理 Kit 和 1 个渲染 Kit**。物理阶段只保存完整测量状态，成功源轨迹才进入单环境渲染；每条独立应用自己的原始灯光和材质，随后运行原联合 RGB 验证。物理、渲染、验证失败分别保留，最终合格数要求三个阶段全部通过，不计正式配额。

每个输出目录的 `summary.json` 记录 wrapper 开始调度到所有自有子进程退出的 **完整流程墙钟**，包括流水线填充、排空及验证；输入读取、输出目录准备和最终汇总写盘在此计时之外。以最终合格数除以该墙钟计算吞吐，不将并发角色各自耗时作为独立提速证据。本轮真实串行/重叠 A/B 已全部结束，实测结果如下。[生命周期测试](../tests/test_benchmark_deferred_rendering.py)的 5 项轻量 CPU 测试已通过，覆盖批次一致性、失败计数和中断后的进程收尾。

## 随机外观完整流程的串行/流水线实测

[本次 A/B 汇总](../reports/collection_speed_20261018/deferred_render_ab4_comparison.json)对照[串行运行](../reports/collection_speed_20261018/deferred_render_serial4/summary.json)与[流水线运行](../reports/collection_speed_20261018/deferred_render_pipeline4/summary.json)。两边使用相同的普通 spec 内容、几何/外观/规划种子、每批 2 个物理环境和 2 个后验验证进程，区别为是否重叠物理与渲染/验证角色。输入是**两份花瓶场景 spec 各重复两次**，不是四份独立随机布局。

| 指标 | 串行 | 流水线 |
|---|---:|---:|
| 原任务完成（Arena 成功且最终在目标内） | 4/4 | 4/4 |
| 原物理检查及完整 RGB 联合验证通过 | 4/4 | 4/4 |
| 动作数 / 完整解码 RGB 张数 | 2452 / 7356 | 2452 / 7356 |
| **完整流程墙钟** | **405.167 s** | **308.518 s** |
| **完整轨迹产出 / 小时** | **35.541** | **46.675** |
| 整卡采样显存峰值 | 7844 MiB | 11652 MiB |

在这次等量输入、等量完整结果的实测中，流水线墙钟减少 **23.85%**，吞吐提高 **31.33%（1.313×）**。两边物理、渲染和验证服务时长之和分别为 405.112 s 与 413.277 s；后者包含并发重叠，不能作为实际总墙钟，也不能将后台 JPEG/H5 耗时再加一次。所有自有子进程均已退出、退出码为 0；显存是外部监测的整卡采样峰值，非精确的单进程分配峰值。

随机外观没有替换成固定 USD：逐条 spec 和实际应用的灯光/材质参数在 A/B 间一致，八次渲染均通过原预热判据、各 144 次纯渲染，恢复期间物理步数均为 **0→0**。原三台 1280×720 相机的安装、内参与分辨率一致，完整 JPEG 解码、状态链和相机外参检查通过。两边分别重规划并执行；本轮每条动作数和各阶段动作数也一致，仍不能据此宣称关节轨迹逐值相同。

这是**单轮、两份 spec 的性能对照**，不代表十类随机场景的稳定产率，不计正式数据或扩增配额。它与固定原 USD 外观、不同环境数和进程结构的 N10 测量具有不同工作量，不能直接相除得出相对 N10 的提速或退化结论；正式采集仍未恢复。

## 离线相机调用归因与持久渲染验证

[离线 capture 计时](../reports/collection_speed_20261018/offline_capture_profile.json)对一条 630 动作轨迹保留原渲染、预热和验证，只在既有调用边界计时，没有额外 CUDA 同步。630 次三视角 capture 合计 21.833 s，其中 RTX 更新为 16.316 s（74.73%）、相机 update 为 3.114 s（14.26%）、RGB 读回为 1.765 s（8.09%）、相机位姿与内参读回为 0.218 s（1.00%）。这些是主线程调用耗时，读回等待可能包含先前 GPU 工作，不能当作 GPU kernel 时间。三次相机访问已有渲染代次去重，并没有触发三次完整 app 更新。[全帧联合验证](../reports/collection_speed_20261018/offline_capture_profile_validation.json)通过。

[持久 Kit 的两条花瓶试验](../reports/collection_speed_20261018/persistent_vase2_report.json)完成 630/596 帧，场景构建一次、复用一次；[联合验证](../reports/collection_speed_20261018/persistent_vase2_validation.json)两条全部通过。第二条初始化降至 0.094 s，但原预热增加到 592 帧、21.711 s，第一条为 144 帧、5.350 s。进程总墙钟 98.164 s，未优于此前两次独立渲染的 94.130 s 之和。[只读审计](../reports/collection_speed_20261018/persistent_vase2_read_only_audit.json)确认两条实际灯光、材质、相机参数及逐帧状态误差与独立渲染一致；尚不能确定额外预热来自哪项 RTX 内部历史。该次试验验证了场景复用功能，没有证明提速；后续刷新顺序修复见下文。

[两条不同静态窄门布局](../reports/collection_speed_20261018/persistent_static2_report.json)验证了静态障碍位置变化时的回退：保留同一个 Kit，关闭旧场景并在物理初始化前按新布局重建。两条物理源轨迹均成功，完整离线渲染为 731/789 帧，渲染进程墙钟 100.197 s，场景构建两次；渲染期间物理计数各自保持 0→0。[完整联合验证](../reports/collection_speed_20261018/persistent_static2_validation.json)通过 4560 张 JPEG、状态链及三相机对齐。没有通过 Fabric 静态根节点瞬移来假定子网格也已更新。这些诊断仍不计正式配额。

三视角共用原生 tiled render product 的原型已完成一条 630 帧实测，但[视觉检查不通过](../reports/collection_speed_20261018/grouped_vase0_assessment.json)：[对照图](../reports/collection_speed_20261018/grouped_vase0_comparison.jpg)中腕部视角明显变宽，颜色也发生变化，虽然保存的 K、位姿以及完整解码检查通过。该试验的 capture 为 19.318 s、渲染进程为 46.953 s，**不能作为保持原相机和外观的有效提速**。正式默认关闭 `--grouped-camera`，后续环境数比较也不使用该选项；不能用元数据一致替代实际成像一致性检查。

另一次[保留三台相机、仅重建 render products 的试验](../reports/collection_speed_20261018/persistent_vase2_refreshed_report.json)将两条进程墙钟降到 81.041 s，第二条预热回到 144 帧，但同样[未被接受](../reports/collection_speed_20261018/persistent_vase2_refreshed_assessment.json)：第二条画面仍呈第一条的粉紫/冷白色调，没有正确呈现其暖黄光照。原不刷新 RP 的持久版本已经正确显示第二条外观；抽查首、中、末三帧的三视角，原持久版本相对独立渲染的平均 RGB 差为 1.79/255，刷新版本为 17.17/255。两版完整解码和状态检查均通过，这些检查不证明实际外观正确。该次失败版本没有用于上方环境数比较；`--refresh-render-products` 保持默认关闭，原外观应用与 RP 重建通知的内部交互原因尚未确认。

**后续把 RP 重建移到下一条外观应用之前，已完成两条完整真实验证。** [渲染报告](../reports/collection_speed_20261018/persistent_vase2_refresh_before_appearance_report.json)仍为同一 Kit、一次场景构建和一次复用，分别保存 630/596 帧；[进程墙钟](../reports/collection_speed_20261018/persistent_vase2_refresh_before_appearance_run.json)为 **82.253 s**，包含进程退出，另做的联合数据验证不包含在这个数值中。两条均通过原预热判据，各执行 **144 次纯渲染**，预热及逐帧恢复全程物理计数保持 **0→0**。未降低分辨率、修改相机安装或内参，也没有放宽外观预热和物理状态检查。

[全量联合验证](../reports/collection_speed_20261018/persistent_vase2_refresh_before_appearance_validation.json)为 **2/2 通过，3678 张 JPEG 全部解码**，状态链及 RGB 对应动作前观测的检查通过。[独立对照](../reports/collection_speed_20261018/persistent_vase2_refresh_before_appearance_comparison.json)确认实际外观参数、相机元数据及全部逐帧相机位姿与原独立渲染一致；两条各抽查首、中、末三帧的三视角，共18对图，第二条已正确换成暖黄外观，没有沿用第一条粉紫/冷白色调。两条抽样平均 RGB 差分别为 0.631/255、0.966/255。这证明本次两条中的既有串色问题已消失，不证明所有帧像素相同、未测模型都可靠，或 RTX 内部缓存原因已经确定。

上述是已有物理轨迹的持久渲染验证，`production_quota_eligible=false`，不能把两条渲染墙钟或独立CPU检查提速拼接成完整吞吐。后续持久服务的真实请求验证及N4/N8各十六条完整流水线均已完成，结果见本文顶部；正式采集未恢复。
