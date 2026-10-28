# G banana 独立基线入口

## 当前进度：attempt06 已完成搬运和下放，释放未通过

`datasets/replicator/g_shape_banana_bowl/base/attempt_06` 记录了 3965 个连续实际状态、496 个动作和六路各 496 帧视频。预抓取、接近、抬升、搬运、下放均已真实执行；释放第 4 个子步（3964）发生香蕉与原碗源三角片的首次相交，被原 strict 检查中止。最后动作只有 4/8 子步，因此仍是失败前缀，不能作为完整成功基线或安全变体交付。

本次运输修复只将目标高度改为当前实际抬升末态的香蕉 root Z，保留原碗 XY、实际香蕉姿态和关闭时的规划 attachment。相对旧 `bowl_root_z + 0.20 m` 规则，本次目标降低约 62.1704 mm；没有更改物体、物理、抓取区域或门槛。搬运期间相对关闭末态的最大附件漂移约 0.561 mm / 1.475°，抬升、搬运和下放均持续保持原区双指加载。attempt05 已验证的稳定抓取和本次真实运输分别有独立证据，不能混写成完整任务成功。

释放首触位于碗口近局部 +Y 一侧；原过滤支撑力为 1.21931 N，但香蕉速度仍为 0.18116 m/s，高于原 0.003 m/s，原成功谓词为 false。原任务允许 on 或 in，并未要求只接触碗底，因此不能仅凭碗口接触认定目标错误或稳定放置。下一步需要独立核对预期支撑接触及源网格/PhysX 表示差异，禁止用此次观测深度事后构造许可阈值。

独立相机、来源绑定、背景稳定性和实际关节自碰撞检查通过。自碰撞覆盖全部 3965 个真实 q，原成本为零；最大关节限位数值超出为 5.35e-8 rad。完整回合门禁仍正确拒绝不完整动作。原图蒙太奇最后一帧对应 3960，未覆盖 3964 首触及之后的稳定状态。汇总见 `reports/replicator_execution_20261023/g_banana/attempt06_diagnostic_bundle01.json`，SHA256 `da82aca0009dab9512a982739d9c234542d7360cba61e458ea8a9d74a6f0e957`。三个安全变体尚未开始。

本次命令（已运行；不得覆盖已有目录）：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/task_collect.py \
  --task-id kitchen_bench_replicator_g_shape_banana_bowl \
  --output datasets/replicator/g_shape_banana_bowl/base/attempt_06 \
  --device cpu --physics-threads 2 --torch-threads 2 --seed 42 --headless \
  --episode-length-s 180 --max-steps 2400 \
  --controller-profile configs/extensions/replicator/g_shape_banana_bowl_controller.json \
  --grasp-candidate-profile configs/extensions/replicator/g_shape_banana_bowl_halfcenter_candidate.json \
  --transport-candidate-profile configs/extensions/replicator/g_shape_banana_bowl_transport_height_candidate.json
```

## 历史：attempt04 及此前的新布局试验

最新真实采集 `datasets/replicator/g_shape_banana_bowl/base/attempt_04` **失败，未完成放置**。它记录了 2575 个连续物理状态、322 个动作和六个完整解码视频；接近及连续 10 个子步的双指中部加载通过，绑定实测抓持状态的抬升规划也通过了门禁。但抬升已执行 19 个动作、150 个子步后，香蕉与原保守夹爪胶囊间隙降至 1.668276 mm，低于原 2 mm 门槛，因此立即停止。该处源网格没有被诊断为相交，不能写成实际碰撞；保守门槛仍然拒绝这次执行。

独立 `attempt04_grasp_motion_diagnostic01.json` 复现了该间隙：香蕉相对实测 TCP 转动约 61.34°、平移约 10.31 mm；假设保持关闭末态的刚性相对姿态，同一 TCP 路径的胶囊间隙约为 33.40 mm。实际接触还出现短暂单侧失载，说明下一步需要修复夹持稳定性，不能仅抬高抓点来回避门槛。源质量为 0.12 kg，但实际 native COM/inertia 和摩擦冲量没有记录，几何质心及力矩估计不能证明唯一原因。

最后一个动作只执行了 6/8 个子步，正式完整回合审查正确拒绝；独立 partial 诊断只审查已观测前缀，不补造两步、不提升为任务验收。原失败记录误写 `rejected_candidate_executed=false`，它不能覆盖实际状态/动作的已执行证据；新采集器已依据真实子步计数区分规划拒绝和执行中断，旧原件不改写。独立报告、视觉检查和失败日志位于 `reports/replicator_execution_20261023/g_banana/attempt04_*`。三个安全变体尚未开始。

最新的香蕉专属 128/8 配置已完成 `neutral_128_8_01`：60 秒、7201 个逐子步状态、15 项检查全部通过，Kettle 最大平移约 0.7403 mm、旋转约 0.16624°，六视频完整解码。配置由 `banana_background_physics.py` 的独立层实现，继承原冰箱安装和源物理，仅覆盖四个壶体的八个求解器迭代属性；碗入水槽和已认证 G 芥末瓶仍走原配置。选型的两组 10 秒试验均保留：128/8 通过，255/16 失败（最大 4.504 mm / 1.058°），效果非单调，不能称为通用收敛证明。此静置通过尚不代表香蕉抓放成功。

当前 canonical 布局（SHA256 `7a032245402428e2ec49305a96370a86301d836685812c6d95fae4704c66dfb1`）的 `neutral_v2_03` 已实跑 45 秒、5401 个逐物理子步状态，15 项检查中 14 项通过。香蕉和碗稳定，原相机同步与六个视频完整解码通过；背景 KettleA01 四个连接刚体未通过原 2 mm / 1° 门槛：按钮最大平移 5.691 mm、壶盖 5.683 mm，最大旋转约 1.335°。失败原始数据保留在 `reports/replicator_execution_20261023/g_banana/neutral_v2_03/`。不能沿用旧布局的绿色静置报告，也不能将当前新布局标为物理验收通过。

新控制配置 `configs/extensions/replicator/g_shape_banana_bowl_controller.json` 绑定上述源布局、香蕉/碗/厨房资产，以及已声明的抓点上移 13 mm。采集入口已接入运行前代码/配置/资产快照和原任务谓词逐子步记录；相关 44 项 CPU 测试及入口 dry-run 通过。真实 attempt04 已运行并以上述抓持失稳失败，不能把几何或静置通过当作抓放成功。

独立 GPU 诊断保存在 `banana_pair_runtime_actual_gpu01`、`banana_pair_runtime_candidate_gpu01` 和 `banana_payload_layout_paths_gpu01`（均位于 `reports/replicator_preparation/`）。旧 attempt03 真实失败轨迹仍因机器人与台面的原球查询被拒绝；新候选接近路径的原 joint/world/self 成本均为零。新假设抬升路径只有携物球与指定源支撑面的查询非零，机器人对全部环境、携物对其余环境、原关节/自碰撞及全源 FCL 均通过。精确支撑面接触检查仅覆盖这一指定物体对。未绑定真实抓持状态的候选仍返回 `valid=false`，不能据此执行或宣称成功。汇总与输入哈希见 `banana_gpu_query_review01.json`。

旧物体位置在修正基座后完成 45 秒中性观察，全部 15 项检查通过。真实 `base/attempt_03` 已执行 pregrasp、approach 并获得连续双指中部接触，但首段 lift 被原 cuRobo 世界碰撞门禁拒绝，未执行该段。新布局和背景配置已有上文独立中性观察，不能复用旧布局结果。G bowl/sink 使用独立入口，其进度见 `G_BOWL_SINK_CONTROLLER.md`；本文件以下保留候选搜索及旧回合历史，其中“尚未”描述对应当时状态，当前结论以上文为准。

## CPU 检查

```bash
python3 scripts/extensions/replicator/task_collect.py --help
bash scripts/arena_python.sh scripts/extensions/replicator/task_collect.py \
  --output reports/replicator_preparation/banana_collector_cpu_new --dry-run
bash scripts/arena_python.sh scripts/extensions/replicator/audit_banana_controller_cpu.py \
  --output reports/replicator_preparation/banana_controller_geometry_cpu_new.json
bash scripts/arena_python.sh -m pytest \
  tests/extensions/replicator/test_task_controller.py \
  tests/extensions/replicator/test_task_adapter_components.py \
  tests/extensions/replicator/test_next_task_source_relation.py -q
```

本轮 27 项 CPU 测试通过。`banana_controller_geometry_cpu01.json` 保留了第一候选的 approach 失败：原 finger 胶囊比真实源指尖长，碰到台面。修订保留胶囊及原 2 mm 非接触间隙，按源指尖和保守包络共同计算 TCP 高度，另加 1 mm 目标余量。`cpu02` 中 368 个原厨房碰撞体下，初态、pregrasp 和 approach 的 IK 与 FCL 均通过。仅为源初态端点检查，不是动态路径或物理成功。

随后新增独立固定台架审计，发现旧 x=-2.1 的机器人摆位虽满足原 On/NextTo，但台架与其他西侧柜体存在 12 对源几何相交；上述 CPU02 和旧中性记录不能作为合格初态。保留全部 21 个 X-only 候选的 `banana_base_search_cpu01.json`；首个合格候选 x=-1.825 已获主代理确认并写入 canonical。仅移动机器人 X 275 mm，Y/Z、yaw、台架高度、香蕉/碗、源关系和安全间隙均未改变。新报告明确绑定更新后的配置哈希；需重新做中性与物理采集。

## 已执行的真实采集命令

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/task_collect.py \
  --output datasets/replicator/g_shape_banana_bowl/base/attempt_03 \
  --device cpu --physics-threads 2 --torch-threads 2 \
  --headless --seed 42 --max-steps 2400 --episode-length-s 180
```

入口内部对 `/workspace/reports/skill_trials/gpu.lock` 非阻塞持锁，不要再外套同一把锁。要求已核对的 `/workspace` 挂载和已存在的共用锁；资源忙时不启动仿真。输出目录必须不存在，失败不覆盖重试。

物理设备和线程与新中性/首任务验证一致：CPU 物理、2 个 PhysX 线程、2 个 Torch CPU 线程；cuRobo 和相机仍使用 GPU。`runtime_threads` 保存请求值和实际读回值。

### approach 拒绝与修复

`attempt_01` 是启动参数失败，`attempt_02` 保留完整 1737 个物理状态、217 帧及六个可解码视频；已执行段的独立 FCL 无失败。离线从其真实末态重建的 46 个 approach 候选中，只有末点被 cuRobo 世界碰撞拒绝：两个末端指尖球（原 5 mm 缓冲保留）与原 CornerCounter 顶面约 0.23 mm 交叠，关节及自碰撞成本均为零。完整拒绝路径和三组具体球/碰撞体对保存在 `reports/replicator_preparation/banana_attempt02_approach_gpu01/`。

修复把原 URDF 开爪指尖球的最低包络纳入抓点计算，TCP Z 从 0.923899501 m 提高到 0.927119647 m，保留原 2 mm 间隙及 1 mm 目标余量。新高度的 46 点 CPU 路径全 FCL 通过，随后 GPU02 的 joint/world/self 原成本全部为零；attempt03 真实接近和双指闭合通过。源手指 CollisionAPI 子网格与香蕉源三角面扫描中，原耦合闭爪角 0.440–0.470 rad 可以双侧接触中部并维持源几何 2 mm 非接触间隙，但该扫描没有证明闭爪后保守球包络通过，不能据此宣称抬升可用。完整扫描见 `reports/replicator_preparation/banana_corrected_grasp_source_cpu01.json`。

CPU 球诊断还发现当前 FCL 的相交对有时返回极小正 distance；A 新任务适配已对每个近邻对显式调用 `collide()`，包括零间隙的预期目标接触。旧 CPU 报告的“未发现球相交”不能覆盖 GPU 拒绝；旧报告保留追溯。

修复后 GPU 离线诊断入口（无 Kit、无物理回放，内部获取原共用锁）：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/diagnose_task_plan.py \
  --episode datasets/replicator/g_shape_banana_bowl/base/attempt_02 \
  --output reports/replicator_preparation/banana_attempt02_approach_gpu02 \
  --recompute-grasp-target --gpu
```

输出已存在时换新路径，禁止覆盖；上面的 GPU02 和 attempt03 均已执行，不能覆盖重跑。collector 在局部候选被拒时保存 `rejected_plan_XX_phase.npz`、哈希和 joint/world/self 分项门禁，拒绝路径不会执行。

### attempt03 抬升拒绝与离线候选

attempt03 保存 2441 个真实物理状态。关闭后的原内指球最低约 0.8871 m，比台面低约 7.8 mm；携物球也穿入台面。精确保存的 46 点 lift 候选在第 0、1、2 点被世界碰撞拒绝，joint/self 成本为零，全源 FCL 通过。原候选、实测六个指关节及实测 TCP-to-banana 都保留；`banana_attempt03_lift_gpu01/diagnostic.json` 使用真实末态重建，没有执行拒绝段。

CPU 有界搜索只改变抓点高度，保留资产、物体/机器人位置、yaw、抓点 XY、原接触区与所有间隙。当前评估 +13 mm 候选，目标 Z 为 0.940119647 m。从实际 pregrasp 状态生成的 approach 及 0–0.440 rad 的 89 个闭合采样通过 CPU 检查，最小原球间隙 4.306 mm。见 `banana_closed_candidate_cpu02/diagnostic.json`。这不是实际力闭合、摩擦或抬升验证，尚未写入运行时。

复查发现旧 FCL `contact.pos` 偶尔位于香蕉源包围盒外，不能当精确源表面交点判断中部区域。旧搜索报告保留，但其中 +11/+12 mm 的区域拒绝理由已撤回。独立 `banana_source_region_partition_cpu01.json` 把全部 15728 个香蕉源三角形按原中部区域六平面精确裁剪，分区面积与原面积相同；再分别检查手指与 inside/outside 源表面的碰撞，+11/+12/+13 mm 都是仅中部双侧接触。+13 mm 作为已检查且余量较大的候选继续评估，不宣称是最小高度。真实 PhysX 接触记录不受该 CPU 诊断局限影响。

固定源位置仍存在独立携物问题：实际香蕉凸包对相邻 `CornerCounter_00_1` 的间隙仅 2.576 mm，源 FCL 通过但原携物球包络相交。该台面不是指定的 `CornerCounter_00_0` 源支撑，不能扩成整台面豁免。独立诊断把原 CUDA 世界查询精确分成 `robot × all_world`、`payload × non_source_support`、`payload × source_support`，逐样本核对三分区拒绝并集与原完整查询相同；所有非接触对仍须通过原球门槛，joint/self 仍用完整原球。以下仅供主代理排队诊断，当前不授权新实采，也不会输出验收通过：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/diagnose_banana_grasp_candidate.py \
  --episode datasets/replicator/g_shape_banana_bowl/base/attempt_03 \
  --search-report reports/replicator_preparation/banana_closed_envelope_search_cpu01/search_report.json \
  --output reports/replicator_preparation/banana_closed_candidate_gpu01 --gpu
```

GPU01 已确认：approach 原门禁全通过；lift 的 `robot × all_world` 成本为零，`payload × non_source_support` 仍拒绝第 0/1 点。三分区覆盖 66060 个原球/碰撞体组合，逐样本零/非零并集与原完整世界门禁完全相同。该报告证明没有因分解漏掉相邻台面，不是任务通过。

随后预声明并保留 31 个仅 X 候选，发现原 authored 香蕉/碗高度分别比严格 2 mm 差 54.1/26.2 nm；不降低门槛，原失败见 `g_banana_canonical_source_geometry_cpu02.json` 和 `banana_payload_layout_search_cpu01/`。第二次声明仅给香蕉/碗各增加 0.1 mm 初始 Z，仍在原 On 5 mm 带内，然后按 1 mm 步长搜索香蕉 X。首个原关系、源支撑/几何、两种平移预测实测姿态原携物球间隙同时通过的候选是 X +14 mm：

- banana 初始位置 `[-2.246, 2.15, 0.9153344465]`。
- bowl 初始位置 `[-1.81, 2.15, 0.9244844748]`。
- 机器人、源资产、全部 yaw 和原安全门槛不变。

候选原件保存于 `banana_payload_layout_search_cpu02/candidate_layout.json`，SHA256 为 `7a032245402428e2ec49305a96370a86301d836685812c6d95fae4704c66dfb1`，经主代理复核后已推广到 canonical。旧 canonical 原件与所有候选哈希保存在 `reports/replicator_preparation/g_tasks_layout_promotion01/`；推广不等于物理验收。独立 `banana_payload_layout_source_geometry_cpu01.json` 的六组初态检查通过，`banana_payload_layout_initial_robot_cpu01.json` 的全源机器人/保守包络/固定台架通过，最小间隙 10.319 mm。`banana_payload_layout_paths_cpu01/diagnostic.json` 的 pregrasp 端点、46 点 approach、89 点 close、47 点 lift 全源 FCL 通过；源区域分割双指 inside 为 61/15、outside 均为 0；闭爪原球最小间隙 4.309 mm；CPU 携物球只剩指定 `_00_0` 源支撑对。

新路径仍使用旧实际 settled 物体姿态的平移预测，不把 0.1 mm 初始高度增量加到 settled 物体上。初态到 pregrasp 的完整规划、GPU 门禁、新布局中性观察与真实采集尚未完成，不能复用旧布局 neutral 冒充新布局通过。下一离线命令：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/diagnose_banana_layout_candidate.py \
  --episode datasets/replicator/g_shape_banana_bowl/base/attempt_03 \
  --grasp-search reports/replicator_preparation/banana_closed_envelope_search_cpu01/search_report.json \
  --layout-search reports/replicator_preparation/banana_payload_layout_search_cpu02/search_report.json \
  --output reports/replicator_preparation/banana_payload_layout_paths_gpu01 --gpu
```

上述所有候选 TCP-to-banana 都是假设新抓点计算，明确标记 hypothetical，不能替代下次真实闭合后的实测 attachment。原采集器和失败样本保持原样；canonical 推广后仍需新 neutral。独立 `SourceSupportCuroboPlanner` 草稿只处理绑定真实 close 状态后的指定 source-support 对，保留全部其他球门槛、joint/self 和全 FCL，并要求闭爪命令为 1。绑定还校验源支撑 current/reference 顶点、三角形和变换的哈希，不能用相同 AABB 替代同一几何。18 项 CPU 测试通过，代码冻结快照见 `banana_pair_runtime_code_freeze01.json`；尚未完成新模块 GPU 验证、采集器接入或实际执行，不能提前放行实采。

## 控制与碰撞模型

- 原 G 厨房、机器人台架高度/朝向、banana/bowl 源资产及全部 On/NextTo/Rotate 关系不变，使用原 `PickAndPlaceTask`。只有无新增风险 base；新任务三类 hazard 的原安全策略尚未定义，不允许执行。
- 从香蕉原网格的局部 y=0 截面算中部抓点，局部 x 约 -33.03 mm，宽约 38.01 mm。TCP +X 朝下，夹爪闭合轴沿香蕉局部 X 水平投影；没有瓶子的 35° 姿态。
- 中部接近、真实闭合、双内指连续 10 个物理样本受力检查、真实抬升、搬运、源网格计算下降目标、松手、撤离、静置。只在规划中保存实测 `inverse(T_world_TCP) @ T_world_banana`；没有写目标位姿、仿真焊接或替换物体。
- cuRobo 携物采用源香蕉凸包的保守体积球覆盖，最多 100 球。源三角形、FCL 凸包、cuRobo 体积球均有来源/区别记录；尚未验证 PhysX cooked decomposition。
- cuRobo 目前只能整对象开关，候选生成按阶段屏蔽 banana，下降/释放阶段另屏蔽 bowl。这是候选规划器的表示限制，不能当作安全证据。调用者不能传任意 ignore_names。
- 独立 FCL 检查始终恢复全部世界对象：每个稠密路径点、reset 初态以及每个真实物理子步都检查。只放行两个 inner_finger 与 banana 的任务接触；其重复 finger 胶囊/整夹爪包络由同时存在的各部件源凸包细检。手掌、外指节、机械臂与 banana，以及所有机器人部位与台面/碗/厨房均保留检查。
- banana 与自己的世界副本不互检。banana 与原 `CornerCounter_00_0` 的支撑只沿用 A 基线已有的 6 mm 源接触带，并要求完整 XY 处于该台面；不是机器人/台面豁免。预期 banana/bowl 接触阶段用原香蕉三角形复核填满凹部的凸包；真实源三角形相交仍拒绝，机器人/碗不豁免。真实 cooked 接触与源三角形可能不同，这仍是待 GPU 诊断的限制。

## 记录和判定

`physics_states.npz` 从 reset 返回的初态开始，包含 8 次 warmup、5 s settle、全部操作及末态。保存实际 q/dq、全机器人 body pose、banana/bowl root state、原目标支持力、全部机器人接触力、每步双指原始接触、背景刚体及可读取 native articulation 的 q/dq、源静态锚点和目标引用变换。`physics_metadata.json` 明确列顺序；不能直接拿旧 mustard 专用验证器解释这些字段。

原三相机与独立 overview 从第一动作前同步拍摄；`trajectory.npz`/`actions.json` 保存动作及物理步索引。另有 startup 前缀、每段 plan/验证、原成功发生时状态、原源关系审计和六个全解码视频。异常会保留失败结果和已有数据。

`physical_trial_passed` 需要原成功序列、末态原目标支持谓词、实际双指抓持/抬升/释放、全物理证据与安全检查及视频解码；缺少任一项均不能通过。`accepted` 固定为 false，直到独立的等时长中性观察、源/台架、实际 q 自碰撞、相机位姿和画面复查完成。新 schema 的这些独立验收尚未完成，三类增广和三 seed 配对也未开始。

首轮 GPU 重点：环境构建与接触视图、实际 settled 抓点/夹持宽度、cuRobo 闭爪携物自碰撞配置、下降/释放的夹爪与碗间隙、源三角形与 cooked 支撑接触差异。任何失败应保存并诊断，不能减小间隙、屏蔽整物体或宣称成功。
